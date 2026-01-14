"""
IMU Path Data Preparation for Web UI

This module prepares JSON-ready data payloads for the web-based IMU path
visualizer, reusing the existing course analysis stack from the matplotlib UI.
"""

import numpy as np
import logging
from typing import Dict, List, Any, Optional

from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics
from donkeycar.course_analysis import (
    YCrossingLapDetector,
    DriftLapDetector,
    MultiLapData,
    MeanCourseBuilder,
    CourseSegmenter,
    SegmentAssigner,
    ThresholdSegmentation,
    ExtremaSegmentation,
    GradientSegmentation,
    HybridSegmentation,
)

logger = logging.getLogger(__name__)


class IMUPathDataBuilder:
    """
    Builds JSON-ready data payloads for web-based IMU path visualization.
    
    Mirrors the logic from InteractiveIMUVisualizer but outputs JSON instead
    of matplotlib visualizations.
    """
    
    def __init__(self, path_data, cfg=None, lap_method='y_crossing',
                 segment_method='gradient', tub_path=None):
        """
        Initialize the data builder.
        
        Args:
            path_data: PathData object with immutable position/velocity data
            cfg: Configuration object with parameters
            lap_method: Lap detection method ('y_crossing' or 'drift')
            segment_method: Segmentation method
                ('threshold', 'extrema', 'gradient', 'hybrid')
            tub_path: Path to Tub directory if source is Tub
                (for segment stats)
        """
        self.path_data = path_data
        self.cfg = cfg
        self.lap_method = lap_method
        self.segment_method = segment_method
        self.tub_path = tub_path
        self.is_tub_data = (tub_path is not None)
        
        # Data processing results
        self.multilap_data = None
        self.mean_course = None
        self.segmentation = None
        self.segment_ids = None
        self.num_laps_for_mean = None
        
        # Segment statistics
        self.segment_rankings = {}
        self.available_ranking_keys = []
        
        # Initialize pipeline
        self._initialize_data_pipeline()
        
        # Load segment statistics if Tub data
        if self.is_tub_data:
            self._load_segment_statistics()
    
    def _initialize_data_pipeline(self):
        """Initialize the complete data processing pipeline."""
        logger.info("Initializing data processing pipeline...")
        
        # Phase 2: Lap Detection
        if self.lap_method == 'y_crossing':
            detector = YCrossingLapDetector(cfg=self.cfg)
        else:
            detector = DriftLapDetector(cfg=self.cfg)
        
        lap_boundaries = detector.detect_laps(self.path_data)
        self.multilap_data = MultiLapData(self.path_data, lap_boundaries)
        self.num_laps_for_mean = self.multilap_data.num_laps
        
        logger.info(f"Detected {self.multilap_data.num_laps} laps")
        
        # Phase 3: Mean Course Building
        self._rebuild_mean_course()
    
    def _rebuild_mean_course(self, num_laps=None):
        """Rebuild mean course from current lap selection."""
        if num_laps is None:
            num_laps = self.num_laps_for_mean
        
        # Create limited MultiLapData if needed
        if num_laps < self.multilap_data.num_laps:
            limited_boundaries = self.multilap_data.lap_boundaries[:num_laps]
            limited_data = MultiLapData(
                self.multilap_data.path_data, limited_boundaries)
        else:
            limited_data = self.multilap_data
        
        # Build mean course
        builder = MeanCourseBuilder(cfg=self.cfg)
        self.mean_course = builder.build(limited_data)
        
        logger.info(f"Built mean course from {num_laps} laps, "
                   f"length: {self.mean_course.length:.1f}m")
        
        # Rebuild segmentation
        self._rebuild_segmentation()
    
    def _rebuild_segmentation(self, segment_method=None):
        """Rebuild segmentation using specified or current method."""
        if segment_method is None:
            segment_method = self.segment_method
        
        # Create segmentation strategy
        if segment_method == 'threshold':
            strategy = ThresholdSegmentation()
        elif segment_method == 'extrema':
            strategy = ExtremaSegmentation()
        elif segment_method == 'gradient':
            strategy = GradientSegmentation()
        else:  # hybrid
            strategy = HybridSegmentation()
        
        # Segment the mean course
        segmenter = CourseSegmenter(strategy, cfg=self.cfg)
        self.segmentation = segmenter.segment(self.mean_course)
        
        logger.info(f"Segmented course using {segment_method}, "
                   f"found {self.segmentation.num_segments} segments")
        
        # Assign segments to full driven path
        assigner = SegmentAssigner(self.segmentation)
        self.segment_ids = assigner.assign(
            self.path_data.x, self.path_data.y)
        
        logger.info("Assigned segments to driven path")
    
    def _load_segment_statistics(self):
        """Load segment rankings if Tub data available."""
        if not self.is_tub_data or not self.tub_path:
            return
        
        try:
            logger.info("Computing segment statistics...")
            # Use TubStatistics to compute rankings
            tub = Tub(self.tub_path, read_only=True)
            try:
                stats = TubStatistics(tub, config=self.cfg)
                # Get available ranking keys from field aggregations
                self.available_ranking_keys = [
                    spec.output_key for spec in stats.field_aggregations
                ]
                logger.info(f"Available ranking keys: "
                          f"{', '.join(self.available_ranking_keys)}")
            finally:
                tub.close()
        except Exception as e:
            logger.error(f"Failed to load segment statistics: {e}")
            self.segment_rankings = {}
            self.available_ranking_keys = []
    
    def _downsample_points(self, indices, max_points=1000):
        """
        Downsample point indices for display.
        
        Args:
            indices: Array of indices to downsample
            max_points: Maximum number of points to return
            
        Returns:
            Downsampled array of indices
        """
        if len(indices) <= max_points:
            return indices
        
        # Uniform downsampling
        step = len(indices) / max_points
        downsampled = [int(i * step) for i in range(max_points)]
        return np.array(downsampled)
    
    def build_json_payload(self, num_laps=None, segment_method=None,
                          max_display_points=1000):
        """
        Build complete JSON payload for web visualization.
        
        Args:
            num_laps: Number of laps for mean course (None = all)
            segment_method: Segmentation method to use (None = current)
            max_display_points: Maximum points to include in display arrays
            
        Returns:
            Dictionary ready for JSON serialization
        """
        # Rebuild if parameters changed
        if num_laps is not None and num_laps != self.num_laps_for_mean:
            self._rebuild_mean_course(num_laps)
            self.num_laps_for_mean = num_laps
        
        if segment_method is not None and segment_method != self.segment_method:
            self._rebuild_segmentation(segment_method)
            self.segment_method = segment_method
        
        # Get downsampling config from cfg if available
        if self.cfg and hasattr(self.cfg, 'IMU_VISUALIZATION_PARAMS'):
            max_display_points = self.cfg.IMU_VISUALIZATION_PARAMS.get(
                'max_display_points', max_display_points)
        
        # Build path points (downsampled for display)
        display_indices = self._downsample_points(
            np.arange(len(self.path_data.timestamp)), max_display_points)
        
        path_points = []
        for idx in display_indices:
            point = {
                't': float(self.path_data.timestamp[idx]),
                'x': float(self.path_data.x[idx]),
                'y': float(self.path_data.y[idx]),
                'v': float(self.path_data.velocity[idx]),
                'h': float(self.path_data.heading[idx]),
                'lap': self._find_lap_for_index(int(idx)),
                'segment': int(self.segment_ids[idx]) if idx < len(
                    self.segment_ids) else None,
            }
            path_points.append(point)
        
        # Build mean course
        mean_course_points = [
            {'x': float(x), 'y': float(y)}
            for x, y in zip(self.mean_course.x, self.mean_course.y)
        ]
        
        # Build segments
        segments = []
        for i, segment in enumerate(self.segmentation.segments):
            segments.append({
                'id': i,
                'start_idx': int(segment.start_index),
                'end_idx': int(segment.end_index),
                'label': f"Seg {i}",
                'type': segment.type.name,
            })
        
        # Build metadata
        metadata = {
            'lap_method': self.lap_method,
            'segment_method': self.segment_method,
            'num_laps': self.num_laps_for_mean,
            'total_laps': self.multilap_data.num_laps,
            'total_points': len(self.path_data.timestamp),
            'display_points': len(path_points),
            'num_segments': self.segmentation.num_segments,
            'mean_course_length': float(self.mean_course.length),
            'total_distance': float(self.path_data.total_distance),
            'duration': float(self.path_data.duration),
            'is_tub_data': self.is_tub_data,
            'available_stats': self.available_ranking_keys,
        }
        
        # Build rankings (if available)
        rankings = {}
        if self.is_tub_data and self.segment_rankings:
            # Note: This is a simplified version - full implementation
            # would require computing segment rankings on-the-fly
            # For now, just indicate which stats are available
            rankings = {
                'available': True,
                'fields': self.available_ranking_keys,
            }
        
        return {
            'path_points': path_points,
            'mean_course': mean_course_points,
            'segments': segments,
            'metadata': metadata,
            'rankings': rankings,
        }
    
    def _find_lap_for_index(self, idx):
        """Find which lap contains the given index."""
        boundaries = self.multilap_data.lap_boundaries
        for lap_idx, boundary in enumerate(boundaries):
            if boundary.start_index <= idx <= boundary.end_index:
                return lap_idx
        # After last boundary
        return len(boundaries)


def prepare_imupath_data(path_data, cfg=None, lap_method='y_crossing',
                        segment_method='gradient', tub_path=None,
                        num_laps=None, max_display_points=1000):
    """
    Convenience function to prepare IMU path data for web visualization.
    
    Args:
        path_data: PathData object
        cfg: Configuration object
        lap_method: Lap detection method
        segment_method: Segmentation method
        tub_path: Path to Tub directory
        num_laps: Number of laps for mean course
        max_display_points: Maximum points for display
        
    Returns:
        JSON-ready dictionary
    """
    builder = IMUPathDataBuilder(
        path_data=path_data,
        cfg=cfg,
        lap_method=lap_method,
        segment_method=segment_method,
        tub_path=tub_path
    )
    
    return builder.build_json_payload(
        num_laps=num_laps,
        segment_method=segment_method,
        max_display_points=max_display_points
    )
