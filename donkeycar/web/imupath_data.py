"""
IMU Path Data Preparation for Web UI

This module provides data preparation utilities for the web-based IMU path
visualizer. It reuses the existing course_analysis pipeline to generate
JSON-ready data payloads.

The data model includes:
- Path points: timestamp, x, y, velocity, heading, lap
- Mean course: x, y points
- Segments: boundaries, labels
- Rankings: segment performance statistics (for Tub data)
- Metadata: lap_method, segment_method, min_loop_distance, num_laps
"""

import logging
import numpy as np
import pandas as pd
from typing import Dict, List, Optional, Any

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
    PathData,
)
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics

logger = logging.getLogger(__name__)


def downsample_points(points: List[Dict], max_points: int) -> List[Dict]:
    """
    Downsample points uniformly to meet max_points limit.
    
    Args:
        points: List of point dictionaries
        max_points: Maximum number of points to return
        
    Returns:
        Downsampled list of points
    """
    if len(points) <= max_points:
        return points
    
    step = len(points) / max_points
    indices = [int(i * step) for i in range(max_points)]
    return [points[i] for i in indices]


class IMUPathDataPreparation:
    """
    Prepares IMU path data for web visualization.
    
    This class orchestrates the course_analysis pipeline and converts
    results into JSON-ready format for the web UI.
    """
    
    def __init__(
        self,
        path_data: PathData,
        cfg: Optional[Any] = None,
        lap_method: str = 'y_crossing',
        segment_method: str = 'gradient',
        num_laps: Optional[int] = None,
        tub_path: Optional[str] = None
    ):
        """
        Initialize data preparation.
        
        Args:
            path_data: PathData object with position/velocity data
            cfg: Configuration object with parameters
            lap_method: Lap detection method ('y_crossing' or 'drift')
            segment_method: Segmentation method
                ('threshold', 'extrema', 'gradient', 'hybrid')
            num_laps: Number of laps for mean course (None = all)
            tub_path: Path to Tub directory (for segment stats)
        """
        self.path_data = path_data
        self.cfg = cfg
        self.lap_method = lap_method
        self.segment_method = segment_method
        self.num_laps = num_laps
        self.tub_path = tub_path
        
        # Processed data
        self.multilap_data = None
        self.mean_course = None
        self.segmentation = None
        self.segment_ids = None
        self.segment_rankings = {}
        self.available_ranking_keys = []
        
        # Run pipeline
        self._process_data()
    
    def _process_data(self):
        """Run the complete data processing pipeline."""
        logger.info("Processing IMU path data...")
        
        # Phase 2: Lap Detection
        if self.lap_method == 'y_crossing':
            detector = YCrossingLapDetector(cfg=self.cfg)
        else:
            detector = DriftLapDetector(cfg=self.cfg)
        
        lap_boundaries = detector.detect_laps(self.path_data)
        self.multilap_data = MultiLapData(self.path_data, lap_boundaries)
        
        # Use specified num_laps or all detected laps
        laps_for_mean = (
            self.num_laps if self.num_laps is not None
            else self.multilap_data.num_laps
        )
        
        logger.info(f"Detected {self.multilap_data.num_laps} laps, "
                   f"using {laps_for_mean} for mean course")
        
        # Phase 3: Mean Course Building
        if laps_for_mean < self.multilap_data.num_laps:
            limited_boundaries = self.multilap_data.lap_boundaries[
                :laps_for_mean]
            limited_data = MultiLapData(
                self.multilap_data.path_data, limited_boundaries)
        else:
            limited_data = self.multilap_data
        
        builder = MeanCourseBuilder(cfg=self.cfg)
        self.mean_course = builder.build(limited_data)
        
        logger.info(f"Built mean course, length: {self.mean_course.length:.1f}m")
        
        # Phase 4: Segmentation
        if self.segment_method == 'threshold':
            strategy = ThresholdSegmentation()
        elif self.segment_method == 'extrema':
            strategy = ExtremaSegmentation()
        elif self.segment_method == 'gradient':
            strategy = GradientSegmentation()
        else:  # hybrid
            strategy = HybridSegmentation()
        
        segmenter = CourseSegmenter(strategy, cfg=self.cfg)
        self.segmentation = segmenter.segment(self.mean_course)
        
        logger.info(f"Segmented course using {self.segment_method}, "
                   f"found {self.segmentation.num_segments} segments")
        
        # Phase 5: Segment Assignment
        assigner = SegmentAssigner(self.segmentation)
        self.segment_ids = assigner.assign(
            self.path_data.x, self.path_data.y)
        
        logger.info("Assigned segments to driven path")
        
        # Load segment statistics if Tub data
        if self.tub_path:
            self._load_segment_statistics()
    
    def _load_segment_statistics(self):
        """Load segment performance statistics from Tub data."""
        if not self.tub_path or not self.segment_ids.any():
            return
        
        try:
            logger.info("Computing segment statistics...")
            tub = Tub(self.tub_path, read_only=True)
            
            try:
                stats = TubStatistics(tub, config=self.cfg)
                
                # Compute segment rankings using lap boundaries and segment IDs
                from collections import defaultdict
                from donkeycar.parts.tub_statistics import FieldAccumulator
                
                segment_instances = defaultdict(list)
                lap_end_indices = {
                    b.end_index for b in self.multilap_data.lap_boundaries
                }
                
                state = {
                    'lap': 0,
                    'segment': self.segment_ids[0] if len(self.segment_ids) > 0 else None,
                    'start_time': None,
                    'start_dist': 0.0,
                    'field_accumulators': [
                        FieldAccumulator(spec) for spec in stats.field_aggregations
                    ]
                }
                
                last_timestamp_ms = 0
                last_distance = 0.0
                max_idx = self.multilap_data.lap_boundaries[-1].end_index if self.multilap_data.lap_boundaries else None
                
                for idx, record in enumerate(tub):
                    if max_idx is not None and idx > max_idx:
                        break
                    if idx >= len(self.segment_ids):
                        break
                    
                    segment = self.segment_ids[idx]
                    timestamp_ms = record.get('_timestamp_ms', 0)
                    distance = record.get('car/distance', 0.0)
                    
                    last_timestamp_ms = timestamp_ms
                    last_distance = distance
                    
                    # Initialize start values
                    if state['start_time'] is None:
                        state['start_time'] = timestamp_ms
                        state['start_dist'] = distance
                    
                    # Accumulate field values
                    for accumulator in state['field_accumulators']:
                        accumulator.accumulate(record)
                    
                    # Check for segment or lap change
                    if segment != state['segment'] or idx in lap_end_indices:
                        # Finalize current segment
                        time_ms = timestamp_ms - state['start_time']
                        dist = distance - state['start_dist']
                        
                        metrics = {'time': time_ms, 'distance': dist}
                        for accumulator in state['field_accumulators']:
                            metrics[accumulator.output_key] = accumulator.finalize()
                        
                        key = (state['lap'], state['segment'])
                        segment_instances[key].append(metrics)
                        
                        # Reset for next segment
                        state['segment'] = segment
                        state['start_time'] = timestamp_ms
                        state['start_dist'] = distance
                        state['field_accumulators'] = [
                            FieldAccumulator(spec) for spec in stats.field_aggregations
                        ]
                        
                        # Check for lap change
                        if idx in lap_end_indices:
                            state['lap'] += 1
                
                # Finalize last segment
                if state['start_time'] is not None:
                    time_ms = last_timestamp_ms - state['start_time']
                    dist = last_distance - state['start_dist']
                    
                    metrics = {'time': time_ms, 'distance': dist}
                    for accumulator in state['field_accumulators']:
                        metrics[accumulator.output_key] = accumulator.finalize()
                    
                    key = (state['lap'], state['segment'])
                    segment_instances[key].append(metrics)
                
                # Compute rankings
                active_laps = len(self.multilap_data.lap_boundaries)
                self.segment_rankings = self._compute_rankings(
                    segment_instances, stats.sorting_strategy, active_laps
                )
                
                # Get available ranking keys
                if self.segment_rankings:
                    first_key = next(iter(self.segment_rankings.keys()))
                    first_metrics = self.segment_rankings[first_key]
                    self.available_ranking_keys = [
                        k for k in first_metrics.keys() if k.endswith('_pct')
                    ]
                
                logger.info(f"Loaded segment rankings for "
                          f"{len(self.segment_rankings)} data points")
                logger.info(f"Available metrics: "
                          f"{', '.join(self.available_ranking_keys)}")
                
            finally:
                tub.close()
                
        except Exception as e:
            logger.error(f"Failed to compute segment statistics: {e}")
            self.segment_rankings = {}
            self.available_ranking_keys = []
    
    def _compute_rankings(
        self,
        segment_instances: Dict,
        sorting_strategy: Any,
        active_laps: int
    ) -> Dict:
        """Compute percentile rankings for segment instances."""
        from donkeycar.pipeline.transformations import compute_rankings
        
        rankings = {}
        
        # For each segment
        for segment_id in range(self.segmentation.num_segments):
            # Collect instances across laps
            instances = []
            for lap in range(active_laps):
                key = (lap, segment_id)
                if key in segment_instances:
                    instances.extend(segment_instances[key])
            
            if not instances:
                continue
            
            # Compute rankings
            ranked = compute_rankings(instances, sorting_strategy)
            
            # Map back to original structure
            lap_idx = 0
            for lap in range(active_laps):
                key = (lap, segment_id)
                if key in segment_instances:
                    for inst in segment_instances[key]:
                        rankings_key = f"lap_{lap}_seg_{segment_id}"
                        rankings[rankings_key] = ranked[lap_idx] if lap_idx < len(ranked) else {}
                        lap_idx += 1
        
        return rankings
    
    def get_data_payload(
        self,
        max_display_points: int = 1000
    ) -> Dict[str, Any]:
        """
        Generate JSON-ready data payload for web UI.
        
        Args:
            max_display_points: Maximum number of points to include
            
        Returns:
            Dictionary containing all visualization data
        """
        # Build lap assignments
        lap_assignments = np.zeros(len(self.path_data.x), dtype=int)
        for lap_idx, boundary in enumerate(self.multilap_data.lap_boundaries):
            lap_assignments[boundary.start_index:boundary.end_index + 1] = lap_idx
        
        # Build full path data (keep all for stats)
        full_path_points = []
        for i in range(len(self.path_data.x)):
            full_path_points.append({
                't': float(self.path_data.timestamp[i]),
                'x': float(self.path_data.x[i]),
                'y': float(self.path_data.y[i]),
                'v': float(self.path_data.velocity[i]),
                'h': float(self.path_data.heading[i]),
                'lap': int(lap_assignments[i]),
                'segment': int(self.segment_ids[i])
            })
        
        # Downsample for display
        display_points = downsample_points(
            full_path_points, max_display_points)
        
        # Mean course data
        mean_course_points = [
            {
                'x': float(self.mean_course.x[i]),
                'y': float(self.mean_course.y[i])
            }
            for i in range(len(self.mean_course.x))
        ]
        
        # Segment data
        segments = []
        for i in range(self.segmentation.num_segments):
            boundary = self.segmentation.segment_boundaries[i]
            segments.append({
                'id': i,
                'start_idx': int(boundary.start_index),
                'end_idx': int(boundary.end_index),
                'x': float(self.mean_course.x[boundary.start_index]),
                'y': float(self.mean_course.y[boundary.start_index]),
                'label': f"S{i}"
            })
        
        # Rankings data
        rankings = {}
        for key, value in self.segment_rankings.items():
            rankings[key] = {k: float(v) for k, v in value.items()}
        
        # Metadata
        metadata = {
            'lap_method': self.lap_method,
            'segment_method': self.segment_method,
            'num_laps': self.multilap_data.num_laps,
            'num_segments': self.segmentation.num_segments,
            'total_distance': float(self.path_data.total_distance),
            'duration': float(self.path_data.duration),
            'available_ranking_keys': self.available_ranking_keys,
            'is_tub_data': self.tub_path is not None
        }
        
        return {
            'path_points': display_points,
            'full_path_points': full_path_points,  # Keep for stats
            'mean_course': mean_course_points,
            'segments': segments,
            'rankings': rankings,
            'metadata': metadata
        }
