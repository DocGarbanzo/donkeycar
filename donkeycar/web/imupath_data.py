"""
IMU Path Data Preparation for Web UI

This module prepares JSON-ready data payloads for the web-based IMU path
visualizer, reusing the existing course analysis stack from the matplotlib UI.
"""

import numpy as np
import logging

from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import (
    FieldAggregationSpec,
    TubStatistics,
)
from donkeycar.pipeline.transformations import SortingStrategy
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

SUPPORTED_AGGREGATIONS = ('avg', 'sum', 'min', 'max', 'median')


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
            tub = Tub(self.tub_path, read_only=True)
            try:
                stats = TubStatistics(tub, config=self.cfg)
                expanded_specs = self._expand_field_aggregations(
                    stats.field_aggregations)
                ranking_strategy = self._build_ranking_strategy(
                    stats.sorting_strategy, expanded_specs)
                expanded_stats = TubStatistics(
                    tub,
                    config=self.cfg,
                    field_aggregations=expanded_specs,
                    sorting_strategy=ranking_strategy,
                )
                self.segment_rankings = self._select_session_rankings(
                    expanded_stats.calculate_segment_performance(),
                    tub.manifest.session_id[1],
                )
                self.available_ranking_keys = [
                    criterion['key'] for criterion
                    in ranking_strategy.criteria
                ]
                logger.info("Available ranking keys: %s",
                           ", ".join(self.available_ranking_keys))
            finally:
                tub.close()
        except Exception as e:
            logger.error(f"Failed to load segment statistics: {e}")
            self.segment_rankings = {}
            self.available_ranking_keys = []

    def _expand_field_aggregations(self, field_specs):
        """Expand configured field specs to all supported aggregations."""
        expanded = []
        seen_keys = set()
        for spec in field_specs:
            for aggregation in SUPPORTED_AGGREGATIONS:
                output_key = spec.output_key
                if aggregation != spec.aggregation:
                    output_key = f"{spec.output_key}_{aggregation}"
                if output_key in seen_keys:
                    continue
                expanded.append(FieldAggregationSpec(
                    field=spec.field,
                    output_key=output_key,
                    index=spec.index,
                    transform=spec.transform,
                    aggregation=aggregation,
                ))
                seen_keys.add(output_key)
        return expanded

    def _build_ranking_strategy(self, sorting_strategy, field_specs):
        """Build ranking strategy with all available field aggregations."""
        criteria = []
        known_keys = set()
        for criterion in sorting_strategy.criteria:
            criteria.append({
                'key': criterion['key'],
                'transform': criterion.get('transform'),
                'reverse': criterion.get('reverse', False),
            })
            known_keys.add(criterion['key'])
        for spec in field_specs:
            if spec.output_key in known_keys:
                continue
            criteria.append({'key': spec.output_key})
            known_keys.add(spec.output_key)
        return SortingStrategy(criteria)

    def _select_session_rankings(self, rankings_by_session, session_id):
        """Select rankings for the active session, fallback to first."""
        if not rankings_by_session:
            return {}
        if not session_id:
            return next(iter(rankings_by_session.values()))
        return rankings_by_session.get(
            session_id, next(iter(rankings_by_session.values())))
    
    def _downsample_points(self, indices, max_points=1000):
        """
        Downsample point indices for display.
        
        Args:
            indices: Array of indices to downsample
            max_points: Maximum number of points to return
            
        Returns:
            Downsampled array of indices (unique, no duplicates)
        """
        # No downsampling needed if we have fewer points than max
        if len(indices) <= max_points:
            return indices
        
        # Uniform downsampling using linspace to avoid out-of-bounds indices
        # linspace with integer dtype produces unique indices (no duplicates)
        downsampled = np.linspace(
            0, len(indices) - 1, num=max_points, dtype=int)
        return downsampled
    
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
                # Defensive check: segment_ids should match path_data length,
                # but guard against edge cases during initialization
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
                'type': segment.segment_type.name,
            })

        # Build segment boundaries (for drawing normal lines)
        segment_boundaries = []
        for boundary in self.segmentation.segment_boundaries:
            point = boundary['point']
            normal = boundary['normal']
            # Create line endpoints (extend normal in both directions)
            line_length = 0.3  # meters
            x1 = float(point[0] - normal[0] * line_length)
            y1 = float(point[1] - normal[1] * line_length)
            x2 = float(point[0] + normal[0] * line_length)
            y2 = float(point[1] + normal[1] * line_length)

            segment_boundaries.append({
                'point': {'x': float(point[0]), 'y': float(point[1])},
                'normal': {'x': float(normal[0]), 'y': float(normal[1])},
                'line': {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2},
                'segment_from': int(boundary['segment_from']),
                'segment_to': int(boundary['segment_to']),
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
        if self.is_tub_data:
            rankings = {
                'available': bool(self.segment_rankings),
                'fields': self.available_ranking_keys,
                'segments': self.segment_rankings,
            }
        
        return {
            'path_points': path_points,
            'mean_course': mean_course_points,
            'segments': segments,
            'segment_boundaries': segment_boundaries,
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
