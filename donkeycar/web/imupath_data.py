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
        """
        Deprecated: segment statistics are now computed on-demand.
        This method is kept for backward compatibility but does nothing.
        """
        # No longer pre-compute segment statistics
        self.segment_rankings = {}
        self.available_ranking_keys = []

    def get_available_fields(self):
        """
        Detect all numeric/vector fields available in the tub.

        Returns:
            List of field metadata dictionaries with keys:
            - name: Field name (e.g., 'imu/gyr')
            - type: Type string from tub (e.g., 'vector', 'float')
            - is_vector: Boolean indicating if field is a vector
            - dimensions: Number of dimensions (None for scalars)
        """
        if not self.is_tub_data or not self.tub_path:
            return []

        try:
            tub = Tub(self.tub_path, read_only=True)
            try:
                fields = []

                # Add built-in computed fields first (time and distance)
                fields.append({
                    'name': 'time',
                    'type': 'float',
                    'is_vector': False,
                    'dimensions': None,
                })
                fields.append({
                    'name': 'distance',
                    'type': 'float',
                    'is_vector': False,
                    'dimensions': None,
                })

                # Iterate through input_types to get field metadata
                for field_name, field_type in tub.input_types.items():
                    # Only include numeric scalars and vectors
                    if field_type in ('float', 'int', 'vector', 'list'):
                        is_vector = field_type in ('vector', 'list')
                        dimensions = None

                        # For vectors, try to determine dimensionality
                        if is_vector:
                            # Read first record to get actual dimensions
                            for record in tub:
                                if (field_name in record and
                                    record[field_name] is not None):
                                    value = record[field_name]
                                    if hasattr(value, '__len__'):
                                        dimensions = len(value)
                                    break

                        fields.append({
                            'name': field_name,
                            'type': field_type,
                            'is_vector': is_vector,
                            'dimensions': dimensions,
                        })

                return fields
            finally:
                tub.close()
        except Exception as e:
            logger.error(f"Failed to detect available fields: {e}")
            return []

    def compute_segment_statistics(self, field_name, method, dimension=None):
        """
        Compute segment statistics on-demand for a specific field/method.

        Uses TubStatistics session rankings for consistency with training.

        Args:
            field_name: Name of the field to aggregate, or built-in metrics
                       'time' or 'distance'
            method: Aggregation method ('delta', 'mean_abs', 'sum_abs',
                   'max', 'min', 'norm'). Ignored for 'time' and 'distance'.
            dimension: For vector fields, which component (0=X, 1=Y, 2=Z,
                      None=compute norm)

        Returns:
            Dictionary mapping lap -> segment -> percentile ranking
        """
        if not self.is_tub_data or not self.tub_path:
            return {}

        # Handle built-in computed metrics (time, distance)
        # These don't need field aggregation - they're already computed
        if field_name in ('time', 'distance'):
            sorting_key = field_name
            field_aggregations = []
        else:
            # Regular tub field - create aggregation spec
            spec = self._build_field_aggregation_spec(
                field_name, method, dimension)
            if spec is None:
                logger.error(f"Unknown aggregation method: {method}")
                return {}
            sorting_key = spec.output_key
            field_aggregations = [spec]

        try:
            tub = Tub(self.tub_path, read_only=True)
            try:
                session_id = self._get_session_id(tub)
                use_lap_0 = self._use_lap_0()
                num_bins = self._count_session_laps(
                    tub, session_id, use_lap_0)
                lap_resolver = None
                segment_resolver = None
                if self._should_use_visual_laps(num_bins):
                    # Use Y-crossing lap detection when car/lap is constant
                    num_bins = self._count_visual_laps(use_lap_0)
                    lap_resolver = self._build_visual_lap_resolver(use_lap_0)
                if self._should_use_visual_segments(
                    tub, session_id):
                    segment_resolver = (
                        self._build_visual_segment_resolver())
                sorting_strategy = SortingStrategy([{'key': sorting_key}])
                stats = TubStatistics(
                    tub,
                    config=self.cfg,
                    sorting_strategy=sorting_strategy,
                    field_aggregations=field_aggregations,
                )
                rankings_by_session = (
                    stats.calculate_segment_performance(
                        use_lap_0,
                        num_bins=num_bins,
                        lap_resolver=lap_resolver,
                        segment_resolver=segment_resolver,
                        session_id=session_id))
                session_rankings = self._select_session_rankings(
                    rankings_by_session, session_id)
                return session_rankings
            finally:
                tub.close()
        except Exception as e:
            logger.error(f"Failed to compute segment statistics: {e}",
                         exc_info=True)
            return {}

    def _build_field_aggregation_spec(self, field_name, method, dimension):
        """Build FieldAggregationSpec for the selected method."""
        output_key = 'computed_stat'
        index = dimension if dimension is not None else None

        if method == 'delta':
            return FieldAggregationSpec(
                field=field_name,
                output_key=output_key,
                index=index,
                aggregation='delta'
            )

        if method == 'mean_abs':
            return FieldAggregationSpec(
                field=field_name,
                output_key=output_key,
                index=index,
                transform=abs,
                aggregation='avg'
            )

        if method == 'sum_abs':
            return FieldAggregationSpec(
                field=field_name,
                output_key=output_key,
                index=index,
                transform=abs,
                aggregation='sum'
            )

        if method == 'max':
            return FieldAggregationSpec(
                field=field_name,
                output_key=output_key,
                index=index,
                aggregation='max'
            )

        if method == 'min':
            return FieldAggregationSpec(
                field=field_name,
                output_key=output_key,
                index=index,
                aggregation='min'
            )

        if method == 'norm':
            return FieldAggregationSpec(
                field=field_name,
                output_key=output_key,
                transform=np.linalg.norm,
                aggregation='avg'
            )

        return None

    def _use_lap_0(self):
        """Determine whether lap 0 should be included in rankings."""
        if self.cfg and hasattr(self.cfg, 'USE_LAP_0'):
            return bool(self.cfg.USE_LAP_0)
        return False

    def _get_session_id(self, tub):
        """Select a session id for ranking context."""
        if tub.manifest.metadata:
            return next(iter(tub.manifest.metadata.keys()))

        for record in tub:
            session_id = record.get('_session_id')
            if session_id:
                return session_id
        return None

    def _count_session_laps(self, tub, session_id, use_lap_0):
        """Count unique laps for session to keep rank bins stable."""
        if not session_id:
            return None

        laps = set()
        for record in tub:
            if record.get('_session_id') != session_id:
                continue
            lap = record.get('car/lap')
            if lap is None:
                continue
            if not use_lap_0 and lap == 0:
                continue
            laps.add(lap)

        if not laps:
            return None
        return len(laps)

    def _should_use_visual_laps(self, lap_count):
        """Check if visual lap detection should be used for rankings."""
        if self.multilap_data.num_laps <= 1:
            return False
        if lap_count is None:
            return True
        return lap_count <= 1

    def _count_visual_laps(self, use_lap_0):
        """Return lap count from visual lap detection."""
        count = self.multilap_data.num_laps
        if count <= 0:
            return None
        if use_lap_0:
            return count
        if count == 1:
            return None
        return count - 1

    def _build_visual_lap_resolver(self, use_lap_0):
        """Create a resolver for lap numbers based on lap boundaries."""
        boundaries = self.multilap_data.lap_boundaries
        if not boundaries:
            return None

        state = {'lap_idx': 0}

        def resolve(record_idx):
            lap_idx = state['lap_idx']
            while lap_idx < len(boundaries):
                boundary = boundaries[lap_idx]
                if record_idx < boundary.start_index:
                    return None
                if record_idx <= boundary.end_index:
                    state['lap_idx'] = lap_idx
                    if not use_lap_0 and lap_idx == 0:
                        return None
                    return lap_idx
                lap_idx += 1
            state['lap_idx'] = lap_idx
            return None

        return resolve

    def _count_segment_cycle_laps(self, use_lap_0):
        """
        Return lap count based on segment cycle completions (4->0 transitions).

        For segment statistics, laps are defined by segment cycles, not
        Y-crossing detection.
        """
        if self.segment_ids is None or len(self.segment_ids) == 0:
            return None

        num_segments = self.segmentation.num_segments
        if num_segments <= 1:
            return None

        last_segment = num_segments - 1

        # Count segment cycle completions
        cycle_count = 0
        for i in range(1, len(self.segment_ids)):
            if (self.segment_ids[i-1] == last_segment and
                self.segment_ids[i] == 0):
                cycle_count += 1

        if cycle_count <= 0:
            return None
        if use_lap_0:
            return cycle_count
        if cycle_count == 1:
            return None
        return cycle_count - 1

    def _build_segment_cycle_lap_resolver(self, use_lap_0):
        """
        Create a resolver for lap numbers based on segment cycle boundaries.

        Laps are defined by segment cycle completions (4->0 transitions), not
        Y-crossing detection. This ensures segment statistics use consistent
        lap definitions.
        """
        if self.segment_ids is None or len(self.segment_ids) == 0:
            return None

        num_segments = self.segmentation.num_segments
        if num_segments <= 1:
            return None

        last_segment = num_segments - 1

        # Find segment cycle boundaries (indices where 4->0 transition occurs)
        cycle_indices = []
        for i in range(1, len(self.segment_ids)):
            if (self.segment_ids[i-1] == last_segment and
                self.segment_ids[i] == 0):
                cycle_indices.append(i)

        if not cycle_indices:
            return None

        # Build lap boundaries for complete laps only
        # - Lap 0 starts at index 0
        # - Each subsequent complete lap starts at a cycle boundary
        # - Exclude the last cycle if it doesn't complete (partial lap)
        # We have N cycles, which means laps 0 through N-1 are complete
        # Lap N (if it exists) is partial and should be excluded
        lap_starts = [0] + cycle_indices[:-1] if len(cycle_indices) > 1 else [0]
        # Last complete lap ends just before the last cycle starts
        last_complete_lap_end = cycle_indices[-1] - 1 if cycle_indices else len(
            self.segment_ids) - 1

        state = {'lap_idx': 0}

        def resolve(record_idx):
            lap_idx = state['lap_idx']

            # Find which lap this record belongs to
            while lap_idx < len(lap_starts):
                lap_start = lap_starts[lap_idx]

                # Determine lap end
                if lap_idx + 1 < len(lap_starts):
                    # Not the last lap: ends one index before next lap starts
                    lap_end = lap_starts[lap_idx + 1] - 1
                else:
                    # Last complete lap: ends at last_complete_lap_end
                    lap_end = last_complete_lap_end

                if record_idx < lap_start:
                    return None

                if lap_start <= record_idx <= lap_end:
                    state['lap_idx'] = lap_idx
                    if not use_lap_0 and lap_idx == 0:
                        return None
                    return lap_idx

                lap_idx += 1

            # Record is beyond last complete lap (trailing partial lap)
            state['lap_idx'] = lap_idx
            return None

        return resolve

    def _should_use_visual_segments(self, tub, session_id):
        """Check if visual segment ids should be used for ranking."""
        if self.segmentation.num_segments <= 1:
            return False
        segment_count = self._count_session_segments(tub, session_id)
        return segment_count is None or segment_count <= 1

    def _count_session_segments(self, tub, session_id):
        """Count unique segments for session."""
        if not session_id:
            return None

        segments = set()
        for record in tub:
            if record.get('_session_id') != session_id:
                continue
            segment = record.get('car/segment')
            if segment is None:
                continue
            segments.add(segment)
        if not segments:
            return None
        return len(segments)

    def _build_visual_segment_resolver(self):
        """Create a resolver for segment ids based on visual assignment."""
        if self.segment_ids is None or len(self.segment_ids) == 0:
            return None

        def resolve(record_idx):
            if record_idx < 0 or record_idx >= len(self.segment_ids):
                return None
            return int(self.segment_ids[record_idx])

        return resolve

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
            # heading is in math coordinates (0° = right/+X, 90° = forward/+Y)
            # Convert back to IMU yaw for display (0° = forward/+Y)
            heading_rad = float(self.path_data.heading[idx])
            heading_deg = np.degrees(heading_rad)
            imu_yaw_deg = 90.0 - heading_deg
            # Normalize to [0, 360)
            imu_yaw_deg = imu_yaw_deg % 360.0

            point = {
                't': float(self.path_data.timestamp[idx]),
                'x': float(self.path_data.x[idx]),
                'y': float(self.path_data.y[idx]),
                'v': float(self.path_data.velocity[idx]),
                'h': heading_rad,  # Math heading for geometric calculations
                'imu_yaw': np.radians(imu_yaw_deg),  # IMU yaw for display
                'd': float(self.path_data.distance[idx]),
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
