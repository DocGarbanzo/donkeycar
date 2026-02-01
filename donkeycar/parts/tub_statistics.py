from collections import defaultdict
from operator import itemgetter
import logging
from copy import copy
from dataclasses import dataclass
from typing import Optional, List, Dict, Callable, Any

import numpy as np

from donkeycar.parts.tub_v2 import Tub
from donkeycar.pipeline.transformations import (
    SortingStrategy,
    default_lap_sorting_strategy
)
from donkeycar.course_analysis import (
    TubPathDataSource,
    YCrossingLapDetector,
    DriftLapDetector,
    MultiLapData,
    MeanCourseBuilder,
    CourseSegmenter,
    HybridSegmentation,
    ThresholdSegmentation,
    ExtremaSegmentation,
    GradientSegmentation,
    get_or_compute_segment_id,
)

logger = logging.getLogger(__name__)


class SegmentTracker:
    """Tracks segment state during iteration to avoid nesting."""

    def __init__(self, field_aggregations, use_lap_0):
        self.field_aggregations = field_aggregations
        self.use_lap_0 = use_lap_0
        self.current_session = None
        self.current_lap = None
        self.current_segment = None
        self.segment_start_time = None
        self.segment_start_dist = None
        self.field_accumulators = self._create_accumulators()

    def _create_accumulators(self):
        return {
            spec.output_key: FieldAccumulator(spec.aggregation)
            for spec in self.field_aggregations
        }

    def should_skip(self, record, lap=None):
        """Check if record should be skipped (lap 0 filter only)."""
        # Note: Don't check car/segment here - it's computed on-the-fly
        # from manifest metadata if not present in record
        if lap is None:
            lap = record.get('car/lap')
        if not self.use_lap_0 and lap == 0:
            return True

        return False

    def handle_session_change(self, session_id, lap, segment, timestamp_ms,
                             distance, segment_instances,
                             finalize_callback):
        """Handle session change, return True if changed."""
        if session_id == self.current_session:
            return False

        finalize_callback(segment_instances, self.current_session,
                         self.current_lap, self.current_segment,
                         self.segment_start_time, self.segment_start_dist,
                         timestamp_ms, distance, self.field_accumulators)

        self.current_session = session_id
        self.current_lap = lap
        self.current_segment = segment
        self.segment_start_time = timestamp_ms
        self.segment_start_dist = distance
        self.field_accumulators = self._create_accumulators()
        return True

    def handle_lap_change(self, lap, segment, timestamp_ms, distance,
                         segment_instances, finalize_callback):
        """Handle lap change, return True if changed."""
        if lap == self.current_lap:
            return False

        finalize_callback(segment_instances, self.current_session,
                         self.current_lap, self.current_segment,
                         self.segment_start_time, self.segment_start_dist,
                         timestamp_ms, distance, self.field_accumulators)

        self.current_lap = lap
        self.current_segment = segment
        self.segment_start_time = timestamp_ms
        self.segment_start_dist = distance
        self.field_accumulators = self._create_accumulators()
        return True

    def handle_segment_change(self, segment, timestamp_ms, distance,
                             segment_instances, finalize_callback):
        """Handle segment change, return True if changed."""
        if segment == self.current_segment:
            return False

        finalize_callback(segment_instances, self.current_session,
                         self.current_lap, self.current_segment,
                         self.segment_start_time, self.segment_start_dist,
                         timestamp_ms, distance, self.field_accumulators)

        self.current_segment = segment
        self.segment_start_time = timestamp_ms
        self.segment_start_dist = distance
        self.field_accumulators = self._create_accumulators()
        return True

    def accumulate_fields(self, record):
        """Accumulate field values for current segment."""
        for spec in self.field_aggregations:
            value = spec.extract(record)
            if value is None:
                continue
            self.field_accumulators[spec.output_key].add(value)


@dataclass
class FieldAggregationSpec:
    """Specification for aggregating a field across lap/segment."""
    field: str                           # e.g., 'car/gyro'
    output_key: str                      # e.g., 'gyro_z_agg'
    index: Optional[int] = None          # Vector index (None for scalars)
    transform: Optional[Callable] = None # Transform function
    aggregation: str = 'avg'             # avg, sum, min, max, median, delta

    def extract(self, record: dict) -> Optional[float]:
        """Extract and transform value from record."""
        try:
            value = record[self.field]
            if self.index is not None:
                value = value[self.index]
            if self.transform:
                value = self.transform(value)
            return float(value)
        except (KeyError, IndexError, TypeError):
            return None


class FieldAccumulator:
    """Accumulates field values for aggregation."""

    def __init__(self, aggregation_method: str):
        self.method = aggregation_method
        self.values = []

    def add(self, value: float):
        self.values.append(value)

    def compute(self) -> Optional[float]:
        if not self.values:
            return None

        if self.method == 'avg':
            return sum(self.values) / len(self.values)
        elif self.method == 'sum':
            return sum(self.values)
        elif self.method == 'min':
            return min(self.values)
        elif self.method == 'max':
            return max(self.values)
        elif self.method == 'median':
            sorted_vals = sorted(self.values)
            mid = len(sorted_vals) // 2
            return sorted_vals[mid]
        elif self.method == 'delta':
            # Delta: last value - first value
            return self.values[-1] - self.values[0]
        else:
            raise ValueError(f'Unknown aggregation method: {self.method}')


class TubStatistics(object):
    """
    A statistics calculator for tub data. Sorts and calculates quantiles for
    lap times, distances and gyro data.
    """

    def __init__(self,
                 tub: Tub,
                 config: Optional[Any] = None,
                 sorting_strategy: Optional[SortingStrategy] = None,
                 field_aggregations: Optional[List] = None):
        """
        Construct tub statistics calculator for tub

        :param tub:                 input tub
        :param config:              Config object (loads FIELD_AGGREGATIONS,
                                    LAP_SORTING_CRITERIA). Required if
                                    field_aggregations not provided.
        :param sorting_strategy:    Optional custom sorting strategy
        :param field_aggregations:  Optional list of FieldAggregationSpec or
                                    dicts. Required if config not provided.
        :raises ValueError:         If neither config nor field_aggregations
                                    provided.
        """
        self.tub = tub

        # Load field aggregations - require explicit configuration
        if field_aggregations is not None:
            self.field_aggregations = self._normalize_field_aggregations(
                field_aggregations)
        elif config:
            self.field_aggregations = (
                self._load_field_aggregations_from_config(config))
        else:
            raise ValueError(
                'TubStatistics requires either config or field_aggregations. '
                'No silent defaults are used - configure FIELD_AGGREGATIONS '
                'in config or pass field_aggregations directly.')

        # Load sorting strategy
        if sorting_strategy:
            self.sorting_strategy = sorting_strategy
        elif config:
            self.sorting_strategy = (
                self._load_sorting_strategy_from_config(config))
        else:
            self.sorting_strategy = default_lap_sorting_strategy()

        logger.info(f'Creating TubStatistics with '
                    f'{len(self.field_aggregations)} field aggregations')

    def _normalize_field_aggregations(self, field_aggregations: List) -> List[
                                         FieldAggregationSpec]:
        """Convert dict specs to FieldAggregationSpec.

        :raises ValueError: If old-style extractor syntax is used.
        """
        normalized = []
        for spec in field_aggregations:
            if isinstance(spec, FieldAggregationSpec):
                normalized.append(spec)
            elif isinstance(spec, dict):
                if 'extractor' in spec:
                    raise ValueError(
                        f'Old-style field_aggregations with "extractor" '
                        f'not supported for field {spec.get("field", "?")}. '
                        f'Use "index" parameter instead.')
                normalized.append(FieldAggregationSpec(
                    field=spec['field'],
                    output_key=spec['output_key'],
                    index=spec.get('index'),
                    transform=spec.get('transform'),
                    aggregation=spec.get('aggregation', 'avg')
                ))
        return normalized

    def _load_field_aggregations_from_config(self, config) -> List[
        FieldAggregationSpec]:
        """Load field aggregation specs from config.

        :raises ValueError: If FIELD_AGGREGATIONS not found in config.
        """
        config_specs = getattr(config, 'FIELD_AGGREGATIONS', None)

        if not config_specs:
            raise ValueError(
                'FIELD_AGGREGATIONS not found in config. '
                'Please define FIELD_AGGREGATIONS in your config file. '
                'Example: FIELD_AGGREGATIONS = [{"field": "car/gyro", '
                '"output_key": "gyro_z_agg", "index": 1, "aggregation": "avg"}]')

        # Convert config dicts to FieldAggregationSpec
        specs = []
        for spec_dict in config_specs:
            spec = FieldAggregationSpec(
                field=spec_dict['field'],
                output_key=spec_dict['output_key'],
                index=spec_dict.get('index'),
                transform=spec_dict.get('transform'),
                aggregation=spec_dict.get('aggregation', 'avg')
            )
            specs.append(spec)
            logger.info(f'Loaded field aggregation: {spec.output_key} from '
                       f'{spec.field}[{spec.index}] using {spec.aggregation}')

        return specs

    def _load_sorting_strategy_from_config(self, config) -> SortingStrategy:
        """Load sorting strategy from config."""
        criteria = getattr(config, 'LAP_SORTING_CRITERIA', None)
        if criteria:
            logger.info(f'Loaded sorting criteria from config: '
                       f'{[c["key"] for c in criteria]}')
            return SortingStrategy(criteria)
        else:
            logger.info('No LAP_SORTING_CRITERIA in config, using minimal '
                       'defaults (time, distance). Configure LAP_SORTING_CRITERIA '
                       'in config to include custom fields like gyro_z_agg.')
            return default_lap_sorting_strategy()

    def generate_laptimes_from_records(self, overwrite=False):

        def new_session(session_id, lap_times, this_session_id, this_lap,
                               record):
            if session_id is not None:
                # copy results of current session
                res[session_id] = copy(lap_times)
                # reset lap times
                lap_times.clear()

            session_id = this_session_id
            lap = this_lap
            time_stamp_ms = record['_timestamp_ms']
            dist = record['car/distance']
            return session_id, lap, time_stamp_ms, dist, lap_times

        session_id = None
        lap = 0
        dist = 0
        time_stamp_ms = None
        lap_times = []
        res = {}

        for record in self.tub:
            this_session_id = record.get('_session_id')
            this_lap = record.get('car/lap', 0)

            if this_session_id != session_id:
                session_id, lap, time_stamp_ms, dist, lap_times = new_session(
                    session_id, lap_times, this_session_id, this_lap, record)
                continue

            if this_lap == lap:
                continue

            assert this_lap > lap, (
                f'Found smaller lap {this_lap} than previous lap {lap} '
                f'in session {session_id}')

            this_time_stamp_ms = record['_timestamp_ms']
            lap_time = (this_time_stamp_ms - time_stamp_ms) / 1000
            this_dist = record['car/distance']
            lap_dist = this_dist - dist
            lap_times.append(dict(lap=lap, time=lap_time, distance=lap_dist))

            lap = this_lap
            time_stamp_ms = this_time_stamp_ms
            dist = this_dist

        assert session_id is not None, "Session id should not be None"
        res[session_id] = lap_times

        for sess_id, lap_times in res.items():
            meta_session_id_dict = self.tub.manifest.metadata.setdefault(
                sess_id, {})
            if overwrite or 'laptimer' not in meta_session_id_dict:
                meta_session_id_dict['laptimer'] = lap_times

        self.tub.manifest.write_metadata()
        logger.info(f'Generated lap times {res}')

    def calculate_lap_performance(self, use_lap_0=False, num_bins=None,
                                  compress=False):
        """
        Creates a dictionary of dictionaries of dictionaries with quantiles
        of sorting criteria to call like d['session_id'][lap_i]['time'] =
        0.2. Depending on the numbers of buckets, say for example 5, 0.2
        would be returned for the fastest 20% of laps and 1.0 would be the
        returned for the slowest 20%. We can also get info on 'distance' and
        'gyro_z_agg' which stands for aggregated gyro_z values of the whole lap.

        Uses the configured sorting_strategy to rank laps, making this method
        modular and extensible.

        :param use_lap_0:   If the 0'th lap should be ignored. On the
                            real car lap zero shows up when the line is
                            crossed the first time hence the lap is
                            incomplete, but in the sim 0 indicates the
                            first complete lap
        :param num_bins: If given, buckets the laps into as many buckets
                            and assigns the numbers i/num_buckets, i=1,...,
                            num_buckets to each lap in that bucket.
        :param compress:    If True, return a dictionary with a single entry
                            where all sessions are compressed into one

        :return:            dict of type
                            {sess_id: {lap_i: {'time': ti,...,'distance': di }}}
        """
        self._calculate_aggregated_fields()
        logger.info(f'Calculating lap performance in tub {self.tub.base_path}')
        sessions \
            = self.tub.manifest.manifest_metadata['sessions']['all_full_ids']
        session_lap_data = list()
        session_lap_metadata = list()  # Track which session each lap belongs to

        for session_id in sessions:
            session_dict = self.tub.manifest.metadata.get(session_id)
            assert session_dict, f"Missing metadata for session_id {session_id}"
            lap_timer = session_dict.get('laptimer')
            if not lap_timer:
                logger.warning(f"Missing or empty laptimer in session_id"
                               f" {session_id} metadata, skipping this id")
                continue
            # Remove lap zero if it shouldn't be considered. It should be first
            # entry, but check before removal.
            if not use_lap_0 and lap_timer[0]['lap'] == 0:
                del(lap_timer[0])
            # Remove laps that are not valid and add in session id
            laps_filtered = [l | {'session_id': session_id} for l in lap_timer
                             if l.get('valid', True)]
            # Track laps and their metadata
            session_lap_data.append(laps_filtered)
            session_lap_metadata.append(session_id)

        # Now we could compress all data per session_id into a single rank
        if compress:
            all_laps = [e for ld in session_lap_data for e in ld]
            session_lap_data = [all_laps]
            if session_lap_metadata:
                session_lap_metadata = [session_lap_metadata[0]]
            else:
                session_lap_metadata = ['compressed']

        # Use SortingStrategy to rank laps (replaces nested rank_laps function)
        session_lap_rank = defaultdict(lambda: defaultdict(dict))
        for laps_data in session_lap_data:
            if not laps_data:
                continue

            # Rank laps using the sorting strategy
            rankings = self.sorting_strategy.rank_laps(laps_data, num_bins)

            # Log detailed ranking information
            self._log_rankings(laps_data, rankings)

            # Convert rankings back to session_lap_rank format
            for lap_idx, lap_rankings in rankings.items():
                lap_data = laps_data[lap_idx]
                session_id = lap_data['session_id']
                lap_num = lap_data['lap']
                session_lap_rank[session_id][lap_num] = lap_rankings

        return session_lap_rank

    def all_lap_times(self):
        """ returns {session_id_1: { lap_i: time_i, ...}, session_id_2:... } """
        d = {s_id: {lap_timer_i['lap']: lap_timer_i['time'] for
                    lap_timer_i in v['laptimer']} for s_id, v in
             self.tub.manifest.metadata.items()}
        return d

    def compute_segment_assignments(
        self,
        lap_detector='ycrossing',
        segmentation_strategy='hybrid',
        min_segment_length=1.0,
        curvature_threshold=0.1
    ):
        """
        Compute segment assignments for all sessions, store in metadata.

        For each session:
        - Loads PathData from tub records
        - Detects laps
        - Builds mean course from ALL laps
        - Segments the course
        - Stores segmentation data in manifest metadata

        :param lap_detector: 'ycrossing' or 'drift'
        :param segmentation_strategy: 'threshold', 'extrema', 'gradient',
                                       'hybrid'
        :param min_segment_length: Minimum segment length in meters
        :param curvature_threshold: Curvature threshold for segmentation
        """
        logger.info(f'Computing segment assignments for tub '
                    f'{self.tub.base_path}')

        sessions = self.tub.manifest.manifest_metadata['sessions'][
            'all_full_ids']

        for session_id in sessions:
            logger.info(f'Processing session {session_id}')

            # Load PathData from tub
            data_source = TubPathDataSource(self.tub.base_path)
            path_data = data_source.load()

            # Detect laps
            if lap_detector == 'ycrossing':
                detector = YCrossingLapDetector()
            elif lap_detector == 'drift':
                detector = DriftLapDetector()
            else:
                raise ValueError(f'Unknown lap detector: {lap_detector}')

            lap_boundaries = detector.detect_laps(path_data)
            multilap_data = MultiLapData(path_data, lap_boundaries)

            if multilap_data.num_laps == 0:
                logger.warning(f'No laps detected in session {session_id}')
                continue

            # Build mean course from ALL laps
            builder = MeanCourseBuilder()
            mean_course = builder.build(multilap_data)

            # Create segmenter with appropriate strategy
            strategy_map = {
                'threshold': ThresholdSegmentation(),
                'extrema': ExtremaSegmentation(),
                'gradient': GradientSegmentation(),
                'hybrid': HybridSegmentation()
            }
            strategy = strategy_map.get(
                segmentation_strategy, HybridSegmentation())
            segmenter = CourseSegmenter(
                strategy,
                params={
                    'min_segment_length': min_segment_length,
                    'straight_curvature_threshold': curvature_threshold
                }
            )
            segmentation = segmenter.segment(mean_course)

            # Store segmentation data in manifest metadata (NOT in records)
            # This enables on-the-fly segment ID computation at training time
            session_dict = self.tub.manifest.metadata.setdefault(
                session_id, {})

            session_dict['segmentation'] = {
                'num_segments': len(segmentation.segments),
                # Mean course data for reconstruction
                'mean_course': {
                    'x': mean_course.x.tolist(),
                    'y': mean_course.y.tolist(),
                    'heading': mean_course.heading.tolist(),
                    'distance': mean_course.distance.tolist(),
                },
                # Segment data for initial segment detection
                'segments': [
                    {
                        'segment_id': int(seg.segment_id),
                        'start_index': int(seg.start_index),
                        'end_index': int(seg.end_index),
                    }
                    for seg in segmentation.segments
                ],
                # Boundary data for crossing detection
                'segment_boundaries': [
                    {
                        'point': b['point'].tolist(),
                        'tangent': b['tangent'].tolist(),
                        'tangent_limit': float(b['tangent_limit']),
                        'expected_denom_sign': int(b['expected_denom_sign']),
                        'segment_from': int(b['segment_from']),
                        'segment_to': int(b['segment_to']),
                    }
                    for b in segmentation.segment_boundaries
                ],
                # Parameters for reference
                'mean_course_params': {
                    'num_laps': len(multilap_data.laps)
                },
                'segmentation_params': {
                    'strategy': segmentation_strategy,
                    'min_segment_length': min_segment_length,
                    'curvature_threshold': curvature_threshold
                }
            }

            logger.info(f'Assigned {len(segmentation.segment_boundaries)} '
                       f'segments to session {session_id}')

        # Write metadata to manifest
        self.tub.manifest.write_metadata()
        logger.info('Segment assignment complete')

    def calculate_segment_performance(self, use_lap_0=False, num_bins=None,
                                     lap_resolver=None, segment_resolver=None,
                                     session_id=None):
        """
        Calculate performance rankings for each segment instance across laps.

        Uses same field aggregation specs as lap performance for consistency.
        Computes segment IDs on-the-fly from stored segmentation metadata.

        :param use_lap_0: If the 0'th lap should be ignored
        :param num_bins: If given, buckets the segments into as many buckets
        :param lap_resolver: Optional callable (record_idx -> lap)
        :param segment_resolver: Optional callable (record_idx -> segment)
        :param session_id: Optional session filter for lap_resolver

        :return: dict of type
                 {sess_id: {lap_i: {seg_i: rankings_dict}}}
        """
        self._calculate_aggregated_fields()
        logger.info(f'Calculating segment performance in tub '
                    f'{self.tub.base_path}')

        segment_instances = defaultdict(lambda: defaultdict(list))
        tracker = SegmentTracker(self.field_aggregations, use_lap_0)

        # Lazy-load assigners per session for on-the-fly segment computation
        assigners = {}
        prev_segments = {}

        record_idx = 0
        for record in self.tub:
            lap = self._resolve_lap(record, record_idx, lap_resolver,
                                   session_id)
            if lap is None:
                record_idx += 1
                continue
            if tracker.should_skip(record, lap):
                record_idx += 1
                continue

            session_id = record.get('_session_id')
            timestamp_ms = record.get('_timestamp_ms', 0)
            distance = record.get('car/distance', 0.0)

            # Try to get segment ID from record first (backward compatibility)
            segment = self._resolve_segment(
                record, record_idx, segment_resolver, session_id)

            # If not in record, compute on-the-fly from metadata
            if segment is None:
                segment = self._compute_segment_from_metadata(
                    record, session_id, assigners, prev_segments)
                if segment is None:
                    record_idx += 1
                    continue  # No segmentation data for this session

            # Handle state changes (early continue on change)
            if tracker.handle_session_change(
                session_id, lap, segment, timestamp_ms, distance,
                segment_instances, self._finalize_segment_instance):
                record_idx += 1
                continue

            if tracker.handle_lap_change(
                lap, segment, timestamp_ms, distance, segment_instances,
                self._finalize_segment_instance):
                record_idx += 1
                continue

            if tracker.handle_segment_change(
                segment, timestamp_ms, distance, segment_instances,
                self._finalize_segment_instance):
                record_idx += 1
                continue

            # Accumulate field values
            tracker.accumulate_fields(record)
            record_idx += 1

        # Finalize last segment
        self._finalize_last_segment(segment_instances, tracker)

        return self._rank_segment_instances(segment_instances, num_bins)

    def _resolve_lap(self, record, record_idx, lap_resolver, session_id):
        """Resolve lap for a record, using optional resolver."""
        if session_id and record.get('_session_id') != session_id:
            return None
        if lap_resolver:
            return lap_resolver(record_idx)
        return record.get('car/lap')

    def _resolve_segment(self, record, record_idx, segment_resolver,
                        session_id):
        """Resolve segment for a record, using optional resolver."""
        if session_id and record.get('_session_id') != session_id:
            return None
        if segment_resolver:
            segment = segment_resolver(record_idx)
            if segment is not None:
                return segment
        return record.get('car/segment')

    def _finalize_last_segment(self, segment_instances, tracker):
        """Finalize the last segment using last record data."""
        last_timestamp = None
        last_distance = None
        for record in self.tub:
            last_timestamp = record.get('_timestamp_ms', 0)
            last_distance = record.get('car/distance', 0.0)

        if tracker.current_session and last_timestamp is not None:
            self._finalize_segment_instance(
                segment_instances, tracker.current_session,
                tracker.current_lap, tracker.current_segment,
                tracker.segment_start_time, tracker.segment_start_dist,
                last_timestamp, last_distance, tracker.field_accumulators
            )

    def _finalize_segment_instance(self, segment_instances, session_id, lap,
                                   segment_id, start_time, start_dist,
                                   end_time, end_dist, field_accumulators):
        """Finalize a segment instance and add to collection."""
        if (session_id is None or segment_id is None or
            start_time is None or start_dist is None):
            return

        # Compute time and distance
        seg_time = (end_time - start_time) / 1000.0
        seg_dist = end_dist - start_dist

        # Build instance dict with all field values
        instance = {
            'lap': lap,
            'time': seg_time,
            'distance': seg_dist
        }

        # Add aggregated field values
        for output_key, accumulator in field_accumulators.items():
            value = accumulator.compute()
            if value is not None:
                instance[output_key] = value

        segment_instances[session_id][segment_id].append(instance)

    def _rank_segment_instances(self, segment_instances, num_bins):
        """Rank segment instances and return session->lap->segment structure."""
        session_segment_rank = defaultdict(lambda: defaultdict(dict))

        for session_id, segments in segment_instances.items():
            self._rank_session_segments(session_id, segments, num_bins,
                                       session_segment_rank)

        return session_segment_rank

    def _log_rankings(self, data_items, rankings, segment_id=None):
        """
        Log detailed ranking information for laps or segments.

        :param data_items: List of dictionaries containing lap/segment data
        :param rankings: Dictionary mapping indices to ranking quantiles
        :param segment_id: Optional segment ID for segment logging (None for
                           lap logging)
        """
        if not self.sorting_strategy.criteria:
            return

        primary_criterion = self.sorting_strategy.criteria[0]
        field_key = primary_criterion['key']
        transform = primary_criterion['transform']
        reverse = primary_criterion['reverse']

        # Extract and transform values using list comprehension
        item_values = [
            (idx, value, transform(value))
            for idx, item_data in enumerate(data_items)
            if (value := item_data.get(field_key)) is not None
        ]

        if not item_values:
            return

        item_values.sort(key=lambda x: x[2], reverse=reverse)

        total_items = len(item_values)
        for rank, (idx, raw_value, _) in enumerate(item_values, start=1):
            lap_num = data_items[idx]['lap']
            quantile = rankings[idx].get(field_key, 0.0)

            segment_label = (f', Segment {segment_id}'
                           if segment_id is not None else '')
            logger.info(
                f'Lap {lap_num}{segment_label}: '
                f'{field_key}={raw_value:.4f}, '
                f'rank={rank}/{total_items}, '
                f'quantile={quantile*100:.0f}%'
            )

    def _rank_session_segments(self, session_id, segments, num_bins,
                              session_segment_rank):
        """Rank all segments for a single session."""
        for segment_id, instances in segments.items():
            if not instances:
                continue

            rankings = self.sorting_strategy.rank_laps(instances, num_bins)

            # Log detailed ranking information
            self._log_rankings(instances, rankings, segment_id)

            self._store_segment_rankings(session_id, segment_id, instances,
                                        rankings, session_segment_rank)

    def _store_segment_rankings(self, session_id, segment_id, instances,
                                rankings, session_segment_rank):
        """Store segment rankings in the result structure."""
        for inst_idx, inst_rankings in rankings.items():
            lap_num = instances[inst_idx]['lap']
            session_segment_rank[session_id][lap_num][segment_id] = (
                inst_rankings)

    def _calculate_aggregated_fields(self):
        """
        Calculate aggregated values for configured fields per lap.

        Generic implementation that handles any field with custom
        extractor and transform functions.
        """
        logger.info(f'Calculating {len(self.field_aggregations)} field '
                    f'aggregations in tub {self.tub.base_path}')

        for field_spec in self.field_aggregations:
            self._aggregate_single_field(field_spec)

    def _aggregate_single_field(self, spec: FieldAggregationSpec):
        """Aggregate a single field across all laps in all sessions."""
        current_session = None
        current_lap = None
        accumulator = FieldAccumulator(spec.aggregation)
        lap_field_map = {}

        for record in self.tub:
            session_id = record.get('_session_id')
            lap = record.get('car/lap', 0)

            # Handle session change - finalize previous if exists
            if session_id != current_session:
                self._maybe_finalize_session(lap_field_map, current_lap,
                                            accumulator, current_session,
                                            spec.output_key)
                current_session = session_id
                current_lap = lap
                lap_field_map = {}
                accumulator = FieldAccumulator(spec.aggregation)

            # Handle lap change
            if lap != current_lap:
                lap_field_map[current_lap] = accumulator.compute()
                current_lap = lap
                accumulator = FieldAccumulator(spec.aggregation)

            # Accumulate value
            value = spec.extract(record)
            if value is None:
                continue
            accumulator.add(value)

        # Finalize last lap
        self._maybe_finalize_session(lap_field_map, current_lap,
                                    accumulator, current_session,
                                    spec.output_key)

    def _maybe_finalize_session(self, lap_field_map, current_lap,
                               accumulator, current_session, output_key):
        """Helper to finalize session only if it exists."""
        if current_session is None:
            return
        self._finalize_lap_field(lap_field_map, current_lap,
                                accumulator, current_session,
                                output_key)

    def _finalize_lap_field(self, lap_field_map, current_lap,
                           accumulator, current_session, output_key):
        """Finalize lap field data and update metadata."""
        lap_field_map[current_lap] = accumulator.compute()
        self._update_field_metadata(lap_field_map, current_session,
                                   output_key)

    def _update_field_metadata(self, lap_field_map: dict, session: str,
                               output_key: str):
        """Update session metadata with aggregated field values."""
        session_dict = self.tub.manifest.metadata.get(session)
        if not session_dict:
            return

        lap_timer = session_dict.get('laptimer')
        if not lap_timer:
            return

        for entry in lap_timer:
            lap_i = entry['lap']
            agg_value = lap_field_map.get(lap_i)

            # Early continue eliminates else branch
            if agg_value is None:
                entry['valid'] = False
                continue

            entry[output_key] = agg_value

    def _compute_segment_from_metadata(self, record, session_id,
                                       assigners, prev_segments):
        """Compute segment ID from metadata for records without car/segment."""
        pos = record.get('car/pos')
        return get_or_compute_segment_id(
            session_id, pos, self.tub.manifest.metadata,
            assigners, prev_segments)
