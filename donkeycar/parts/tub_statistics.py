from collections import defaultdict
from operator import itemgetter
import logging
from copy import copy
from typing import Optional, List, Dict, Callable, Any

from donkeycar.parts.tub_v2 import Tub
from donkeycar.pipeline.transformations import (
    SortingStrategy,
    default_lap_sorting_strategy
)

logger = logging.getLogger(__name__)


class TubStatistics(object):
    """
    A statistics calculator for tub data. Sorts and calculates quantiles for
    lap times, distances and gyro data.
    """

    def __init__(self,
                 tub: Tub,
                 gyro_z_index: int = 1,
                 sorting_strategy: Optional[SortingStrategy] = None,
                 field_aggregations: Optional[List[Dict[str, Any]]] = None):
        """
        Construct tub statistics calculator for tub

        :param tub:                 input tub
        :param gyro_z_index:        z coordinate in 3d gyro vector (backward compat)
        :param sorting_strategy:    Optional custom sorting strategy for lap ranking.
                                    If None, uses default (time, distance, gyro_z_agg)
        :param field_aggregations:  Optional list of field aggregation specs:
                                    [{'field': 'car/gyro',
                                      'output_key': 'gyro_z_agg',
                                      'extractor': lambda r: r['car/gyro'][1],
                                      'transform': abs}]
                                    If None, uses default gyro aggregation
        """
        self.tub = tub
        self.sorting_strategy = sorting_strategy or default_lap_sorting_strategy()

        # Default field aggregation (backward compatible)
        if field_aggregations is None:
            self.field_aggregations = [{
                'field': 'car/gyro',
                'output_key': 'gyro_z_agg',
                'extractor': lambda record: record['car/gyro'][gyro_z_index],
                'transform': abs
            }]
        else:
            self.field_aggregations = field_aggregations

        logger.info(f'Creating TubStatistics with '
                    f'{len(self.field_aggregations)} field aggregations')

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
            this_lap = record['car/lap']

            if this_session_id != session_id:
                session_id, lap, time_stamp_ms, dist, lap_times = new_session(
                    session_id, lap_times, this_session_id, this_lap, record)
                continue

            if this_lap == lap:
                continue

            assert this_lap > lap, (f'Found smaller lap {this_lap} than previous'
                                    f' lap {lap} in session {session_id}')

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
            meta_session_id_dict = self.tub.manifest.metadata.get(sess_id)
            if not meta_session_id_dict:
                self.tub.manifest.metadata[sess_id] = dict(laptimer=lap_times)
            elif ('laptimer' in meta_session_id_dict and overwrite
                  or 'laptimer' not in meta_session_id_dict):
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
            session_lap_metadata = [session_lap_metadata[0] if session_lap_metadata else 'compressed']

        # Use SortingStrategy to rank laps (replaces nested rank_laps function)
        session_lap_rank = defaultdict(lambda: defaultdict(dict))
        for laps_data in session_lap_data:
            if not laps_data:
                continue

            # Rank laps using the sorting strategy
            rankings = self.sorting_strategy.rank_laps(laps_data, num_bins)

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

    def _aggregate_single_field(self, field_spec: dict):
        """Aggregate a single field across all records."""
        output_key = field_spec['output_key']
        extractor = field_spec['extractor']
        transform = field_spec.get('transform', lambda x: x)

        aggregator = FieldAggregator()

        for record in self.tub:
            lap = record['car/lap']
            session_id = record['_session_id']

            # Extract and transform value, skip on error
            value = self._extract_and_transform(record, extractor, transform, output_key)
            if value is None:
                continue

            # Update aggregation state
            aggregator.process_record(session_id, lap, value,
                                     lambda data, sess: self._update_field_metadata(data, sess, output_key))

        # Finalize last session
        aggregator.finalize(lambda data, sess: self._update_field_metadata(data, sess, output_key))

    def _extract_and_transform(self, record: dict, extractor: Callable,
                               transform: Callable, output_key: str) -> Optional[float]:
        """Extract value from record and apply transformation."""
        try:
            raw_val = extractor(record)
            return transform(raw_val)
        except (KeyError, IndexError, TypeError) as e:
            logger.warning(f'Failed to extract {output_key}: {e}')
            return None

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
            if agg_value is None:
                entry['valid'] = False
            else:
                entry[output_key] = agg_value


class FieldAggregator:
    """
    Handles state for aggregating field values across sessions and laps.

    Separates state management from the main logic to reduce nesting.
    """

    def __init__(self):
        self.current_session = None
        self.current_lap = None
        self.lap_sum = 0.0
        self.lap_count = 0
        self.lap_field_map = {}

    def process_record(self, session_id: str, lap: int, value: float,
                      update_callback: Callable):
        """Process a single record's value."""
        # Session change - finalize previous session
        if session_id != self.current_session:
            self._finalize_session(update_callback)
            self._start_new_session(session_id)
            self.current_lap = lap
            return

        # Lap change - finalize previous lap
        if lap != self.current_lap:
            self._finalize_lap()
            self.current_lap = lap

        # Accumulate value for current lap
        self.lap_sum += value
        self.lap_count += 1

    def _finalize_lap(self):
        """Save accumulated data for current lap."""
        if self.lap_count == 0 or self.current_lap is None:
            return

        avg_value = self.lap_sum / self.lap_count
        self.lap_field_map[self.current_lap] = avg_value
        self.lap_sum = 0.0
        self.lap_count = 0

    def _finalize_session(self, update_callback: Callable):
        """Save accumulated data for current session."""
        if self.current_session is None:
            return

        self._finalize_lap()
        update_callback(self.lap_field_map, self.current_session)
        self.lap_field_map = {}

    def _start_new_session(self, session_id: str):
        """Initialize state for new session."""
        self.current_session = session_id
        self.lap_sum = 0.0
        self.lap_count = 0

    def finalize(self, update_callback: Callable):
        """Finalize any remaining data."""
        self._finalize_session(update_callback)
