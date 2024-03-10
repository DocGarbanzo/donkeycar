from collections import defaultdict
from operator import itemgetter
import logging
from copy import copy

from donkeycar.parts.tub_v2 import Tub

logger = logging.getLogger(__name__)


class TubStatistics(object):
    """
    A statistics calculator for tub data. Sorts and calculates quantiles for
    lap times, distances and gyro data.
    """

    def __init__(self, tub: Tub, gyro_z_index: int = 1):
        """
        Construct tub statistics calculator for tub

        :param tub:             input tub
        :param gyro_z_index:    z coordinate in 3d gyro vector (this is 1 in
                                sim and 2 in car)
        """
        self.tub = tub
        self.gyro_z_index = gyro_z_index
        logger.info(f'Creating TubStatistics with gyro_z index {gyro_z_index}'
                    f' assuming use {"gym" if gyro_z_index==1 else "real"}')

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
        def rank_laps(laps_filtered, num_buckets, session_lap_rank):
            num_laps = len(laps_filtered)
            num_buckets = num_buckets or num_laps
            for sort_by in ('time', 'distance', 'gyro_z_agg'):
                laps_sorted = sorted(laps_filtered, key=itemgetter(sort_by))
                session_ids = set()
                for i, lap_i in enumerate(laps_sorted):
                    rel_i = int(i * num_buckets / num_laps + 1) / num_buckets
                    session_id = lap_i['session_id']
                    session_lap_rank[session_id][lap_i['lap']][sort_by] = rel_i
                    session_ids.add(session_id)
                log_text = f'Session {session_ids} with {num_laps} valid laps'
                if num_laps > 0:
                    log_text \
                        += f', min {sort_by}: {laps_sorted[0][sort_by]:5.2f}, '\
                           f'max {sort_by}: {laps_sorted[-1][sort_by]:5.2f}'
                logger.info(log_text)

        self._calculate_aggregated_gyro()
        logger.info(f'Calculating lap performance in tub {self.tub.base_path}')
        sessions \
            = self.tub.manifest.manifest_metadata['sessions']['all_full_ids']
        session_lap_data = list()
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
            # lap_timer is a list of dictionaries, sort by time, distance and
            # gyro_z aggregated respectively and record the quantile bins for
            # each of these values per lap in the session_lap_rank dict.
            session_lap_data.append(laps_filtered)

        # Now we could compress all data per session_id into a single rank
        if compress:
            session_lap_data = [[e for ld in session_lap_data for e in ld]]

        session_lap_rank = defaultdict(lambda: defaultdict(dict))
        for laps_data in session_lap_data:
            rank_laps(laps_data, num_bins, session_lap_rank)

        return session_lap_rank

    def all_lap_times(self):
        """ returns {session_id_1: { lap_i: time_i, ...}, session_id_2:... } """
        d = {s_id: {lap_timer_i['lap']: lap_timer_i['time'] for
                    lap_timer_i in v['laptimer']} for s_id, v in
             self.tub.manifest.metadata.items()}
        return d

    def _calculate_aggregated_gyro(self):
        """
        Updates lap_timer in tub metadate with values of aggregated gyro_z
        values. Note, in the sim, thy gyro z is in the middle coordinate,
        not the last.
        """

        def update_metadata(lap_gyro_map, session):
            session_dict = self.tub.manifest.metadata.get(session)
            assert session_dict, \
                f"Missing metadata for session_id {session}"
            lap_timer = session_dict.get('laptimer')
            # Update lap_timer with aggregated values and mark each lap that
            # is not in the map as invalid, so it will not be used in the
            # statistics.
            for entry in lap_timer:
                lap_i = entry['lap']
                agg_value = lap_gyro_map.get(lap_i)
                if agg_value is None:
                    entry['valid'] = False
                else:
                    entry['gyro_z_agg'] = agg_value

        logger.info(f'Calculating aggregated gyro in tub {self.tub.base_path}')
        prev_session = None
        prev_lap = None
        gyro_z_agg = 0
        lap_gyro_map = {}
        count = 0   # counts the number of records in each lap
        for record in self.tub:
            lap = record['car/lap']
            session_id = record['_session_id']

            if session_id != prev_session:
                # If new session found update the map with the value of the
                # last lap of the prev session and update the lap timer of
                # the previous session
                if prev_session is not None:
                    # we should never overwrite anything in the map,
                    # otherwise there is a data problem.
                    assert prev_lap not in lap_gyro_map, \
                        f'Lap {prev_lap} should not be in the map'
                    lap_gyro_map[prev_lap] = gyro_z_agg / count
                    update_metadata(lap_gyro_map, prev_session)

                # update current session to new session
                prev_session = session_id
                # reset lap / gyro map
                lap_gyro_map.clear()
                prev_lap = None
                count = 0

            if lap != prev_lap:
                # only update map if we haven't started a fresh session
                if prev_lap is not None:
                    # we should never overwrite anything in the map,
                    # otherwise there is a data problem.
                    assert prev_lap not in lap_gyro_map, \
                        f'Lap {prev_lap} should not be in the map'
                    # add aggregated normalised gyro value to map
                    lap_gyro_map[prev_lap] = gyro_z_agg / count
                # zero the aggregation value and update lap
                gyro_z_agg = 0
                count = 0
                prev_lap = lap

            val = abs(record['car/gyro'][self.gyro_z_index])
            gyro_z_agg += val
            count += 1

        # update for last lap, because we didn't go through the if lap !=
        # prev_lap part any more and hence need to update the map with the
        # last lap info

        # we should never overwrite anything in the map,
        # otherwise there is a data problem.
        assert lap not in lap_gyro_map, \
            f'Lap {lap} should not be in the map'

        lap_gyro_map[lap] = gyro_z_agg / count
        update_metadata(lap_gyro_map, prev_session)

    def _calculate_laptimer_index(self):
        """
        Updates lap_timer in tub metadata with start and end index of
        records in that lap
        """
        logger.info(f'Calculating laptimer index in tub {self.tub.base_path}')
        prev_lap = None
        prev_session = None
        for record in self.tub:
            idx = record['_index']
            lap = record['car/lap']
            session_id = record['_session_id']
            if lap != prev_lap:
                session_dict = self.tub.manifest.metadata.get(session_id)
                assert session_dict, \
                    f"Missing metadata for session_id {session_id}"
                lap_timer = session_dict['laptimer']
                lap_timer_i = lap_timer[lap]
                assert lap_timer_i['lap'] == lap, \
                    f"Inconsistent laptimer in session {session_id}"
                lap_timer_i['start_index'] = idx
                if prev_session is not None:
                    if prev_session != session_id:
                        prev_dict = self.tub.manifest.metadata[prev_session]
                        plt = prev_dict['laptimer']
                    else:
                        plt = lap_timer
                    if prev_lap is not None:
                        plt[prev_lap]['end_index'] = idx - 1
                prev_lap = lap
