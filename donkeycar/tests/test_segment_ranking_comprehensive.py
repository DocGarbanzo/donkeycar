"""
Comprehensive tests for the segment-based ranking system.

Tests the combinatorial space of aggregation methods, multi-field
ranking interactions, driving behavior discrimination, and transform
functions. Complements existing test files by testing what they do
NOT cover (see PRD gap analysis in docs/prd_segment_ranking_tests.md).

All test data created through official Tub.write_record() or
TubWriter.run() interfaces. No internal state manipulation.
"""
import os
import shutil
import tempfile
import unittest

from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import (
    TubStatistics, FieldAggregationSpec,
)
from donkeycar.pipeline.transformations import SortingStrategy

from donkeycar.tests.tub_test_data_generator import (
    DrivingProfile,
    STRAIGHT_FAST, STRAIGHT_SLOW,
    TURN_SMOOTH, TURN_AGGRESSIVE, TURN_JERKY,
    BRAKING_HARD, BRAKING_GENTLE, CHICANE,
    MINIMAL_INPUTS, MINIMAL_TYPES,
    SENSOR_RANGES,
    create_multilap_tub,
    create_multilap_tub_via_writer,
    create_tub_with_varied_segment_times,
    validate_generated_tub,
)


class SegmentRankingTestBase(unittest.TestCase):
    """Base class providing tub lifecycle management."""

    def setUp(self):
        self.test_dirs = []

    def tearDown(self):
        for d in self.test_dirs:
            if os.path.exists(d):
                shutil.rmtree(d)

    def _make_tub_dir(self) -> str:
        d = tempfile.mkdtemp()
        self.test_dirs.append(d)
        return d

    @staticmethod
    def _build_sorting_strategy(field_aggregations):
        criteria = []
        for spec in field_aggregations:
            criteria.append({
                'key': spec.output_key,
                'transform': spec.transform or (lambda x: x),
                'reverse': spec.reverse,
            })
        return SortingStrategy(criteria)

    def _rank_laps(self, tub, field_aggregations, use_lap_0=True):
        """
        Convenience: generate laptimes, aggregate fields, rank.

        Returns {session_id: {lap: {key: ranking}}}
        """
        strategy = self._build_sorting_strategy(field_aggregations)
        stats = TubStatistics(
            tub, field_aggregations=field_aggregations,
            sorting_strategy=strategy)
        stats.generate_laptimes_from_records(overwrite=True)
        return stats.calculate_lap_performance(use_lap_0=use_lap_0)

    def _rank_segments(self, tub, field_aggregations, use_lap_0=True):
        """
        Convenience: generate laptimes, aggregate fields,
        rank segments.

        Returns {session_id: {lap: {segment: {key: ranking}}}}
        """
        strategy = self._build_sorting_strategy(field_aggregations)
        stats = TubStatistics(
            tub, field_aggregations=field_aggregations,
            sorting_strategy=strategy)
        stats.generate_laptimes_from_records(overwrite=True)
        return stats.calculate_segment_performance(
            use_lap_0=use_lap_0)

    def _get_session_id(self, tub):
        sessions = tub.manifest.manifest_metadata['sessions']
        return sessions['all_full_ids'][0]


# ════════════════════════════════════════════════════════════════════
# TestDataAuthenticity — confidence gate
# ════════════════════════════════════════════════════════════════════


class TestDataAuthenticity(SegmentRankingTestBase):
    """Verify the data generator produces physically plausible data."""

    def test_straight_segment_has_zero_yaw_and_lateral_accel(self):
        path = self._make_tub_dir()
        profiles = [[STRAIGHT_FAST], [STRAIGHT_SLOW]]
        tub = create_multilap_tub(path, profiles)
        for record in tub:
            if record.get('car/lap', 0) >= 2:
                continue  # skip final record
            gyro_z = record['car/gyro'][2]
            accel_y = record['car/accel'][1]
            self.assertAlmostEqual(
                gyro_z, 0.0, delta=0.05,
                msg=f"Straight segment has gyro_z={gyro_z}")
            self.assertAlmostEqual(
                accel_y, 0.0, delta=0.05,
                msg=f"Straight segment has accel_y={accel_y}")
        tub.close()

    def test_turn_has_correlated_gyro_and_lateral_accel(self):
        path = self._make_tub_dir()
        profiles = [[TURN_SMOOTH], [TURN_AGGRESSIVE]]
        tub = create_multilap_tub(path, profiles)
        for record in tub:
            if record.get('car/lap', 0) >= 2:
                continue
            gyro_z = abs(record['car/gyro'][2])
            accel_y = abs(record['car/accel'][1])
            if gyro_z > 0.05:
                self.assertGreater(
                    accel_y, 0.01,
                    f"Turning (gyro_z={gyro_z}) without "
                    f"lateral accel (accel_y={accel_y})")
        tub.close()

    def test_braking_produces_negative_longitudinal_accel(self):
        path = self._make_tub_dir()
        profiles = [[BRAKING_HARD], [BRAKING_GENTLE]]
        tub = create_multilap_tub(path, profiles, records_per_segment=20)
        records_per_lap = 20
        rec_count = 0
        for record in tub:
            lap = record.get('car/lap', 0)
            if lap >= 2:
                continue
            rec_in_lap = rec_count % records_per_lap
            progress = rec_in_lap / records_per_lap
            if progress < 0.5:
                accel_x = record['car/accel'][0]
                self.assertLess(
                    accel_x, 0.05,
                    f"Braking phase has positive accel_x={accel_x}")
            rec_count += 1
        tub.close()

    def test_speed_within_rc_car_range(self):
        path = self._make_tub_dir()
        all_profiles = [
            STRAIGHT_FAST, STRAIGHT_SLOW, TURN_SMOOTH,
            TURN_AGGRESSIVE, TURN_JERKY, BRAKING_HARD,
            BRAKING_GENTLE, CHICANE,
        ]
        profiles = [[p] for p in all_profiles]
        tub = create_multilap_tub(path, profiles)
        for record in tub:
            speed = record['car/speed']
            self.assertGreaterEqual(speed, 0.0)
            self.assertLessEqual(speed, SENSOR_RANGES['speed_max'])
        tub.close()

    def test_all_sensors_within_normalized_range(self):
        path = self._make_tub_dir()
        all_profiles = [
            STRAIGHT_FAST, STRAIGHT_SLOW, TURN_SMOOTH,
            TURN_AGGRESSIVE, TURN_JERKY, BRAKING_HARD,
            BRAKING_GENTLE, CHICANE,
        ]
        profiles = [[p] for p in all_profiles]
        tub = create_multilap_tub(path, profiles)
        for record in tub:
            for i in range(3):
                gyro = record['car/gyro'][i]
                self.assertGreaterEqual(gyro, -1.0,
                                        f"gyro[{i}]={gyro}")
                self.assertLessEqual(gyro, 1.0,
                                     f"gyro[{i}]={gyro}")
            for i in range(3):
                accel = record['car/accel'][i]
                self.assertGreaterEqual(accel, -1.0,
                                        f"accel[{i}]={accel}")
                self.assertLessEqual(accel, 1.0,
                                     f"accel[{i}]={accel}")
        tub.close()

    def test_speed_continuity_between_records(self):
        path = self._make_tub_dir()
        records_per_segment = 20
        profiles = [
            [BRAKING_HARD, STRAIGHT_FAST],
            [TURN_SMOOTH, TURN_AGGRESSIVE],
        ]
        tub = create_multilap_tub(
            path, profiles, records_per_segment=records_per_segment)
        prev_speed = None
        prev_ts = None
        prev_lap = None
        prev_segment = None
        rec_count = 0
        for record in tub:
            lap = record.get('car/lap', 0)
            ts = record['_timestamp_ms']
            speed = record['car/speed']
            # Determine which segment this record belongs to
            segment = (rec_count % (records_per_segment * 2)
                       ) // records_per_segment
            # Only check continuity within the same segment
            same_segment = (prev_segment is not None
                            and segment == prev_segment)
            if (prev_speed is not None and lap == prev_lap
                    and same_segment):
                dt_s = (ts - prev_ts) / 1000.0
                if dt_s > 0:
                    speed_change = abs(speed - prev_speed)
                    max_change = 10.0 * dt_s + 0.01
                    self.assertLessEqual(
                        speed_change, max_change,
                        f"Speed jump {speed_change:.2f} in {dt_s:.3f}s")
            prev_speed = speed
            prev_ts = ts
            prev_lap = lap
            prev_segment = segment
            rec_count += 1
        tub.close()

    def test_smooth_driving_has_lower_noise_than_jerky(self):
        path = self._make_tub_dir()
        profiles = [[TURN_SMOOTH], [TURN_JERKY]]
        tub = create_multilap_tub(path, profiles, records_per_segment=50)
        laps_gyro = {0: [], 1: []}
        for record in tub:
            lap = record.get('car/lap', 0)
            if lap in laps_gyro:
                laps_gyro[lap].append(record['car/gyro'][2])
        import numpy as np
        std_smooth = np.std(laps_gyro[0])
        std_jerky = np.std(laps_gyro[1])
        self.assertLess(
            std_smooth, std_jerky,
            f"Smooth ({std_smooth:.4f}) noisier than "
            f"jerky ({std_jerky:.4f})")
        tub.close()

    def test_validate_generated_tub_runs_implicitly(self):
        path = self._make_tub_dir()
        profiles = [
            [TURN_AGGRESSIVE, BRAKING_HARD],
            [STRAIGHT_FAST, CHICANE],
            [TURN_SMOOTH, BRAKING_GENTLE],
        ]
        tub = create_multilap_tub(path, profiles)
        summary = validate_generated_tub(tub, strict=False)
        self.assertTrue(summary['valid'])
        self.assertEqual(len(summary['violations']), 0)
        self.assertGreaterEqual(summary['speed_range'][0], 0)
        self.assertLessEqual(
            summary['speed_range'][1], SENSOR_RANGES['speed_max'])
        tub.close()


# ════════════════════════════════════════════════════════════════════
# TestFieldAggregationMethods
# ════════════════════════════════════════════════════════════════════


class TestFieldAggregationMethods(SegmentRankingTestBase):
    """Test each aggregation method through ranking order."""

    def test_avg_aggregation_gyro_smoothness(self):
        path = self._make_tub_dir()
        profiles = [
            [TURN_SMOOTH, STRAIGHT_FAST],
            [TURN_AGGRESSIVE, STRAIGHT_FAST],
            [TURN_JERKY, STRAIGHT_FAST],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # TURN_SMOOTH has lowest avg(abs(gyro_z))
        self.assertLess(perf[s][0]['gyro_z_agg'],
                        perf[s][1]['gyro_z_agg'])
        tub.close()

    def test_sum_aggregation_total_yaw(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.0, curvature=0.1)],
            [DrivingProfile(speed=2.0, curvature=0.5)],
            [DrivingProfile(speed=2.0, curvature=1.0)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='yaw_sum',
                transform=abs, aggregation='sum'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        self.assertLess(perf[s][0]['yaw_sum'], perf[s][1]['yaw_sum'])
        self.assertLess(perf[s][1]['yaw_sum'], perf[s][2]['yaw_sum'])
        tub.close()

    def test_min_aggregation_hardest_braking(self):
        path = self._make_tub_dir()
        profiles = [
            [BRAKING_HARD],
            [BRAKING_GENTLE],
            [TURN_SMOOTH],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/accel', index=0, output_key='brake_min',
                transform=None, aggregation='min'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Ascending sort: most negative min ranks first
        self.assertLess(perf[s][0]['brake_min'],
                        perf[s][1]['brake_min'])
        self.assertLess(perf[s][1]['brake_min'],
                        perf[s][2]['brake_min'])
        tub.close()

    def test_max_aggregation_peak_acceleration(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.0, speed_delta=1.5)],
            [DrivingProfile(speed=2.0, speed_delta=0.8)],
            [DrivingProfile(speed=2.0, speed_delta=0.2)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/accel', index=0, output_key='accel_peak',
                transform=None, aggregation='max'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Ascending: lowest max ranks first
        self.assertLess(perf[s][2]['accel_peak'],
                        perf[s][1]['accel_peak'])
        self.assertLess(perf[s][1]['accel_peak'],
                        perf[s][0]['accel_peak'])
        tub.close()

    def test_median_aggregation_typical_speed(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=1.5, speed_delta=0.0)],
            [DrivingProfile(speed=2.5, speed_delta=0.0)],
            [DrivingProfile(speed=3.5, speed_delta=0.0)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/speed', output_key='speed_med',
                aggregation='median'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        self.assertLess(perf[s][0]['speed_med'],
                        perf[s][1]['speed_med'])
        self.assertLess(perf[s][1]['speed_med'],
                        perf[s][2]['speed_med'])
        tub.close()

    def test_delta_aggregation_speed_change(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.0, speed_delta=1.0)],
            [DrivingProfile(speed=2.5, speed_delta=0.0)],
            [DrivingProfile(speed=3.0, speed_delta=-1.0)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/speed', output_key='speed_delta',
                aggregation='delta'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Delta = last - first. Ascending: most negative first
        self.assertLess(perf[s][2]['speed_delta'],
                        perf[s][1]['speed_delta'])
        self.assertLess(perf[s][1]['speed_delta'],
                        perf[s][0]['speed_delta'])
        tub.close()

    def test_delta_single_record_segment(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.0)],
            [DrivingProfile(speed=3.5)],
        ]
        tub = create_multilap_tub(path, profiles, records_per_segment=1)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/speed', output_key='speed_delta',
                aggregation='delta'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # With 1 record, delta = v - v = 0 for both laps.
        # Ranking assigns different quantiles to ties, so just
        # check both are valid rankings.
        self.assertGreater(perf[s][0]['speed_delta'], 0)
        self.assertLessEqual(perf[s][0]['speed_delta'], 1.0)
        self.assertGreater(perf[s][1]['speed_delta'], 0)
        self.assertLessEqual(perf[s][1]['speed_delta'], 1.0)
        tub.close()

    def test_median_even_count(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.0, speed_delta=0.0)],
            [DrivingProfile(speed=3.5, speed_delta=0.0)],
        ]
        tub = create_multilap_tub(path, profiles, records_per_segment=4)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/speed', output_key='speed_med',
                aggregation='median'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        self.assertLess(perf[s][0]['speed_med'],
                        perf[s][1]['speed_med'])
        tub.close()


# ════════════════════════════════════════════════════════════════════
# TestMultiFieldRankingPriority
# ════════════════════════════════════════════════════════════════════


class TestMultiFieldRankingPriority(SegmentRankingTestBase):
    """Test priority ordering and tiebreaking in multi-field ranking."""

    def test_primary_sort_key_dominates(self):
        path = self._make_tub_dir()
        lap_segment_times = [
            [2000, 2000],
            [1000, 2000],
            [3000, 2000],
        ]
        base = TURN_SMOOTH
        profiles = [[base, base]] * 3
        tub = create_tub_with_varied_segment_times(
            path, 3, 2, lap_segment_times,
            profiles=profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Lap 1 is fastest -> best time ranking
        self.assertLess(perf[s][1]['time'], perf[s][0]['time'])
        self.assertLess(perf[s][0]['time'], perf[s][2]['time'])
        tub.close()

    def test_secondary_breaks_ties(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.5, curvature=0.1)],
            [DrivingProfile(speed=2.5, curvature=0.5)],
            [DrivingProfile(speed=2.5, curvature=0.9)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Gyro ranking differentiates since times are equal
        self.assertLess(perf[s][0]['gyro_z_agg'],
                        perf[s][1]['gyro_z_agg'])
        self.assertLess(perf[s][1]['gyro_z_agg'],
                        perf[s][2]['gyro_z_agg'])
        tub.close()

    def test_four_field_ranking(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=3.5, curvature=0.1,
                            braking_intensity=0.5), STRAIGHT_FAST],
            [DrivingProfile(speed=3.0, curvature=0.3,
                            braking_intensity=1.5), STRAIGHT_FAST],
            [DrivingProfile(speed=2.5, curvature=0.5,
                            braking_intensity=2.5), STRAIGHT_FAST],
            [DrivingProfile(speed=2.0, curvature=0.7,
                            braking_intensity=3.5), STRAIGHT_FAST],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
            FieldAggregationSpec(
                field='car/accel', index=0, output_key='brake_min',
                aggregation='min'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        for lap_idx in range(4):
            lap_rank = perf[s][lap_idx]
            self.assertIn('time', lap_rank)
            self.assertIn('distance', lap_rank)
            self.assertIn('gyro_z_agg', lap_rank)
            self.assertIn('brake_min', lap_rank)
        # Gyro: lap 0 best (0.1), lap 3 worst (0.7)
        self.assertLess(perf[s][0]['gyro_z_agg'],
                        perf[s][3]['gyro_z_agg'])
        tub.close()

    def test_two_field_ranking_time_distance_only(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=3.5)],
            [DrivingProfile(speed=2.0)],
            [DrivingProfile(speed=1.0)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        for lap_idx in range(3):
            lap_rank = perf[s][lap_idx]
            self.assertEqual(len(lap_rank), 2)
            self.assertIn('time', lap_rank)
            self.assertIn('distance', lap_rank)
            self.assertNotIn('gyro_z_agg', lap_rank)
        tub.close()

    def test_ranking_order_matches_config_order(self):
        path1 = self._make_tub_dir()
        path2 = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=3.5, curvature=0.8)],
            [DrivingProfile(speed=1.5, curvature=0.1)],
            [DrivingProfile(speed=2.5, curvature=0.4)],
        ]
        tub1 = create_multilap_tub(path1, profiles)
        tub2 = create_multilap_tub(path2, profiles)

        aggs_a = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        aggs_b = [
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
            FieldAggregationSpec(output_key='time'),
        ]

        perf_a = self._rank_laps(tub1, aggs_a)
        perf_b = self._rank_laps(tub2, aggs_b)
        s1 = self._get_session_id(tub1)
        s2 = self._get_session_id(tub2)

        # Same per-field rankings regardless of config order
        self.assertEqual(perf_a[s1][0]['time'],
                         perf_b[s2][0]['time'])
        self.assertEqual(perf_a[s1][0]['gyro_z_agg'],
                         perf_b[s2][0]['gyro_z_agg'])

        # lap_pct vector order differs based on config order
        keys_a = ['time', 'gyro_z_agg']
        keys_b = ['gyro_z_agg', 'time']
        pct_a = [perf_a[s1][0][k] for k in keys_a]
        pct_b = [perf_b[s2][0][k] for k in keys_b]
        # Lap 0 (fast, lots of turning): best time, worst gyro
        # So time rank != gyro rank
        self.assertNotEqual(pct_a[0], pct_b[0])
        tub1.close()
        tub2.close()


# ════════════════════════════════════════════════════════════════════
# TestDrivingBehaviorDiscrimination
# ════════════════════════════════════════════════════════════════════


class TestDrivingBehaviorDiscrimination(SegmentRankingTestBase):
    """Test that ranking distinguishes meaningful driving styles."""

    def test_fast_smooth_vs_fast_jerky(self):
        path = self._make_tub_dir()
        smooth_seg = DrivingProfile(
            speed=2.5, curvature=0.5, smoothness=0.95)
        jerky_seg = DrivingProfile(
            speed=2.5, curvature=0.5, smoothness=0.2)

        # Use single-segment laps so the smoothness difference
        # is not diluted by a shared neutral segment.
        profiles = [
            [smooth_seg],
            [jerky_seg],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
            FieldAggregationSpec(
                field='car/accel', index=1, output_key='lat_g_max',
                transform=abs, aggregation='max'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Smooth lap has lower avg abs gyro
        self.assertLess(perf[s][0]['gyro_z_agg'],
                        perf[s][1]['gyro_z_agg'])
        # Smooth lap has lower peak lateral accel
        self.assertLess(perf[s][0]['lat_g_max'],
                        perf[s][1]['lat_g_max'])
        tub.close()

    def test_slow_stable_vs_fast_unstable(self):
        path = self._make_tub_dir()
        slow_stable = DrivingProfile(
            speed=1.5, curvature=0.3, smoothness=0.95)
        fast_unstable = DrivingProfile(
            speed=3.5, curvature=0.3, smoothness=0.3)

        profiles = [
            [slow_stable],
            [fast_unstable],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Both laps have same duration (same records_per_segment
        # and segment_time_ms), so time rankings are arbitrary.
        # With ascending sort, lower distance ranks first.
        # Slow car has less distance -> lower quantile.
        self.assertLess(perf[s][0]['distance'],
                        perf[s][1]['distance'])
        # Slow stable ranks better on gyro (lower noise)
        self.assertLess(perf[s][0]['gyro_z_agg'],
                        perf[s][1]['gyro_z_agg'])
        tub.close()

    def test_tight_racing_line_vs_wide_line(self):
        path = self._make_tub_dir()
        tight = DrivingProfile(speed=2.0, curvature=0.8)
        wide = DrivingProfile(speed=3.5, curvature=0.2)

        lap_segment_times = [[2000], [2000]]
        tub = create_tub_with_varied_segment_times(
            path, 2, 1, lap_segment_times,
            profiles=[[tight], [wide]])
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/accel', index=1, output_key='lat_g',
                transform=abs, aggregation='max'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Tight has less distance -> better distance ranking
        self.assertLess(perf[s][0]['distance'],
                        perf[s][1]['distance'])
        # Tight has higher lateral G -> worse lat_g ranking
        self.assertGreater(perf[s][0]['lat_g'],
                           perf[s][1]['lat_g'])
        tub.close()

    def test_aggressive_vs_conservative_braking(self):
        path = self._make_tub_dir()
        profiles = [
            [BRAKING_HARD],
            [BRAKING_GENTLE],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/accel', index=0, output_key='brake_min',
                aggregation='min'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Hard braking -> more negative min -> ranks first ascending
        self.assertLess(perf[s][0]['brake_min'],
                        perf[s][1]['brake_min'])
        tub.close()


# ════════════════════════════════════════════════════════════════════
# TestTransformFunctions
# ════════════════════════════════════════════════════════════════════


class TestTransformFunctions(SegmentRankingTestBase):
    """Test transform functions applied before aggregation."""

    def test_abs_transform(self):
        path1 = self._make_tub_dir()
        path2 = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.0, curvature=0.5)],
            [DrivingProfile(speed=2.0, curvature=-0.5)],
            [DrivingProfile(speed=2.0, curvature=1.0)],
        ]
        tub1 = create_multilap_tub(path1, profiles)
        tub2 = create_multilap_tub(path2, profiles)

        aggs_with_abs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_abs',
                transform=abs, aggregation='avg'),
        ]
        aggs_without_abs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_raw',
                transform=None, aggregation='avg'),
        ]

        perf_abs = self._rank_laps(tub1, aggs_with_abs)
        perf_raw = self._rank_laps(tub2, aggs_without_abs)
        s1 = self._get_session_id(tub1)
        s2 = self._get_session_id(tub2)

        # With abs: left and right turns (±0.5) have similar avg(abs)
        # but noise makes them slightly different, so use approximate
        # equality. Both should rank better than curvature=1.0.
        self.assertAlmostEqual(perf_abs[s1][0]['gyro_abs'],
                               perf_abs[s1][1]['gyro_abs'],
                               delta=0.4)
        # Without abs: left (+0.5) and right (-0.5) rank differently
        self.assertNotEqual(perf_raw[s2][0]['gyro_raw'],
                            perf_raw[s2][1]['gyro_raw'])
        tub1.close()
        tub2.close()

    def test_square_transform(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=2.0, curvature=0.5, smoothness=0.95)],
            [DrivingProfile(speed=2.0, curvature=0.5, smoothness=0.2)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_sq',
                transform=lambda x: x ** 2, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Noisy lap -> higher avg(gyro^2) due to spikes
        self.assertLess(perf[s][0]['gyro_sq'],
                        perf[s][1]['gyro_sq'])
        tub.close()

    def test_identity_transform(self):
        path = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=3.0,
                            braking_profile='brake_then_accel',
                            braking_intensity=5.0)],
            [DrivingProfile(speed=3.0,
                            braking_profile='brake_then_accel',
                            braking_intensity=1.5)],
            [DrivingProfile(speed=3.0, speed_delta=1.0)],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/accel', index=0, output_key='accel_min',
                transform=None, aggregation='min'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        # Most negative min ranks first ascending
        self.assertLess(perf[s][0]['accel_min'],
                        perf[s][1]['accel_min'])
        self.assertLess(perf[s][1]['accel_min'],
                        perf[s][2]['accel_min'])
        tub.close()

    def test_reverse_ranking(self):
        path1 = self._make_tub_dir()
        path2 = self._make_tub_dir()
        profiles = [
            [DrivingProfile(speed=1.0)],
            [DrivingProfile(speed=2.5)],
            [DrivingProfile(speed=4.0)],
        ]
        tub1 = create_multilap_tub(path1, profiles)
        tub2 = create_multilap_tub(path2, profiles)

        field_aggs_asc = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/speed', output_key='speed_avg',
                aggregation='avg', reverse=False),
        ]
        field_aggs_desc = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/speed', output_key='speed_avg',
                aggregation='avg', reverse=True),
        ]

        perf_asc = self._rank_laps(tub1, field_aggs_asc)
        perf_desc = self._rank_laps(tub2, field_aggs_desc)
        s1 = self._get_session_id(tub1)
        s2 = self._get_session_id(tub2)

        # Ascending: lowest speed (1.0) ranks best
        self.assertLess(perf_asc[s1][0]['speed_avg'],
                        perf_asc[s1][2]['speed_avg'])
        # Descending: highest speed (4.0) ranks best
        self.assertLess(perf_desc[s2][2]['speed_avg'],
                        perf_desc[s2][0]['speed_avg'])
        tub1.close()
        tub2.close()


# ════════════════════════════════════════════════════════════════════
# TestTubWriterIntegration
# ════════════════════════════════════════════════════════════════════


class TestTubWriterIntegration(SegmentRankingTestBase):
    """Test ranking through the official TubWriter.run() interface."""

    def test_tub_writer_produces_valid_rankings(self):
        path = self._make_tub_dir()
        profiles = [
            [TURN_SMOOTH, STRAIGHT_FAST],
            [TURN_AGGRESSIVE, STRAIGHT_FAST],
            [TURN_JERKY, STRAIGHT_FAST],
        ]
        tub = create_multilap_tub_via_writer(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        self.assertEqual(len(perf[s]), 3)
        for lap_idx in range(3):
            self.assertEqual(
                set(perf[s][lap_idx].keys()),
                {'time', 'distance', 'gyro_z_agg'})
            for key in perf[s][lap_idx]:
                r = perf[s][lap_idx][key]
                self.assertGreater(r, 0)
                self.assertLessEqual(r, 1.0)
        tub.close()

    def test_tub_writer_vs_direct_write_consistency(self):
        path1 = self._make_tub_dir()
        path2 = self._make_tub_dir()
        profiles = [
            [TURN_SMOOTH],
            [TURN_AGGRESSIVE],
            [TURN_JERKY],
        ]
        tub_direct = create_multilap_tub(path1, profiles)
        tub_writer = create_multilap_tub_via_writer(path2, profiles)

        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]

        perf_d = self._rank_laps(tub_direct, field_aggs)
        perf_w = self._rank_laps(tub_writer, field_aggs)
        sd = self._get_session_id(tub_direct)
        sw = self._get_session_id(tub_writer)

        # Gyro rankings should be in same ORDER
        gyro_order_d = sorted(
            range(3),
            key=lambda i: perf_d[sd][i]['gyro_z_agg'])
        gyro_order_w = sorted(
            range(3),
            key=lambda i: perf_w[sw][i]['gyro_z_agg'])
        self.assertEqual(gyro_order_d, gyro_order_w)
        tub_direct.close()
        tub_writer.close()

    def test_tub_without_images(self):
        path = self._make_tub_dir()
        profiles = [
            [TURN_SMOOTH],
            [TURN_AGGRESSIVE],
        ]
        tub = create_multilap_tub(
            path, profiles,
            inputs=MINIMAL_INPUTS, types=MINIMAL_TYPES)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        self.assertEqual(len(perf[s]), 2)
        self.assertLess(perf[s][0]['gyro_z_agg'],
                        perf[s][1]['gyro_z_agg'])
        tub.close()


# ════════════════════════════════════════════════════════════════════
# TestOnTheFlySegmentComputation
# ════════════════════════════════════════════════════════════════════


class TestOnTheFlySegmentComputation(SegmentRankingTestBase):
    """Test segment rankings with car/segment field in records."""

    def test_ranking_from_record_field(self):
        path = self._make_tub_dir()
        segment_inputs = MINIMAL_INPUTS + ['car/segment']
        segment_types = MINIMAL_TYPES + ['int']
        profiles = [
            [TURN_SMOOTH, STRAIGHT_FAST],
            [TURN_AGGRESSIVE, STRAIGHT_FAST],
            [TURN_JERKY, STRAIGHT_FAST],
        ]
        tub = create_multilap_tub(
            path, profiles,
            inputs=segment_inputs, types=segment_types)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_segments(tub, field_aggs)
        s = self._get_session_id(tub)
        for lap in range(3):
            self.assertIn(lap, perf[s])
            self.assertEqual(len(perf[s][lap]), 2)
            for seg in range(2):
                self.assertIn(seg, perf[s][lap])
        tub.close()

    def test_segment_rankings_structure_with_multiple_fields(self):
        path = self._make_tub_dir()
        segment_inputs = MINIMAL_INPUTS + ['car/segment']
        segment_types = MINIMAL_TYPES + ['int']
        profiles = [
            [TURN_SMOOTH, STRAIGHT_FAST],
            [TURN_AGGRESSIVE, STRAIGHT_FAST],
            [TURN_JERKY, STRAIGHT_FAST],
        ]
        tub = create_multilap_tub(
            path, profiles,
            inputs=segment_inputs, types=segment_types)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
            FieldAggregationSpec(
                field='car/accel', index=0, output_key='brake_min',
                aggregation='min'),
        ]
        perf = self._rank_segments(tub, field_aggs)
        s = self._get_session_id(tub)
        for lap in range(3):
            for seg in range(2):
                rank = perf[s][lap][seg]
                self.assertEqual(
                    set(rank.keys()),
                    {'time', 'distance', 'gyro_z_agg', 'brake_min'})
        tub.close()


# ════════════════════════════════════════════════════════════════════
# TestRankingEdgeCases
# ════════════════════════════════════════════════════════════════════


class TestRankingEdgeCases(SegmentRankingTestBase):
    """Test graceful handling of degenerate inputs."""

    def test_single_lap_ranking(self):
        path = self._make_tub_dir()
        profiles = [
            [TURN_SMOOTH],
            [TURN_AGGRESSIVE],
        ]
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs, use_lap_0=False)
        s = self._get_session_id(tub)
        self.assertEqual(len(perf[s]), 1)
        self.assertIn(1, perf[s])
        # Single lap -> ranking = 1.0
        self.assertEqual(perf[s][1]['time'], 1.0)
        self.assertEqual(perf[s][1]['gyro_z_agg'], 1.0)
        tub.close()

    def test_missing_field_in_record(self):
        path = self._make_tub_dir()
        # Create tub WITHOUT car/accel field
        inputs = ['car/lap', 'car/distance', 'car/gyro', 'car/speed']
        types = ['int', 'float', 'vector', 'float']
        profiles = [
            [DrivingProfile(speed=3.0)],
            [DrivingProfile(speed=2.0)],
        ]
        tub = create_multilap_tub(
            path, profiles, inputs=inputs, types=types)
        # Config references car/accel which doesn't exist
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/accel', index=0, output_key='accel_agg',
                aggregation='avg'),
        ]
        # Should not crash even when a field is missing.
        # All laps become invalid when accel_agg cannot be
        # computed, so the result is empty.
        perf = self._rank_laps(tub, field_aggs)
        self.assertIsNotNone(perf)
        # No valid laps since the required field is absent
        self.assertEqual(len(perf), 0)

        # Verify that using only valid fields still works
        field_aggs_valid = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(output_key='distance'),
        ]
        perf2 = self._rank_laps(tub, field_aggs_valid)
        s = self._get_session_id(tub)
        self.assertIn(s, perf2)
        self.assertEqual(len(perf2[s]), 2)
        self.assertIn('time', perf2[s][0])
        tub.close()

    def test_field_with_omitted_values(self):
        path = self._make_tub_dir()
        inputs = ['car/lap', 'car/distance', 'car/gyro', 'car/speed']
        types = ['int', 'float', 'vector', 'float']
        tub = Tub(path, inputs, types)

        start_time_ms = 1000000
        for lap in range(2):
            for i in range(10):
                record = {
                    'car/lap': lap,
                    'car/distance': float(lap * 50 + i * 5),
                    'car/gyro': [0.0, 0.0, 0.5],
                    '_timestamp_ms': start_time_ms
                    + lap * 10000 + i * 1000,
                }
                # Only include speed for some records
                if i % 3 != 0:
                    record['car/speed'] = 2.0
                tub.write_record(record)
        # Final record
        tub.write_record({
            'car/lap': 2,
            'car/distance': 100.0,
            'car/gyro': [0.0, 0.0, 0.5],
            'car/speed': 2.0,
            '_timestamp_ms': start_time_ms + 20000,
        })

        # Close and reopen to populate sessions metadata
        tub.close()
        tub = Tub(path, read_only=True)

        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/speed', output_key='speed_avg',
                aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        self.assertIn('speed_avg', perf[s][0])
        self.assertIn('speed_avg', perf[s][1])
        tub.close()

    def test_all_segments_same_performance(self):
        path = self._make_tub_dir()
        profile = DrivingProfile(speed=2.5, curvature=0.5)
        profiles = [[profile, profile]] * 3
        tub = create_multilap_tub(path, profiles)
        field_aggs = [
            FieldAggregationSpec(output_key='time'),
            FieldAggregationSpec(
                field='car/gyro', index=2, output_key='gyro_z_agg',
                transform=abs, aggregation='avg'),
        ]
        perf = self._rank_laps(tub, field_aggs)
        s = self._get_session_id(tub)
        self.assertEqual(len(perf[s]), 3)
        for lap_idx in range(3):
            self.assertGreater(perf[s][lap_idx]['time'], 0)
            self.assertLessEqual(perf[s][lap_idx]['time'], 1.0)
            self.assertGreater(perf[s][lap_idx]['gyro_z_agg'], 0)
            self.assertLessEqual(perf[s][lap_idx]['gyro_z_agg'], 1.0)
        # Rankings are spread: all different despite same data
        time_ranks = sorted(
            perf[s][i]['time'] for i in range(3))
        self.assertEqual(len(set(time_ranks)), 3)
        tub.close()


if __name__ == '__main__':
    unittest.main()
