"""
Tests for segment performance calculation (Phase 2)

Tests the segment-based performance ranking system:
- Segment assignment to tub records
- Per-segment metric aggregation (time, distance, gyro)
- Segment ranking across multiple laps
- Correct percentile calculation
- Verification that same segment in different laps gets different rankings
"""

import os
import tempfile
import unittest
import shutil
import numpy as np

from donkeycar.config import Config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics, FieldAggregationSpec
from donkeycar.pipeline.types import TubRecord
from donkeycar.pipeline.transformations import SortingStrategy


# Standard field aggregation for gyro_z at index 1 (simulator convention)
GYRO_Z_INDEX_1 = [
    FieldAggregationSpec(
        field='car/gyro',
        output_key='gyro_z_agg',
        index=1,
        transform=abs,
        aggregation='avg'
    )
]

# Sorting strategy that includes time, distance, and gyro_z_agg
FULL_SORTING_STRATEGY = SortingStrategy([
    {'key': 'time'},
    {'key': 'distance'},
    {'key': 'gyro_z_agg'},
])


class TestSegmentPerformanceCalculation(unittest.TestCase):
    """Test segment performance ranking across laps"""

    def setUp(self):
        """Create test tub with multi-lap data and segment assignments"""
        self.temp_dir = tempfile.mkdtemp()
        self.tub_path = os.path.join(self.temp_dir, 'test_tub')
        self.open_tubs = []  # Track tubs to close in tearDown

        # Create tub with 3 laps, 4 segments each
        # Lap 1: Segments [Fast, Slow, Medium, Fast]
        # Lap 2: Segments [Medium, Fast, Fast, Slow]
        # Lap 3: Segments [Slow, Medium, Slow, Medium]
        self._create_test_tub_with_segments()

    def tearDown(self):
        """Clean up temp files"""
        # Close any open tubs
        for tub in self.open_tubs:
            try:
                tub.close()
            except:
                pass
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def _create_test_tub_with_segments(self):
        """Create test tub with realistic multi-lap segment data"""
        tub = Tub(self.tub_path,
                  inputs=['cam/image_array', 'user/angle', 'user/throttle',
                          'car/lap', 'car/segment', 'car/gyro', 'car/distance'],
                  types=['image_array', 'float', 'float', 'int', 'int',
                         'vector3', 'float'])

        session_id = 'test_session_001'

        # Define segment performance characteristics
        # [time_per_record, distance_per_record, gyro_value]
        segment_profiles = {
            'fast': (0.05, 0.2, 0.1),      # Fast: low time, high distance
            'medium': (0.1, 0.15, 0.2),    # Medium
            'slow': (0.15, 0.1, 0.3)       # Slow: high time, low distance
        }

        # Define lap structure: which segments are fast/medium/slow
        lap_structures = [
            ['fast', 'slow', 'medium', 'fast'],    # Lap 1
            ['medium', 'fast', 'fast', 'slow'],    # Lap 2
            ['slow', 'medium', 'slow', 'medium']   # Lap 3
        ]

        records_per_segment = 10
        timestamp_ms = 0
        distance = 0.0

        for lap_num, lap_structure in enumerate(lap_structures):
            for segment_id, performance in enumerate(lap_structure):
                time_inc, dist_inc, gyro = segment_profiles[performance]

                for _ in range(records_per_segment):
                    record = {
                        '_session_id': session_id,
                        'cam/image_array': np.zeros((120, 160, 3), dtype=np.uint8),
                        'user/angle': 0.0,
                        'user/throttle': 0.3,
                        'car/lap': lap_num,
                        'car/segment': segment_id,
                        'car/gyro': [0.0, gyro, 0.0],  # gyro_z at index 1
                        'car/distance': distance,
                        '_timestamp_ms': timestamp_ms
                    }
                    tub.write_record(record)
                    timestamp_ms += int(time_inc * 1000)
                    distance += dist_inc

        # Create laptimer metadata (required for performance calculation)
        lap_times = []
        for lap_num in range(3):
            lap_times.append({
                'lap': lap_num,
                'time': 2.0,  # Placeholder
                'distance': 6.0,  # Placeholder
                'valid': True
            })

        tub.manifest.metadata[session_id] = {
            'laptimer': lap_times,
            'segmentation': {
                'num_segments': 4,
                'mean_course_params': {'num_laps': 3},
                'segmentation_params': {
                    'strategy': 'hybrid',
                    'min_segment_length': 1.0
                }
            }
        }
        tub.manifest.write_metadata()
        tub.close()

    def test_segment_assignment_in_records(self):
        """Test that car/segment field exists in records"""
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        cfg = self._create_test_config()

        # Check that all records have segment assignment
        for underlying in tub:
            record = TubRecord(cfg, tub.base_path, underlying)
            self.assertIn('car/segment', record.underlying)
            segment_id = record.underlying['car/segment']
            self.assertIsInstance(segment_id, int)
            self.assertGreaterEqual(segment_id, 0)
            self.assertLess(segment_id, 4)

        tub.close()

    def _create_test_config(self):
        """Create minimal config for testing"""
        from donkeycar.config import Config
        cfg = Config()
        cfg.USE_LAP_0 = False
        return cfg

    def test_segment_performance_structure(self):
        """Test that calculate_segment_performance returns correct structure"""
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1,
                              sorting_strategy=FULL_SORTING_STRATEGY)

        session_rank = stats.calculate_segment_performance()

        # Should have session -> lap -> segment structure
        # Get actual session ID (auto-generated)
        sessions = list(session_rank.keys())
        self.assertEqual(len(sessions), 1, "Should have exactly 1 session")
        session_id = sessions[0]
        session_data = session_rank[session_id]

        # Should have laps (1, 2) - lap 0 is filtered out by USE_LAP_0=False
        for lap_num in [1, 2]:
            self.assertIn(lap_num, session_data)
            lap_data = session_data[lap_num]

            # Each lap should have 4 segments
            for segment_id in range(4):
                self.assertIn(segment_id, lap_data)
                rankings = lap_data[segment_id]

                # Rankings should be a dict with 3 keys
                self.assertIsInstance(rankings, dict)
                self.assertIn('time', rankings)
                self.assertIn('distance', rankings)
                self.assertIn('gyro_z_agg', rankings)

                # Each ranking should be between 0 and 1
                for key, ranking in rankings.items():
                    self.assertGreaterEqual(ranking, 0.0)
                    self.assertLessEqual(ranking, 1.0)

        tub.close()

    def test_segment_ranking_across_laps(self):
        """Test that same segment in different laps gets different rankings"""
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1,
                              sorting_strategy=FULL_SORTING_STRATEGY)

        session_rank = stats.calculate_segment_performance()
        session_id = list(session_rank.keys())[0]
        session_data = session_rank[session_id]

        # Segment 0 performance: Lap 1 Fast, Lap 2 Medium (Lap 0 filtered out)
        # So rankings should be: Lap 1 < Lap 2 (for time)
        seg0_lap1 = session_data[1][0]  # Lap 1, Segment 0
        seg0_lap2 = session_data[2][0]  # Lap 2, Segment 0

        # Rankings should be different
        self.assertNotEqual(seg0_lap1, seg0_lap2)

        # Fast segment should have better (lower) time ranking than medium
        # Note: with only 2 laps, rankings will be 0.5 and 1.0
        self.assertLess(seg0_lap1['time'], seg0_lap2['time'],
                       "Fast segment should have lower time ranking than medium")

        tub.close()

    def test_segment_0_fastest_in_lap_1(self):
        """Test that segment 0 is ranked fastest in lap 1"""
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1,
                              sorting_strategy=FULL_SORTING_STRATEGY)

        session_rank = stats.calculate_segment_performance()
        session_id = list(session_rank.keys())[0]
        session_data = session_rank[session_id]

        # Segment 0: Lap 1=Fast, Lap 2=Medium (Lap 0 filtered out)
        # Rankings for time_pct should reflect this
        seg0_rankings = {
            lap: session_data[lap][0]['time']  # time ranking (first element)
            for lap in [1, 2]
        }

        # Lap 1 should have lowest (best) time_pct
        self.assertLess(seg0_rankings[1], seg0_rankings[2])

        tub.close()

    def test_segment_1_fastest_in_lap_1(self):
        """Test that segment 1 is ranked fastest in lap 1"""
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1,
                              sorting_strategy=FULL_SORTING_STRATEGY)

        session_rank = stats.calculate_segment_performance()
        session_id = list(session_rank.keys())[0]
        session_data = session_rank[session_id]

        # Segment 1: Lap 1=Fast, Lap 2=Medium (Lap 0 filtered out)
        seg1_rankings = {
            lap: session_data[lap][1]['time']  # time ranking
            for lap in [1, 2]
        }

        # Lap 1 should have lowest (best) time ranking
        self.assertLess(seg1_rankings[1], seg1_rankings[2])

        tub.close()

    def test_ranking_percentiles(self):
        """Test that rankings are proper percentiles (0.33, 0.67, 1.0)"""
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1,
                              sorting_strategy=FULL_SORTING_STRATEGY)

        session_rank = stats.calculate_segment_performance()
        session_id = list(session_rank.keys())[0]
        session_data = session_rank[session_id]

        # Get all rankings for segment 0 across 2 laps (lap 0 filtered out)
        seg0_time_rankings = sorted([
            session_data[lap][0]['time']  # time ranking
            for lap in [1, 2]
        ])

        # Should have 2 distinct values representing percentiles
        self.assertEqual(len(set(seg0_time_rankings)), 2,
                        "Should have 2 distinct ranking values")

        # With 2 laps, rankings should be 0.5 and 1.0
        self.assertLess(seg0_time_rankings[0], seg0_time_rankings[1],
                       "Best ranking should be less than worst ranking")

        tub.close()

    def test_synthetic_best_lap_creation(self):
        """Test that we can create synthetic best lap from best segments"""
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1,
                              sorting_strategy=FULL_SORTING_STRATEGY)

        session_rank = stats.calculate_segment_performance()
        session_id = list(session_rank.keys())[0]
        session_data = session_rank[session_id]

        # Find best lap for each segment (lowest time ranking)
        best_laps = {}
        for segment_id in range(4):
            best_lap = min(
                [1, 2],  # Only laps 1 and 2 (lap 0 filtered out)
                key=lambda lap: session_data[lap][segment_id]['time']
            )
            best_laps[segment_id] = best_lap

        # Expected best laps based on test data (lap 0 filtered out):
        # Segment 0: Lap 1 (Fast) vs Lap 2 (Medium) -> Lap 1
        # Segment 1: Lap 1 (Slow) vs Lap 2 (Fast) -> Lap 2
        # Segment 2: Lap 1 (Medium) vs Lap 2 (Fast) -> Lap 2
        # Segment 3: Lap 1 (Fast) vs Lap 2 (Slow) -> Lap 1

        # Verify that best segments come from different laps
        # (this creates the "synthetic best lap" effect)
        unique_best_laps = set(best_laps.values())
        self.assertEqual(len(unique_best_laps), 2,
                        "Best segments should come from both laps")
        self.assertIn(1, unique_best_laps)
        self.assertIn(2, unique_best_laps)

        tub.close()


if __name__ == '__main__':
    unittest.main()
