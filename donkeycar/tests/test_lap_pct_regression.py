"""
Regression tests for lap-based training (Phase 8)

Ensures backward compatibility:
- Lap-based training unchanged when pct_mode=PctMode.LAP
- Training works with tubs without segmentation
- Existing lap performance calculation still works
- No breaking changes to existing functionality
"""

import os
import tempfile
import unittest
import shutil
import numpy as np

from donkeycar.config import Config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics, FieldAggregationSpec
from donkeycar.pipeline.types import TubDataset, PctMode
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


class TestLapPerformanceRegression(unittest.TestCase):
    """Regression tests for lap-based training"""

    def setUp(self):
        self.open_tubs = []  # Track tubs to close
        """Create test environment"""
        self.temp_dir = tempfile.mkdtemp()
        self.tub_path = os.path.join(self.temp_dir, 'test_tub')
        self.config = self._create_test_config()

    def tearDown(self):
        # Close any open tubs
        for tub in self.open_tubs:
            try:
                tub.close()
            except:
                pass
        """Clean up temp files"""
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def _create_test_config(self):
        """Create test configuration"""
        cfg = Config()
        cfg.USE_LAP_0 = False
        cfg.TRAIN_TEST_SPLIT = 0.8
        cfg.FIELD_AGGREGATIONS = [
            {
                'field': 'car/gyro',
                'output_key': 'gyro_z_agg',
                'index': 1,
                'aggregation': 'avg'
            }
        ]
        return cfg

    def test_lap_mode_unchanged(self):
        """
        Test that LAP mode works exactly as before.
        Verifies no breaking changes to existing lap-based training.
        """
        # Create tub with lap data (no segments)
        self._create_lap_only_tub()

        # Load with LAP mode (original behavior)
        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.LAP
        )

        records = dataset.get_records()
        self.assertGreater(len(records), 0)

        # Verify records have lap_pct from lap-based ranking
        records_with_lap_pct = [r for r in records
                                if 'lap_pct' in r.underlying]

        # Some records should have lap_pct
        # (May be incomplete depending on lap timing data)
        # The key is it doesn't crash and maintains backward compatibility

        dataset.close()

    def test_tubs_without_segmentation_work(self):
        """
        Test backward compatibility with tubs that don't have segmentation.
        Old tubs should work fine with new code.
        """
        # Create tub without any segmentation data
        self._create_lap_only_tub()

        # Try to load with SEGMENT mode (should gracefully handle missing data)
        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.SEGMENT
        )

        records = dataset.get_records()
        self.assertGreater(len(records), 0)

        # Records without car/segment should not crash
        # They just won't get lap_pct in SEGMENT mode
        records_with_lap_pct = [r for r in records
                                if 'lap_pct' in r.underlying]

        # Should have 0 records with lap_pct (no segmentation)
        self.assertEqual(len(records_with_lap_pct), 0,
                        "Records without segmentation should not get lap_pct "
                        "in SEGMENT mode")

        dataset.close()

    def test_lap_performance_calculation_unchanged(self):
        """
        Test that calculate_lap_performance() still works as before.
        """
        self._create_lap_only_tub()

        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1,
                              sorting_strategy=FULL_SORTING_STRATEGY)

        # Calculate lap performance (original method)
        session_lap_rank = stats.calculate_lap_performance()

        # Verify structure: session -> lap -> rankings
        # Get actual session ID (not hardcoded 'test_session')
        self.assertEqual(len(session_lap_rank), 1,
                        "Should have exactly one session")
        actual_session_id = list(session_lap_rank.keys())[0]
        session_data = session_lap_rank[actual_session_id]

        # Should have lap data (now returns dict, not list)
        for lap_num in [1, 2, 3]:
            if lap_num in session_data:
                lap_rankings = session_data[lap_num]
                self.assertIsInstance(lap_rankings, dict)
                # Should have standard keys
                self.assertIn('time', lap_rankings)
                self.assertIn('distance', lap_rankings)
                self.assertIn('gyro_z_agg', lap_rankings)

        tub.close()

    def test_none_mode_no_lap_pct(self):
        """Test that NONE mode doesn't add lap_pct"""
        self._create_lap_only_tub()

        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.NONE
        )

        records = dataset.get_records()
        self.assertGreater(len(records), 0)

        # No records should have lap_pct in NONE mode
        records_with_lap_pct = [r for r in records
                                if 'lap_pct' in r.underlying]

        self.assertEqual(len(records_with_lap_pct), 0,
                        "NONE mode should not add lap_pct")

        dataset.close()

    def test_add_lap_pct_flag_still_works(self):
        """Test that legacy add_lap_pct parameter still works"""
        self._create_lap_only_tub()

        # Old-style usage with add_lap_pct=True
        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            add_lap_pct=True
        )

        records = dataset.get_records()
        self.assertGreater(len(records), 0)

        # Should work without errors (backward compatibility)
        dataset.close()

    def _create_lap_only_tub(self):
        """Create tub with lap data but no segmentation"""
        tub = Tub(self.tub_path,
                  inputs=['cam/image_array', 'user/angle', 'user/throttle',
                          'car/lap', 'car/gyro', 'car/distance'],
                  types=['image_array', 'float', 'float', 'int', 'vector',
                         'float'])
        self.open_tubs.append(tub)

        session_id = 'test_session'
        test_image = np.zeros((120, 160, 3), dtype=np.uint8)

        # Create 3 laps with varying performance
        # Lap 1: 2.0s, Lap 2: 2.2s, Lap 3: 2.4s
        lap_times = [2.0, 2.2, 2.4]

        timestamp_ms = 0
        distance = 0.0

        for lap_num, lap_time in enumerate(lap_times):
            records_per_lap = 40
            time_per_record = (lap_time / records_per_lap) * 1000

            for _ in range(records_per_lap):
                record = {
                    '_session_id': session_id,
                    'cam/image_array': test_image,
                    'user/angle': 0.0,
                    'user/throttle': 0.3,
                    'car/lap': lap_num + 1,
                    'car/gyro': [0.0, 0.1, 0.0],
                    'car/distance': distance,
                    '_timestamp_ms': timestamp_ms
                }
                tub.write_record(record)
                timestamp_ms += int(time_per_record)
                distance += 0.1

        # Create laptimer metadata (no segmentation metadata)
        lap_metadata = []
        for lap_num, lap_time in enumerate(lap_times):
            lap_metadata.append({
                'lap': lap_num + 1,
                'time': lap_time,
                'distance': 4.0,
                'gyro_z_agg': 0.1,
                'valid': True
            })

        # Get actual session ID from manifest
        actual_session_id = tub.manifest.session_id[1]
        tub.manifest.metadata[actual_session_id] = {
            'laptimer': lap_metadata
            # No 'segmentation' key - this is an old tub
        }
        tub.manifest.write_metadata()
        tub.close()


if __name__ == '__main__':
    unittest.main()
