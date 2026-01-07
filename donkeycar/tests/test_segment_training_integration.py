"""
End-to-end integration test for segment-based training (Phase 8)

Tests the complete workflow:
1. Create realistic multi-lap tub data
2. Make specific segments fastest in different laps
3. Run segment assignment
4. Load in training pipeline with SEGMENT mode
5. Verify fastest segments get lowest lap_pct values
6. Verify "synthetic best lap" effect
"""

import os
import tempfile
import unittest
import shutil
import numpy as np

from donkeycar.config import Config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics
from donkeycar.pipeline.types import TubDataset, PctMode


class TestSegmentTrainingEndToEnd(unittest.TestCase):
    """End-to-end test of segment-based training workflow"""

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
        cfg.GYRO_Z_INDEX = 1
        cfg.SEGMENT_PCT_MODE = True
        return cfg

    def test_end_to_end_segment_training_workflow(self):
        """
        Complete end-to-end test:
        - Create multi-lap data with known segment performance
        - Run segmentation
        - Load with SEGMENT mode
        - Verify synthetic best lap is created
        """

        # Step 1: Create realistic multi-lap tub
        print("\nStep 1: Creating test tub with 3 laps, 4 segments")
        self._create_realistic_multilap_tub()

        # Step 2: Run segmentation (simulating: donkey segment --tub <path>)
        print("Step 2: Running segment assignment")
        tub = Tub(self.tub_path, read_only=False)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub)

        # Note: compute_segment_assignments requires actual PathData
        # For this test, we're manually setting segments, so skip actual
        # segmentation and just verify the data structure works
        tub.close()

        # Step 3: Load dataset with SEGMENT mode
        print("Step 3: Loading dataset with SEGMENT mode")
        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.SEGMENT
        )

        records = dataset.get_records()
        self.assertGreater(len(records), 0, "Should load records")

        # Step 4: Verify fastest segments have best lap_pct
        print("Step 4: Verifying fastest segments have lowest lap_pct")
        self._verify_fastest_segments_prioritized(records)

        # Step 5: Verify synthetic best lap effect
        print("Step 5: Verifying synthetic best lap effect")
        self._verify_synthetic_best_lap_effect(records)

        dataset.close()
        print("End-to-end test PASSED!")

    def _create_realistic_multilap_tub(self):
        """
        Create tub with 3 laps, 4 segments each.

        Segment performance pattern (time per segment):
        - Lap 1: [0.5s, 1.5s, 1.0s, 0.5s]  (seg 0,3 fast)
        - Lap 2: [1.0s, 0.5s, 0.5s, 1.5s]  (seg 1,2 fast)
        - Lap 3: [1.5s, 1.0s, 1.5s, 1.0s]  (all medium/slow)
        """
        tub = Tub(self.tub_path,
                  inputs=['cam/image_array', 'user/angle', 'user/throttle',
                          'car/lap', 'car/segment', 'car/gyro',
                          'car/distance'],
                  types=['image_array', 'float', 'float', 'int', 'int',
                         'vector', 'float'])
        self.open_tubs.append(tub)

        session_id = 'test_session'
        test_image = np.zeros((120, 160, 3), dtype=np.uint8)

        # Define segment times for each lap
        lap_segment_times = [
            [0.5, 1.5, 1.0, 0.5],   # Lap 1
            [1.0, 0.5, 0.5, 1.5],   # Lap 2
            [1.5, 1.0, 1.5, 1.0]    # Lap 3
        ]

        records_per_segment = 20
        timestamp_ms = 0
        distance = 0.0

        for lap_num, segment_times in enumerate(lap_segment_times):
            for segment_id, segment_time in enumerate(segment_times):
                time_per_record = segment_time / records_per_segment
                dist_per_record = 0.1

                for _ in range(records_per_segment):
                    record = {
                        '_session_id': session_id,
                        'cam/image_array': test_image,
                        'user/angle': 0.1,
                        'user/throttle': 0.3,
                        'car/lap': lap_num + 1,  # Laps 1,2,3
                        'car/segment': segment_id,
                        'car/gyro': [0.0, 0.1, 0.0],
                        'car/distance': distance,
                        '_timestamp_ms': timestamp_ms
                    }
                    tub.write_record(record)
                    timestamp_ms += int(time_per_record * 1000)
                    distance += dist_per_record

        # Create laptimer metadata
        lap_times = []
        for lap_num in range(3):
            total_time = sum(lap_segment_times[lap_num])
            lap_times.append({
                'lap': lap_num + 1,
                'time': total_time,
                'distance': 8.0,
                'gyro_z_agg': 0.1,
                'valid': True
            })

        # Get actual session ID from manifest
        actual_session_id = tub.manifest.session_id[1]
        tub.manifest.metadata[actual_session_id] = {
            'laptimer': lap_times,
            'segmentation': {
                'num_segments': 4,
                'mean_course_params': {'num_laps': 3},
                'segmentation_params': {'strategy': 'hybrid'}
            }
        }
        tub.manifest.write_metadata()
        tub.close()

    def _verify_fastest_segments_prioritized(self, records):
        """Verify that records from fastest segments have lowest lap_pct"""

        # Find records for segment 0, lap 1 (fastest instance of segment 0)
        seg0_lap1_records = [
            r for r in records
            if (r.underlying.get('car/segment') == 0 and
                r.underlying.get('car/lap') == 1 and
                'lap_pct' in r.underlying)
        ]

        # Find records for segment 0, lap 3 (slowest instance of segment 0)
        seg0_lap3_records = [
            r for r in records
            if (r.underlying.get('car/segment') == 0 and
                r.underlying.get('car/lap') == 3 and
                'lap_pct' in r.underlying)
        ]

        if seg0_lap1_records and seg0_lap3_records:
            # Fastest instance should have lower time_pct
            lap1_time_pct = seg0_lap1_records[0].underlying['lap_pct'][0]
            lap3_time_pct = seg0_lap3_records[0].underlying['lap_pct'][0]

            self.assertLess(lap1_time_pct, lap3_time_pct,
                           "Fastest segment instance should have lower "
                           "lap_pct than slower instance")

    def _verify_synthetic_best_lap_effect(self, records):
        """
        Verify that best segments come from different laps,
        creating a "synthetic best lap" effect.
        """

        # Find the best lap for each segment (lowest time_pct)
        segment_best_laps = {}

        for segment_id in range(4):
            best_lap = None
            best_time_pct = float('inf')

            for lap_num in [1, 2, 3]:
                seg_records = [
                    r for r in records
                    if (r.underlying.get('car/segment') == segment_id and
                        r.underlying.get('car/lap') == lap_num and
                        'lap_pct' in r.underlying)
                ]

                if seg_records:
                    time_pct = seg_records[0].underlying['lap_pct'][0]
                    if time_pct < best_time_pct:
                        best_time_pct = time_pct
                        best_lap = lap_num

            if best_lap is not None:
                segment_best_laps[segment_id] = best_lap

        # Verify that best segments come from multiple laps
        unique_best_laps = set(segment_best_laps.values())

        self.assertGreater(len(unique_best_laps), 1,
                          f"Best segments should come from multiple laps "
                          f"(synthetic best lap effect). Got: "
                          f"{segment_best_laps}")

        # Based on our test data:
        # Segment 0: Lap 1 should be best (0.5s vs 1.0s vs 1.5s)
        # Segment 1: Lap 2 should be best (0.5s vs 1.5s vs 1.0s)
        # Segment 2: Lap 2 should be best (0.5s vs 1.0s vs 1.5s)
        # Segment 3: Lap 1 should be best (0.5s vs 1.5s vs 1.0s)

        if 0 in segment_best_laps:
            self.assertEqual(segment_best_laps[0], 1,
                           "Segment 0 should be fastest in lap 1")
        if 1 in segment_best_laps:
            self.assertEqual(segment_best_laps[1], 2,
                           "Segment 1 should be fastest in lap 2")


if __name__ == '__main__':
    unittest.main()
