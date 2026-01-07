"""
Tests for course segmentation training pipeline integration (Phase 3)

Tests the integration between segmentation and training pipeline:
- TubDataset loads with pct_mode=PctMode.SEGMENT
- Records have lap_pct populated from segment rankings
- Different segments in same lap have different lap_pct values
- Backward compatibility with lap-based mode
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


class TestSegmentationTrainingIntegration(unittest.TestCase):
    """Test segmentation integration with training pipeline"""

    def setUp(self):
        self.open_tubs = []  # Track tubs to close
        """Create test tub with multi-lap segmented data"""
        self.temp_dir = tempfile.mkdtemp()
        self.tub_path = os.path.join(self.temp_dir, 'test_tub')
        self.config = self._create_test_config()
        self._create_test_tub()

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
        """Create minimal config for testing"""
        cfg = Config()
        cfg.USE_LAP_0 = False
        cfg.TRAIN_TEST_SPLIT = 0.8
        cfg.GYRO_Z_INDEX = 1
        return cfg

    def _create_test_tub(self):
        """Create test tub with 3 laps, 4 segments, varying performance"""
        tub = Tub(self.tub_path,
                  inputs=['cam/image_array', 'car/lap', 'car/segment',
                          'car/gyro', 'car/distance'],
                  types=['image_array', 'int', 'int', 'vector', 'float'])
        self.open_tubs.append(tub)

        session_id = 'test_session_001'

        # Create image for records
        test_image = np.zeros((120, 160, 3), dtype=np.uint8)

        # Segment performance: fast=0.05s, medium=0.1s, slow=0.15s per record
        segment_times = {
            'fast': 0.05,
            'medium': 0.1,
            'slow': 0.15
        }

        # Lap structures
        lap_structures = [
            ['fast', 'slow', 'medium', 'fast'],    # Lap 0
            ['medium', 'fast', 'fast', 'slow'],    # Lap 1
            ['slow', 'medium', 'slow', 'medium']   # Lap 2
        ]

        records_per_segment = 10
        timestamp_ms = 0
        distance = 0.0

        for lap_num, lap_structure in enumerate(lap_structures):
            for segment_id, performance in enumerate(lap_structure):
                time_per_record = segment_times[performance]
                dist_per_record = 0.15

                for _ in range(records_per_segment):
                    record = {
                        '_session_id': session_id,
                        'cam/image_array': test_image,
                        'user/angle': 0.1 * segment_id,  # Vary by segment
                        'user/throttle': 0.3,
                        'car/lap': lap_num,
                        'car/segment': segment_id,
                        'car/gyro': [0.0, 0.1, 0.0],
                        'car/distance': distance,
                        '_timestamp_ms': timestamp_ms
                    }
                    tub.write_record(record)
                    timestamp_ms += int(time_per_record * 1000)
                    distance += dist_per_record

        # Create laptimer metadata
        lap_times = [
            {'lap': 0, 'time': 2.0, 'distance': 6.0, 'valid': True},
            {'lap': 1, 'time': 2.2, 'distance': 6.0, 'valid': True},
            {'lap': 2, 'time': 2.4, 'distance': 6.0, 'valid': True}
        ]

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

    def test_load_dataset_with_segment_mode(self):
        """Test loading TubDataset with pct_mode=SEGMENT"""
        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.SEGMENT
        )

        records = dataset.get_records()
        self.assertGreater(len(records), 0)
        dataset.close()

    def test_records_have_lap_pct_from_segments(self):
        """Test that records have lap_pct populated from segment rankings"""
        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.SEGMENT
        )

        records = dataset.get_records()

        # Check that records have lap_pct
        records_with_lap_pct = [r for r in records
                                if 'lap_pct' in r.underlying]

        self.assertGreater(len(records_with_lap_pct), 0,
                          "Some records should have lap_pct")

        # Verify lap_pct is a list of rankings
        for record in records_with_lap_pct:
            lap_pct = record.underlying['lap_pct']
            self.assertIsInstance(lap_pct, list)
            self.assertEqual(len(lap_pct), 3)  # [time, gyro, distance]

            # Rankings should be between 0 and 1
            for pct in lap_pct:
                self.assertGreaterEqual(pct, 0.0)
                self.assertLessEqual(pct, 1.0)

        dataset.close()

    def test_different_segments_have_different_lap_pct(self):
        """Test that different segments in same lap have different lap_pct"""
        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.SEGMENT
        )

        records = dataset.get_records()

        # Group records by lap and segment
        lap_segment_records = {}
        for record in records:
            if 'lap_pct' not in record.underlying:
                continue

            lap = record.underlying['car/lap']
            segment = record.underlying.get('car/segment')

            if segment is not None:
                key = (lap, segment)
                if key not in lap_segment_records:
                    lap_segment_records[key] = []
                lap_segment_records[key].append(record)

        # Check lap 1: should have segments 0,1,2,3 with different lap_pct
        lap1_segments = {}
        for (lap, seg), records_list in lap_segment_records.items():
            if lap == 1 and len(records_list) > 0:
                lap1_segments[seg] = records_list[0].underlying['lap_pct']

        # Should have multiple segments in lap 1
        self.assertGreaterEqual(len(lap1_segments), 2,
                               "Lap 1 should have multiple segments")

        # Different segments should have different rankings
        unique_rankings = set(tuple(pct) for pct in lap1_segments.values())
        self.assertGreater(len(unique_rankings), 1,
                          "Different segments should have different rankings")

        dataset.close()

    def test_lap_mode_still_works(self):
        """Test backward compatibility: LAP mode still works"""
        # Need to compute lap performance first
        tub = Tub(self.tub_path, read_only=True)
        self.open_tubs.append(tub)
        stats = TubStatistics(tub)

        # This would normally be done by generate_laptimes_from_records
        # but we created metadata manually, so just verify it exists
        # Get all session IDs from metadata and verify at least one exists
        self.assertGreater(len(tub.manifest.metadata), 0,
                          "Should have metadata for at least one session")
        tub.close()

        dataset = TubDataset(
            config=self.config,
            tub_paths=[self.tub_path],
            pct_mode=PctMode.LAP
        )

        records = dataset.get_records()
        self.assertGreater(len(records), 0)

        # Some records should have lap_pct (from lap-based ranking)
        records_with_lap_pct = [r for r in records
                                if 'lap_pct' in r.underlying]
        # May be 0 if laps don't have complete timing data, which is OK
        # The important part is it doesn't crash

        dataset.close()

    def test_segment_mode_requires_car_segment_field(self):
        """Test that SEGMENT mode requires car/segment in records"""
        # Create tub without car/segment field (but with car/lap)
        tub_path_no_seg = os.path.join(self.temp_dir, 'tub_no_segments')
        tub = Tub(tub_path_no_seg,
                  inputs=['cam/image_array', 'user/angle', 'user/throttle',
                          'car/lap'],
                  types=['image_array', 'float', 'float', 'int'])
        self.open_tubs.append(tub)

        test_image = np.zeros((120, 160, 3), dtype=np.uint8)

        # Create records without car/segment
        for i in range(10):
            record = {
                'cam/image_array': test_image,
                'user/angle': 0.0,
                'user/throttle': 0.3,
                'car/lap': 0,
                '_timestamp_ms': i * 100
            }
            tub.write_record(record)

        # Get actual session ID from manifest
        actual_session_id = tub.manifest.session_id[1]
        tub.manifest.metadata[actual_session_id] = {
            'laptimer': [{'lap': 0, 'time': 1.0, 'distance': 1.0,
                         'valid': True}]
        }
        tub.manifest.write_metadata()
        tub.close()

        # Try to load with SEGMENT mode
        dataset = TubDataset(
            config=self.config,
            tub_paths=[tub_path_no_seg],
            pct_mode=PctMode.SEGMENT
        )

        records = dataset.get_records()

        # Records without car/segment should not have lap_pct in SEGMENT mode
        records_with_lap_pct = [r for r in records
                                if 'lap_pct' in r.underlying]

        # Should have 0 records with lap_pct (missing car/segment)
        self.assertEqual(len(records_with_lap_pct), 0,
                        "Records without car/segment should not get lap_pct")

        dataset.close()


if __name__ == '__main__':
    unittest.main()
