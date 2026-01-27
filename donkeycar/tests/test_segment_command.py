"""
Test for donkey segment command - actually runs compute_segment_assignments()

This test exercises the REAL segmentation workflow to catch bugs like:
- JSON serialization of numpy types
- Manifest metadata storage
- On-the-fly segment ID computation
"""

import os
import tempfile
import unittest
import shutil
import json
import numpy as np

from donkeycar.config import Config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics, FieldAggregationSpec


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


class TestSegmentCommand(unittest.TestCase):
    """Test actual segment command execution"""

    def setUp(self):
        """Create test environment"""
        self.temp_dir = tempfile.mkdtemp()
        self.tub_path = os.path.join(self.temp_dir, 'test_tub')

    def tearDown(self):
        """Clean up temp files"""
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def _create_oval_track_tub(self, num_laps=3, points_per_lap=100):
        """
        Create tub with realistic oval track data.

        This creates actual position data (car/pos) that the segmentation
        algorithm can process, unlike fake hardcoded segments.
        """
        tub = Tub(
            self.tub_path,
            inputs=['cam/image_array', 'user/angle', 'user/throttle',
                    'car/lap', 'car/pos', 'car/euler', 'car/speed',
                    'car/gyro', 'car/distance'],
            types=['image_array', 'float', 'float', 'int', 'vector',
                   'vector', 'float', 'vector', 'float']
        )

        test_image = np.zeros((120, 160, 3), dtype=np.uint8)
        timestamp_ms = 0
        distance = 0.0

        # Create oval track: two straights + two curves
        # Track is ~10m long (2x 3m straights + 2x 2m curves)
        for lap in range(num_laps):
            for i in range(points_per_lap):
                t = i / points_per_lap  # 0 to 1 around the track

                # Parametric oval: x = 3*cos(2*pi*t), y = 1.5*sin(2*pi*t)
                angle = 2 * np.pi * t
                x = 3.0 * np.cos(angle)
                y = 1.5 * np.sin(angle)

                # Heading tangent to oval
                heading = np.degrees(np.arctan2(
                    1.5 * np.cos(angle),
                    -3.0 * np.sin(angle)
                ))

                record = {
                    'cam/image_array': test_image,
                    'user/angle': 0.0,
                    'user/throttle': 0.5,
                    'car/lap': lap + 1,
                    'car/pos': [x, y, 0.0],
                    'car/euler': [0.0, 0.0, heading],
                    'car/speed': 1.0,
                    'car/gyro': [0.0, 0.1, 0.0],
                    'car/distance': distance,
                    '_timestamp_ms': timestamp_ms
                }
                tub.write_record(record)

                timestamp_ms += 100  # 100ms per point = 10Hz
                distance += 0.1  # ~10m per lap

        # Add laptimer metadata (required for lap detection)
        session_id = tub.manifest.session_id[1]
        tub.manifest.metadata[session_id] = {
            'laptimer': [
                {'lap': i + 1, 'time': 10.0, 'distance': 10.0,
                 'gyro_z_agg': 0.1, 'valid': True}
                for i in range(num_laps)
            ]
        }
        tub.manifest.write_metadata()
        tub.close()

        return session_id

    def test_compute_segment_assignments_json_serializable(self):
        """
        Test that compute_segment_assignments produces JSON-serializable output.

        This catches the numpy int64/float64 serialization bug.
        """
        session_id = self._create_oval_track_tub(num_laps=3)

        # Run actual segmentation
        tub = Tub(self.tub_path, read_only=False)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1)
        stats.compute_segment_assignments(
            lap_detector='ycrossing',
            segmentation_strategy='hybrid'
        )
        tub.close()

        # Verify manifest can be read and re-serialized (catches numpy types)
        tub = Tub(self.tub_path, read_only=True)
        session_data = tub.manifest.metadata.get(session_id, {})
        seg_data = session_data.get('segmentation')

        self.assertIsNotNone(seg_data, "Segmentation data should exist")
        self.assertGreater(seg_data['num_segments'], 0,
                          "Should have at least one segment")

        # This will raise TypeError if numpy types weren't converted
        try:
            json.dumps(seg_data)
        except TypeError as e:
            self.fail(f"Segmentation data not JSON serializable: {e}")

        tub.close()

    def test_segmentation_metadata_structure(self):
        """Test that segmentation metadata has all required fields."""
        session_id = self._create_oval_track_tub(num_laps=3)

        tub = Tub(self.tub_path, read_only=False)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1)
        stats.compute_segment_assignments()
        tub.close()

        tub = Tub(self.tub_path, read_only=True)
        session_data = tub.manifest.metadata.get(session_id, {})
        seg_data = session_data.get('segmentation')

        # Verify structure
        self.assertIn('num_segments', seg_data)
        self.assertIn('mean_course', seg_data)
        self.assertIn('segments', seg_data)
        self.assertIn('segment_boundaries', seg_data)

        # Verify mean_course arrays
        mc = seg_data['mean_course']
        self.assertIn('x', mc)
        self.assertIn('y', mc)
        self.assertIn('heading', mc)
        self.assertIn('distance', mc)
        self.assertIsInstance(mc['x'], list, "mean_course.x should be list")

        # Verify segments structure
        for seg in seg_data['segments']:
            self.assertIn('segment_id', seg)
            self.assertIn('start_index', seg)
            self.assertIn('end_index', seg)
            self.assertIsInstance(seg['segment_id'], int)

        # Verify boundaries structure
        for b in seg_data['segment_boundaries']:
            self.assertIn('point', b)
            self.assertIn('tangent', b)
            self.assertIn('segment_from', b)
            self.assertIn('segment_to', b)
            self.assertIsInstance(b['segment_from'], int)

        tub.close()

    def test_on_the_fly_segment_computation(self):
        """
        Test that segment IDs can be computed on-the-fly from metadata.

        This verifies the full workflow: store metadata, reconstruct assigner,
        compute segment IDs.
        """
        from donkeycar.course_analysis import get_or_compute_segment_id

        session_id = self._create_oval_track_tub(num_laps=2)

        # Run segmentation
        tub = Tub(self.tub_path, read_only=False)
        stats = TubStatistics(tub, field_aggregations=GYRO_Z_INDEX_1)
        stats.compute_segment_assignments()
        tub.close()

        # Compute segment IDs on-the-fly (like training pipeline does)
        tub = Tub(self.tub_path, read_only=True)
        assigners = {}
        prev_segments = {}

        computed_segments = []
        for record in tub:
            pos = record.get('car/pos')
            session = record.get('_session_id')

            segment_id = get_or_compute_segment_id(
                session, pos, tub.manifest.metadata,
                assigners, prev_segments
            )
            computed_segments.append(segment_id)

        tub.close()

        # Should have computed segments for all records
        self.assertEqual(len(computed_segments), 200)  # 2 laps * 100 points

        # Should have computed at least some valid segment IDs
        valid_segments = [s for s in computed_segments if s is not None]
        self.assertGreater(len(valid_segments), 0,
                          "Should compute valid segment IDs")

        # Note: Full segment transition detection is complex and depends on
        # boundary crossing logic. The key test here is that we CAN compute
        # segment IDs from reconstructed metadata.


if __name__ == '__main__':
    unittest.main()
