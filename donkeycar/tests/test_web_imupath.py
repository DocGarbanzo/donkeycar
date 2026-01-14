"""
Tests for web-based IMU path visualizer data preparation.

Tests the IMUPathDataPreparation class which prepares data for the web UI.
"""

import unittest
import numpy as np
from donkeycar.course_analysis import PathData
from donkeycar.web.imupath_data import IMUPathDataPreparation, downsample_points


class TestDownsampling(unittest.TestCase):
    """Test downsampling utility function"""

    def test_downsample_below_max(self):
        """Test that downsampling leaves points unchanged if below max"""
        points = [{'x': i, 'y': i} for i in range(10)]
        result = downsample_points(points, 20)
        self.assertEqual(len(result), 10)
        self.assertEqual(result, points)

    def test_downsample_above_max(self):
        """Test that downsampling reduces points when above max"""
        points = [{'x': i, 'y': i} for i in range(100)]
        result = downsample_points(points, 10)
        self.assertEqual(len(result), 10)

    def test_downsample_preserves_first_last(self):
        """Test that downsampling includes first and last points"""
        points = [{'x': i, 'y': i} for i in range(100)]
        result = downsample_points(points, 10)
        self.assertEqual(result[0], points[0])
        # Last point should be close to original last point
        self.assertGreaterEqual(result[-1]['x'], 90)


class TestIMUPathDataPreparation(unittest.TestCase):
    """Test IMUPathDataPreparation class"""

    def setUp(self):
        """Create test path data"""
        # Create a simple circular path (2 laps)
        num_points = 400
        t = np.linspace(0, 4 * np.pi, num_points)  # 2 full circles
        
        timestamp = np.linspace(0, 40, num_points)
        x = 5 * np.cos(t)
        y = 5 * np.sin(t)
        heading = t + np.pi/2  # Tangent to circle
        velocity = np.ones(num_points) * 2.0
        
        self.path_data = PathData(timestamp, x, y, heading, velocity)

    def test_initialization(self):
        """Test that data preparation initializes correctly"""
        prep = IMUPathDataPreparation(
            path_data=self.path_data,
            lap_method='y_crossing',
            segment_method='gradient'
        )
        
        # Should have processed data
        self.assertIsNotNone(prep.multilap_data)
        self.assertIsNotNone(prep.mean_course)
        self.assertIsNotNone(prep.segmentation)
        self.assertIsNotNone(prep.segment_ids)

    def test_lap_detection(self):
        """Test that laps are detected"""
        prep = IMUPathDataPreparation(
            path_data=self.path_data,
            lap_method='y_crossing',
            segment_method='gradient'
        )
        
        # Should detect approximately 2 laps for circular path
        self.assertGreaterEqual(prep.multilap_data.num_laps, 1)

    def test_segmentation(self):
        """Test that course is segmented"""
        prep = IMUPathDataPreparation(
            path_data=self.path_data,
            lap_method='y_crossing',
            segment_method='gradient'
        )
        
        # Should have at least one segment
        self.assertGreaterEqual(prep.segmentation.num_segments, 1)

    def test_segment_assignment(self):
        """Test that segments are assigned to path"""
        prep = IMUPathDataPreparation(
            path_data=self.path_data,
            lap_method='y_crossing',
            segment_method='gradient'
        )
        
        # Should have segment IDs for all points
        self.assertEqual(len(prep.segment_ids), len(self.path_data.x))
        
        # All segment IDs should be valid
        self.assertTrue(np.all(prep.segment_ids >= 0))
        self.assertTrue(np.all(prep.segment_ids < prep.segmentation.num_segments))

    def test_get_data_payload(self):
        """Test that data payload is generated correctly"""
        prep = IMUPathDataPreparation(
            path_data=self.path_data,
            lap_method='y_crossing',
            segment_method='gradient'
        )
        
        payload = prep.get_data_payload(max_display_points=100)
        
        # Check required keys
        self.assertIn('path_points', payload)
        self.assertIn('mean_course', payload)
        self.assertIn('segments', payload)
        self.assertIn('rankings', payload)
        self.assertIn('metadata', payload)
        
        # Check downsampling
        self.assertLessEqual(len(payload['path_points']), 100)
        
        # Check metadata
        self.assertIn('lap_method', payload['metadata'])
        self.assertIn('segment_method', payload['metadata'])
        self.assertIn('num_laps', payload['metadata'])
        self.assertIn('num_segments', payload['metadata'])

    def test_different_segment_methods(self):
        """Test different segmentation methods"""
        for method in ['threshold', 'extrema', 'gradient', 'hybrid']:
            prep = IMUPathDataPreparation(
                path_data=self.path_data,
                lap_method='y_crossing',
                segment_method=method
            )
            
            self.assertEqual(prep.segment_method, method)
            self.assertGreaterEqual(prep.segmentation.num_segments, 1)

    def test_num_laps_parameter(self):
        """Test that num_laps parameter is respected"""
        prep = IMUPathDataPreparation(
            path_data=self.path_data,
            lap_method='y_crossing',
            segment_method='gradient',
            num_laps=1  # Force single lap
        )
        
        # Mean course should be built from 1 lap
        # (can't directly test, but at least verify it doesn't crash)
        payload = prep.get_data_payload()
        self.assertIsNotNone(payload)


if __name__ == '__main__':
    unittest.main()
