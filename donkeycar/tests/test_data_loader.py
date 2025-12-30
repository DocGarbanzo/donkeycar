"""
Tests for data loading infrastructure (Phase 1)

Tests all components:
- PathData: Immutable container for path data
- CSVPathDataSource: Load from CSV files
- TubPathDataSource: Load from Tub directories

Following TDD: Tests written FIRST, should FAIL initially.
"""

import os
import tempfile
import unittest
import numpy as np
import pandas as pd

from donkeycar.course_analysis import (
    PathData, PathDataSource, CSVPathDataSource, TubPathDataSource
)


class TestPathData(unittest.TestCase):
    """Test PathData immutable container"""

    def test_create_path_data(self):
        """Test creating PathData from numpy arrays"""
        timestamp = np.array([0.0, 0.1, 0.2, 0.3])
        x = np.array([0.0, 1.0, 2.0, 3.0])
        y = np.array([0.0, 0.5, 1.0, 1.5])
        heading = np.array([0.0, 0.1, 0.2, 0.3])
        velocity = np.array([1.0, 1.0, 1.0, 1.0])

        path_data = PathData(timestamp, x, y, heading, velocity)
        self.assertEqual(len(path_data), 4)
        np.testing.assert_array_equal(path_data.x, x)

    def test_path_data_immutability(self):
        """Test that PathData arrays cannot be modified"""
        timestamp = np.array([0.0, 0.1, 0.2])
        x = np.array([0.0, 1.0, 2.0])
        y = np.array([0.0, 0.5, 1.0])
        heading = np.array([0.0, 0.1, 0.2])
        velocity = np.array([1.0, 1.0, 1.0])

        path_data = PathData(timestamp, x, y, heading, velocity)

        # Modification should raise error
        with self.assertRaises((ValueError, AttributeError)):
            path_data.x[0] = 999.0

    def test_path_data_properties(self):
        """Test PathData computed properties"""
        timestamp = np.array([0.0, 0.1, 0.2, 0.3])
        x = np.array([0.0, 1.0, 2.0, 3.0])
        y = np.array([0.0, 1.0, 2.0, 3.0])
        heading = np.array([0.0, 0.1, 0.2, 0.3])
        velocity = np.array([1.0, 1.5, 2.0, 2.5])

        path_data = PathData(timestamp, x, y, heading, velocity)

        self.assertAlmostEqual(path_data.total_distance,
                              np.sqrt(2) * 3, places=2)
        self.assertAlmostEqual(path_data.duration, 0.3)
        self.assertAlmostEqual(path_data.mean_velocity, 1.75)


class TestCSVPathDataSource(unittest.TestCase):
    """Test CSV file loading"""

    def setUp(self):
        """Create synthetic CSV file for testing"""
        self.temp_dir = tempfile.mkdtemp()
        self.csv_path = os.path.join(self.temp_dir, 'test_path.csv')

        # Create synthetic oval track (2 laps)
        t = np.linspace(0, 4*np.pi, 200)
        x = 10 * np.cos(t)
        y = 5 * np.sin(t - np.pi/2)  # Start below y=0
        h = np.arctan2(np.diff(y, append=y[-1]), np.diff(x, append=x[-1]))
        v = np.ones_like(t) * 2.0  # 2 m/s constant

        # Write CSV
        df = pd.DataFrame({'t': t, 'x': x, 'y': y, 'h': h, 'v': v})
        df.to_csv(self.csv_path, index=False)

    def tearDown(self):
        """Clean up temp files"""
        if os.path.exists(self.csv_path):
            os.remove(self.csv_path)
        os.rmdir(self.temp_dir)

    def test_load_csv(self):
        """Test loading CSV file into PathData"""
        source = CSVPathDataSource(self.csv_path)
        path_data = source.load()

        self.assertEqual(len(path_data), 200)
        self.assertEqual(path_data.x.shape, (200,))
        self.assertAlmostEqual(path_data.x[0], 10.0, places=1)

    def test_csv_missing_columns(self):
        """Test error handling for missing columns"""
        bad_csv = os.path.join(self.temp_dir, 'bad.csv')
        df = pd.DataFrame({'t': [0, 1], 'x': [0, 1]})  # Missing y,h,v
        df.to_csv(bad_csv, index=False)

        source = CSVPathDataSource(bad_csv)
        with self.assertRaises(ValueError):
            source.load()

        os.remove(bad_csv)

    def test_csv_file_not_found(self):
        """Test error handling for missing file"""
        source = CSVPathDataSource('/nonexistent/path.csv')
        with self.assertRaises(FileNotFoundError):
            source.load()


class TestTubPathDataSource(unittest.TestCase):
    """Test Tub directory loading"""

    def test_load_tub(self):
        """Test loading from Tub directory"""
        # This requires actual Tub test data
        # Skip for now, will implement when Tub format is clearer
        self.skipTest("Tub loader not yet implemented")


if __name__ == '__main__':
    unittest.main()
