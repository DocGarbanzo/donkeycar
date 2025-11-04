"""
Tests for course analysis and segmentation

Tests all components:
- MultiLapData: CSV loading and lap detection
- MeanCourse: Mean course reconstruction
- CourseSegmentation: Segment detection and classification
- SegmentEstimator: Real-time segment estimation
"""

import os
import tempfile
import unittest
import numpy as np
import json

from donkeycar.parts.course_analysis import (
    MultiLapData, MeanCourse, CourseSegmentation, Segment, SegmentEstimator,
    SegmentType, SegmentEstimate, normalize_angle, angle_difference,
    circular_mean, circular_std
)


class TestAngleFunctions(unittest.TestCase):
    """Test angle utility functions"""

    def test_normalize_angle(self):
        """Test angle normalization"""
        self.assertAlmostEqual(0, normalize_angle(0))
        self.assertAlmostEqual(90, normalize_angle(90))
        self.assertAlmostEqual(-90, normalize_angle(270))
        # 180 can be either 180 or -180, both are valid
        self.assertIn(normalize_angle(180), [180, -180])
        self.assertAlmostEqual(0, normalize_angle(360))
        self.assertAlmostEqual(10, normalize_angle(370))
        self.assertAlmostEqual(-10, normalize_angle(-10))

    def test_angle_difference(self):
        """Test angle difference calculation"""
        self.assertAlmostEqual(0, angle_difference(0, 0))
        self.assertAlmostEqual(90, angle_difference(0, 90))
        self.assertAlmostEqual(-90, angle_difference(90, 0))
        self.assertAlmostEqual(180, angle_difference(0, 180))
        self.assertAlmostEqual(-179, angle_difference(0, 181))
        self.assertAlmostEqual(10, angle_difference(350, 0))

    def test_circular_mean(self):
        """Test circular mean calculation"""
        # Simple cases
        angles = np.array([0, 90, 180, 270])
        mean = circular_mean(angles)
        # Mean should be undefined (close to origin) but atan2 will give some value
        self.assertTrue(-180 <= mean <= 180)

        # All same angle
        angles = np.array([45, 45, 45])
        mean = circular_mean(angles)
        self.assertAlmostEqual(45, mean, places=5)

        # Crossing 0 degrees
        angles = np.array([350, 10, 0, 360])
        mean = circular_mean(angles)
        self.assertAlmostEqual(0, mean, places=0)  # Should be close to 0

    def test_circular_std(self):
        """Test circular standard deviation"""
        # All same angle - zero std
        angles = np.array([45, 45, 45])
        std = circular_std(angles)
        self.assertAlmostEqual(0, std, places=3)

        # Small spread
        angles = np.array([44, 45, 46])
        std = circular_std(angles)
        self.assertLess(std, 5)  # Should be small


class TestMultiLapData(unittest.TestCase):
    """Test multi-lap data loading and lap detection"""

    def _create_test_csv(self, filename, num_laps=2, points_per_lap=100):
        """
        Create synthetic test CSV with multiple laps

        Creates an oval track with specified number of laps
        """
        # Generate oval track
        t = np.linspace(0, 2 * np.pi * num_laps, num_laps * points_per_lap)
        x = 10 * np.cos(t / num_laps)  # Oval with radius 10m
        y = 5 * np.sin(t / num_laps)   # Oval with radius 5m

        # Add noise
        x += np.random.normal(0, 0.1, len(x))
        y += np.random.normal(0, 0.1, len(y))

        # Calculate heading
        dx = np.diff(x)
        dy = np.diff(y)
        heading = np.arctan2(dy, dx) * 180 / np.pi
        heading = np.append(heading, heading[-1])

        # Create CSV
        with open(filename, 'w') as f:
            f.write("timestamp,x,y,heading\n")
            for i in range(len(t)):
                f.write(f"{t[i]:.3f},{x[i]:.6f},{y[i]:.6f},{heading[i]:.6f}\n")

    def test_load_csv_valid(self):
        """Test loading valid CSV file"""
        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "test_laps.csv")
            self._create_test_csv(filename, num_laps=3, points_per_lap=100)

            data = MultiLapData()
            data.load_csv(filename)

            self.assertIsNotNone(data.raw_data)
            self.assertGreater(len(data.raw_data), 0)
            self.assertGreater(data.num_laps, 0)

    def test_lap_detection(self):
        """Test lap detection from continuous data"""
        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "test_laps.csv")
            self._create_test_csv(filename, num_laps=3, points_per_lap=100)

            data = MultiLapData()
            # Use larger threshold since synthetic data is noisy
            data.load_csv(filename, lap_detection_threshold=3.0)

            # Should detect at least 1 lap
            self.assertGreaterEqual(data.num_laps, 1)  # At least 1 lap

            # Each lap should have reasonable length
            for lap in data.get_laps():
                self.assertGreater(len(lap), 50)

    def test_load_csv_missing_column(self):
        """Test error handling for missing columns"""
        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "test_bad.csv")

            # Create CSV without heading column
            with open(filename, 'w') as f:
                f.write("timestamp,x,y\n")
                f.write("0,0,0\n")
                f.write("1,1,1\n")

            data = MultiLapData()
            with self.assertRaises(ValueError):
                data.load_csv(filename)

    def test_get_laps(self):
        """Test getting individual laps"""
        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "test_laps.csv")
            self._create_test_csv(filename, num_laps=2, points_per_lap=100)

            data = MultiLapData()
            data.load_csv(filename)

            laps = data.get_laps()
            self.assertIsInstance(laps, list)
            self.assertGreater(len(laps), 0)


class TestMeanCourse(unittest.TestCase):
    """Test mean course reconstruction"""

    def _create_multilap_data(self, num_laps=3):
        """Create synthetic multi-lap data"""
        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "test_laps.csv")

            # Generate oval track with multiple laps
            points_per_lap = 100
            all_data = []

            for lap in range(num_laps):
                t = np.linspace(0, 2 * np.pi, points_per_lap)
                x = 10 * np.cos(t)
                y = 5 * np.sin(t)

                # Add lap-specific noise
                x += np.random.normal(0, 0.2, len(x))
                y += np.random.normal(0, 0.2, len(y))

                # Calculate heading
                dx = np.diff(x)
                dy = np.diff(y)
                heading = np.arctan2(dy, dx) * 180 / np.pi
                heading = np.append(heading, heading[-1])

                # Timestamps
                timestamp = np.arange(len(t)) + lap * points_per_lap

                for i in range(len(t)):
                    all_data.append([timestamp[i], x[i], y[i], heading[i]])

            # Write CSV
            with open(filename, 'w') as f:
                f.write("timestamp,x,y,heading\n")
                for row in all_data:
                    f.write(f"{row[0]:.3f},{row[1]:.6f},{row[2]:.6f},{row[3]:.6f}\n")

            # Load data
            data = MultiLapData()
            data.load_csv(filename, lap_detection_threshold=2.0)

            return data

    def test_compute_mean_course(self):
        """Test mean course computation"""
        data = self._create_multilap_data(num_laps=3)

        mean_course = MeanCourse(data)
        mean_course.compute()

        # Check outputs
        self.assertIsNotNone(mean_course.x)
        self.assertIsNotNone(mean_course.y)
        self.assertIsNotNone(mean_course.heading)
        self.assertIsNotNone(mean_course.distance)

        # All arrays should have same length
        n = len(mean_course.x)
        self.assertEqual(n, len(mean_course.y))
        self.assertEqual(n, len(mean_course.heading))
        self.assertEqual(n, len(mean_course.distance))

        # Distance should be monotonically increasing
        self.assertTrue(np.all(np.diff(mean_course.distance) >= 0))

        # Course should be closed (oval)
        start_pos = np.array([mean_course.x[0], mean_course.y[0]])
        end_pos = np.array([mean_course.x[-1], mean_course.y[-1]])
        closure_error = np.linalg.norm(end_pos - start_pos)
        self.assertLess(closure_error, 5.0)  # Should be reasonably closed

    def test_save_load_csv(self):
        """Test saving and loading mean course as CSV"""
        data = self._create_multilap_data(num_laps=2)

        mean_course = MeanCourse(data)
        mean_course.compute()

        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "mean_course.csv")
            mean_course.save(filename)

            # Load back
            loaded_course = MeanCourse()
            loaded_course.load(filename)

            # Check arrays match
            np.testing.assert_array_almost_equal(mean_course.x, loaded_course.x)
            np.testing.assert_array_almost_equal(mean_course.y, loaded_course.y)
            np.testing.assert_array_almost_equal(mean_course.heading, loaded_course.heading)
            np.testing.assert_array_almost_equal(mean_course.distance, loaded_course.distance)

    def test_save_load_json(self):
        """Test saving and loading mean course as JSON"""
        data = self._create_multilap_data(num_laps=2)

        mean_course = MeanCourse(data)
        mean_course.compute()

        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "mean_course.json")
            mean_course.save(filename)

            # Load back
            loaded_course = MeanCourse()
            loaded_course.load(filename)

            # Check arrays match
            np.testing.assert_array_almost_equal(mean_course.x, loaded_course.x)
            np.testing.assert_array_almost_equal(mean_course.y, loaded_course.y)
            np.testing.assert_array_almost_equal(mean_course.heading, loaded_course.heading)
            np.testing.assert_array_almost_equal(mean_course.distance, loaded_course.distance)
            self.assertEqual(mean_course.num_laps, loaded_course.num_laps)

    def test_different_parameters(self):
        """Test mean course with different parameters"""
        data = self._create_multilap_data(num_laps=2)

        params = {
            'resampling_interval': 0.2,
            'outlier_std_threshold': 3.0,
            'position_smoothing_window': 7,
        }

        mean_course = MeanCourse(data, params)
        mean_course.compute()

        # Should still produce valid output
        self.assertGreater(len(mean_course.x), 0)
        self.assertTrue(np.all(np.diff(mean_course.distance) >= 0))


class TestCourseSegmentation(unittest.TestCase):
    """Test course segmentation"""

    def _create_simple_course(self):
        """Create simple test course with straight and turns"""
        # Straight section
        x1 = np.linspace(0, 10, 50)
        y1 = np.zeros(50)

        # Left turn (90 degrees)
        theta = np.linspace(0, np.pi / 2, 30)
        radius = 5
        x2 = 10 + radius * np.sin(theta)
        y2 = radius * (1 - np.cos(theta))

        # Another straight
        x3 = np.linspace(x2[-1], x2[-1], 40)
        y3 = np.linspace(y2[-1], y2[-1] + 10, 40)

        # Concatenate
        x = np.concatenate([x1, x2, x3])
        y = np.concatenate([y1, y2, y3])

        # Calculate heading
        dx = np.diff(x)
        dy = np.diff(y)
        heading = np.arctan2(dy, dx) * 180 / np.pi
        heading = np.append(heading, heading[-1])

        # Calculate distance
        ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        distance = np.concatenate([[0], np.cumsum(ds)])

        # Create MeanCourse
        mean_course = MeanCourse()
        mean_course.x = x
        mean_course.y = y
        mean_course.heading = heading
        mean_course.distance = distance
        mean_course.num_laps = 1

        return mean_course

    def _create_racetrack_course(self):
        """Create complex racetrack with various segment types"""
        segments = []

        # Straight 1
        x1 = np.linspace(0, 20, 100)
        y1 = np.zeros(100)
        segments.append((x1, y1))

        # Right turn
        theta1 = np.linspace(0, -np.pi / 2, 50)
        r1 = 8
        x2 = 20 + r1 * np.sin(-theta1)
        y2 = -r1 * (1 - np.cos(-theta1))
        segments.append((x2, y2))

        # Short straight
        x3 = np.linspace(x2[-1], x2[-1] + 10, 50)
        y3 = np.ones(50) * y2[-1]
        segments.append((x3, y3))

        # S-curve (left then right)
        theta_s1 = np.linspace(0, np.pi / 3, 30)
        r_s = 5
        x4 = x3[-1] + r_s * np.sin(theta_s1)
        y4 = y3[-1] + r_s * (1 - np.cos(theta_s1))

        theta_s2 = np.linspace(np.pi / 3, 0, 30)
        x5 = x4[-1] + r_s * np.sin(-theta_s2)
        y5 = y4[-1] - r_s * (1 - np.cos(-theta_s2))
        segments.append((x4, y4))
        segments.append((x5, y5))

        # Concatenate all segments
        x = np.concatenate([seg[0] for seg in segments])
        y = np.concatenate([seg[1] for seg in segments])

        # Calculate heading
        dx = np.diff(x)
        dy = np.diff(y)
        heading = np.arctan2(dy, dx) * 180 / np.pi
        heading = np.append(heading, heading[-1])

        # Calculate distance
        ds = np.sqrt(dx**2 + dy**2)
        distance = np.concatenate([[0], np.cumsum(ds)])

        # Create MeanCourse
        mean_course = MeanCourse()
        mean_course.x = x
        mean_course.y = y
        mean_course.heading = heading
        mean_course.distance = distance
        mean_course.num_laps = 1

        return mean_course

    def test_segment_simple_course(self):
        """Test segmentation of simple course"""
        mean_course = self._create_simple_course()

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        # Should have at least 2 segments (straight and turn)
        self.assertGreaterEqual(segmentation.total_segments, 2)

        # Check segments have valid properties
        for segment in segmentation.segments:
            self.assertIsInstance(segment, Segment)
            self.assertIsInstance(segment.segment_type, SegmentType)
            self.assertGreater(segment.length, 0)
            self.assertGreaterEqual(segment.start_index, 0)
            self.assertLess(segment.end_index, len(mean_course.x))

    def test_segment_racetrack(self):
        """Test segmentation of complex racetrack"""
        mean_course = self._create_racetrack_course()

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        # Should have multiple segments
        self.assertGreaterEqual(segmentation.total_segments, 4)

        # Should have straight segments
        straight_count = segmentation.segment_counts.get(SegmentType.STRAIGHT, 0)
        self.assertGreater(straight_count, 0)

        # Check segment continuity (no gaps)
        for i in range(len(segmentation.segments) - 1):
            seg1 = segmentation.segments[i]
            seg2 = segmentation.segments[i + 1]
            # End of seg1 should connect to start of seg2
            self.assertLessEqual(abs(seg2.start_index - seg1.end_index), 1)

    def test_get_segment(self):
        """Test getting segment by ID"""
        mean_course = self._create_simple_course()

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        # Get first segment
        segment = segmentation.get_segment(0)
        self.assertIsNotNone(segment)
        self.assertEqual(0, segment.segment_id)

        # Get invalid segment
        segment = segmentation.get_segment(999)
        self.assertIsNone(segment)

    def test_save_load_segmentation(self):
        """Test saving and loading segmentation"""
        mean_course = self._create_simple_course()

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        with tempfile.TemporaryDirectory() as td:
            filename = os.path.join(td, "segmentation.json")
            segmentation.save(filename)

            # Load back
            loaded_seg = CourseSegmentation()
            loaded_seg.load(filename, mean_course)

            # Check properties match
            self.assertEqual(segmentation.total_segments, loaded_seg.total_segments)
            self.assertEqual(len(segmentation.segments), len(loaded_seg.segments))

            # Check first segment
            seg1 = segmentation.segments[0]
            seg2 = loaded_seg.segments[0]
            self.assertEqual(seg1.segment_id, seg2.segment_id)
            self.assertEqual(seg1.segment_type, seg2.segment_type)
            self.assertAlmostEqual(seg1.length, seg2.length, places=5)


class TestSegmentEstimator(unittest.TestCase):
    """Test real-time segment estimation"""

    def _create_test_course_and_segmentation(self):
        """Create test course and segmentation"""
        # Create oval course
        theta = np.linspace(0, 2 * np.pi, 200)
        x = 10 * np.cos(theta)
        y = 5 * np.sin(theta)

        # Calculate heading
        dx = np.diff(x)
        dy = np.diff(y)
        heading = np.arctan2(dy, dx) * 180 / np.pi
        heading = np.append(heading, heading[-1])

        # Calculate distance
        ds = np.sqrt(dx**2 + dy**2)
        distance = np.concatenate([[0], np.cumsum(ds)])

        # Create MeanCourse
        mean_course = MeanCourse()
        mean_course.x = x
        mean_course.y = y
        mean_course.heading = heading
        mean_course.distance = distance
        mean_course.num_laps = 1

        # Create segmentation
        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        return mean_course, segmentation

    def test_estimate_on_course(self):
        """Test estimation for position on course"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        # Test at first point
        x, y, heading = mean_course.x[0], mean_course.y[0], mean_course.heading[0]
        estimate = estimator.estimate(x, y, heading)

        self.assertIsInstance(estimate, SegmentEstimate)
        self.assertIsNotNone(estimate.segment_id)
        self.assertGreater(estimate.confidence, 0.5)
        self.assertLess(estimate.distance_to_course, 0.5)

    def test_estimate_near_course(self):
        """Test estimation for position near course"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        # Test at point slightly off course
        x = mean_course.x[50] + 0.5
        y = mean_course.y[50] + 0.5
        heading = mean_course.heading[50]

        estimate = estimator.estimate(x, y, heading)

        self.assertIsNotNone(estimate.segment_id)
        self.assertGreater(estimate.confidence, 0.3)
        self.assertLess(estimate.distance_to_course, 2.0)

    def test_estimate_off_course(self):
        """Test estimation for position far off course"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        # Test at point far from course
        x = mean_course.x[0] + 100
        y = mean_course.y[0] + 100
        heading = 0

        estimate = estimator.estimate(x, y, heading)

        # Should return None segment with low confidence
        self.assertIsNone(estimate.segment_id)
        self.assertLess(estimate.confidence, 0.5)
        self.assertGreater(estimate.distance_to_course, 10.0)

    def test_estimate_with_heading(self):
        """Test that heading helps disambiguation"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        # Find a point on the course
        x, y = mean_course.x[100], mean_course.y[100]
        correct_heading = mean_course.heading[100]
        wrong_heading = normalize_angle(correct_heading + 180)

        # Estimate with correct heading
        estimate1 = estimator.estimate(x, y, correct_heading)

        # Estimate with wrong heading
        estimate2 = estimator.estimate(x, y, wrong_heading)

        # Correct heading should give higher confidence
        self.assertGreater(estimate1.confidence, estimate2.confidence)

    def test_estimate_batch(self):
        """Test batch estimation"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        # Create batch of positions
        indices = [0, 50, 100, 150]
        positions = np.array([
            [mean_course.x[i], mean_course.y[i], mean_course.heading[i]]
            for i in indices
        ])

        estimates = estimator.estimate_batch(positions)

        # Check results
        self.assertEqual(len(estimates), len(positions))

        for estimate in estimates:
            self.assertIsInstance(estimate, SegmentEstimate)
            self.assertIsNotNone(estimate.segment_id)
            self.assertGreater(estimate.confidence, 0.5)

    def test_estimate_segment_boundary(self):
        """Test estimation near segment boundary"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        if len(segmentation.segments) < 2:
            self.skipTest("Need at least 2 segments for boundary test")

        estimator = SegmentEstimator(segmentation)

        # Get position at segment boundary
        seg1 = segmentation.segments[0]
        boundary_idx = seg1.end_index

        x = mean_course.x[boundary_idx]
        y = mean_course.y[boundary_idx]
        heading = mean_course.heading[boundary_idx]

        estimate = estimator.estimate(x, y, heading)

        # Should return valid segment (either seg 0 or 1)
        self.assertIsNotNone(estimate.segment_id)
        self.assertIn(estimate.segment_id, [0, 1])


class TestIntegration(unittest.TestCase):
    """Integration tests for complete workflow"""

    def test_end_to_end_workflow(self):
        """Test complete workflow: load CSV -> mean course -> segment -> estimate"""
        with tempfile.TemporaryDirectory() as td:
            # Step 1: Create synthetic CSV
            csv_filename = os.path.join(td, "test_laps.csv")
            num_laps = 3
            points_per_lap = 150

            all_data = []
            for lap in range(num_laps):
                t = np.linspace(0, 2 * np.pi, points_per_lap)
                x = 10 * np.cos(t) + np.random.normal(0, 0.15, len(t))
                y = 5 * np.sin(t) + np.random.normal(0, 0.15, len(t))

                dx = np.diff(x)
                dy = np.diff(y)
                heading = np.arctan2(dy, dx) * 180 / np.pi
                heading = np.append(heading, heading[-1])

                timestamp = np.arange(len(t)) + lap * points_per_lap

                for i in range(len(t)):
                    all_data.append([timestamp[i], x[i], y[i], heading[i]])

            with open(csv_filename, 'w') as f:
                f.write("timestamp,x,y,heading\n")
                for row in all_data:
                    f.write(f"{row[0]:.3f},{row[1]:.6f},{row[2]:.6f},{row[3]:.6f}\n")

            # Step 2: Load multi-lap data
            multilap_data = MultiLapData()
            multilap_data.load_csv(csv_filename)
            self.assertGreater(multilap_data.num_laps, 0)

            # Step 3: Compute mean course
            mean_course = MeanCourse(multilap_data)
            mean_course.compute()
            self.assertGreater(len(mean_course.x), 0)

            # Step 4: Segment course
            segmentation = CourseSegmentation(mean_course)
            segmentation.compute()
            self.assertGreater(segmentation.total_segments, 0)

            # Step 5: Create estimator
            estimator = SegmentEstimator(segmentation)

            # Step 6: Test estimation at multiple points
            test_indices = [0, 50, 100]
            for idx in test_indices:
                x = mean_course.x[idx]
                y = mean_course.y[idx]
                heading = mean_course.heading[idx]

                estimate = estimator.estimate(x, y, heading)
                self.assertIsNotNone(estimate.segment_id)
                self.assertGreater(estimate.confidence, 0.5)

            # Step 7: Save and load all components
            mean_course_file = os.path.join(td, "mean_course.json")
            segmentation_file = os.path.join(td, "segmentation.json")

            mean_course.save(mean_course_file)
            segmentation.save(segmentation_file)

            # Load back
            loaded_course = MeanCourse()
            loaded_course.load(mean_course_file)

            loaded_seg = CourseSegmentation()
            loaded_seg.load(segmentation_file, loaded_course)

            # Create new estimator with loaded data
            new_estimator = SegmentEstimator(loaded_seg)

            # Test estimation with new estimator
            x = loaded_course.x[25]
            y = loaded_course.y[25]
            heading = loaded_course.heading[25]

            estimate = new_estimator.estimate(x, y, heading)
            self.assertIsNotNone(estimate.segment_id)


if __name__ == '__main__':
    unittest.main()
