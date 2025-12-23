"""
Tests for segment estimator (real-time point-based estimation)
"""

import unittest
import numpy as np

from donkeycar.parts.course_analysis import (
    MeanCourse, CourseSegmentation, SegmentEstimator, SegmentEstimate,
    normalize_angle
)


class TestSegmentEstimator(unittest.TestCase):
    """Test real-time segment estimation"""

    def _create_test_course_and_segmentation(self):
        """Create test course and segmentation"""
        theta = np.linspace(0, 2 * np.pi, 200)
        x = 10 * np.cos(theta)
        y = 5 * np.sin(theta)

        dx = np.diff(x)
        dy = np.diff(y)
        heading = np.arctan2(dy, dx)
        heading = np.append(heading, heading[-1])

        ds = np.sqrt(dx**2 + dy**2)
        distance = np.concatenate([[0], np.cumsum(ds)])

        mean_course = MeanCourse()
        mean_course.x = x
        mean_course.y = y
        mean_course.heading = heading
        mean_course.distance = distance
        mean_course.num_laps = 1

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        return mean_course, segmentation

    def test_estimate_on_course(self):
        """Test estimation for position on course"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        x = mean_course.x[0]
        y = mean_course.y[0]
        heading = mean_course.heading[0]
        estimate = estimator.estimate(x, y, heading)

        self.assertIsInstance(estimate, SegmentEstimate)
        self.assertIsNotNone(estimate.segment_id)
        self.assertGreater(estimate.confidence, 0.5)
        self.assertLess(estimate.distance_to_course, 0.5)

    def test_estimate_near_course(self):
        """Test estimation for position near course"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

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

        x = mean_course.x[0] + 100
        y = mean_course.y[0] + 100
        heading = 0

        estimate = estimator.estimate(x, y, heading)

        self.assertIsNone(estimate.segment_id)
        self.assertLess(estimate.confidence, 0.5)
        self.assertGreater(estimate.distance_to_course, 10.0)

    def test_estimate_with_heading(self):
        """Test that heading helps disambiguation"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        x = mean_course.x[100]
        y = mean_course.y[100]
        correct_heading = mean_course.heading[100]
        wrong_heading = normalize_angle(correct_heading + np.pi)

        estimate1 = estimator.estimate(x, y, correct_heading)
        estimate2 = estimator.estimate(x, y, wrong_heading)

        self.assertGreater(estimate1.confidence, estimate2.confidence)

    def test_estimate_batch(self):
        """Test batch estimation"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        indices = [0, 50, 100, 150]
        positions = np.array([
            [mean_course.x[i], mean_course.y[i], mean_course.heading[i]]
            for i in indices
        ])

        estimates = estimator.estimate_batch(positions)

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

        seg1 = segmentation.segments[0]
        boundary_idx = seg1.end_index

        x = mean_course.x[boundary_idx]
        y = mean_course.y[boundary_idx]
        heading = mean_course.heading[boundary_idx]

        estimate = estimator.estimate(x, y, heading)

        self.assertIsNotNone(estimate.segment_id)
        self.assertIn(estimate.segment_id, [0, 1])
