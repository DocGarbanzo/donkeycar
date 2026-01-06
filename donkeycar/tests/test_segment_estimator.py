"""
Tests for segment estimator (real-time point-based estimation)
"""

import unittest
import numpy as np

from donkeycar.course_analysis import (
    MeanCourse, CourseSegmenter, GradientSegmentation, SegmentEstimator, 
    SegmentEstimate, normalize_angle
)
from donkeycar.tests.course_test_fixtures import create_mean_course_from_arrays


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

        # Use new MeanCourse constructor with required parameters
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        # Create segmentation using CourseSegmenter
        segmenter = CourseSegmenter(GradientSegmentation())
        segmentation = segmenter.segment(mean_course)

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
        self.assertLess(estimate.cross_track_error, 0.5)

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
        self.assertLess(estimate.cross_track_error, 2.0)

    def test_estimate_off_course(self):
        """Test estimation for position far off course"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        x = mean_course.x[0] + 100
        y = mean_course.y[0] + 100
        heading = 0

        estimate = estimator.estimate(x, y, heading)

        # New API always returns a segment (nearest neighbor)
        self.assertIsNotNone(estimate.segment_id)
        self.assertLess(estimate.confidence, 0.5)
        self.assertGreater(estimate.cross_track_error, 10.0)

    def test_estimate_with_heading(self):
        """Test that heading is captured in estimate"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        x = mean_course.x[100]
        y = mean_course.y[100]
        correct_heading = mean_course.heading[100]
        wrong_heading = normalize_angle(correct_heading + np.pi)

        estimate1 = estimator.estimate(x, y, correct_heading)
        estimate2 = estimator.estimate(x, y, wrong_heading)

        # Heading error should be different
        self.assertLess(estimate1.heading_error, estimate2.heading_error)

    def test_estimate_multiple_positions(self):
        """Test estimating multiple positions (replacing batch test)"""
        mean_course, segmentation = self._create_test_course_and_segmentation()

        estimator = SegmentEstimator(segmentation)

        indices = [0, 50, 100, 150]
        
        for i in indices:
            x = mean_course.x[i]
            y = mean_course.y[i]
            heading = mean_course.heading[i]
            
            estimate = estimator.estimate(x, y, heading)

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
