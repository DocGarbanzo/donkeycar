"""
Integration tests for course analysis (Phase 6)

Critical tests validating full workflows as specified in CLAUDE.md:
- Simulate actual user workflows
- Use actual detected boundaries (not guessed values)
- Test correctness, not just validity

These tests catch the bugs that unit tests miss!
"""

import unittest
import numpy as np
import tempfile
import os
import pandas as pd

from donkeycar.parts.course_analysis import (
    PathData, CSVPathDataSource,
    YCrossingLapDetector, DriftLapDetector, MultiLapData,
    MeanCourseBuilder, MeanCourse,
    GradientSegmentation, CourseSegmenter, CourseSegmentation,
    SegmentAssigner
)


def create_synthetic_3lap_oval(points_per_lap=100):
    """Create realistic 3-lap oval for testing"""
    num_laps = 3
    t = np.linspace(0, num_laps * 2 * np.pi, num_laps * points_per_lap)
    x = 10 * np.cos(t)
    y = 5 * np.sin(t - np.pi/2)  # Start below y=0
    h = np.arctan2(np.diff(y, append=y[-1]), np.diff(x, append=x[-1]))
    v = np.ones_like(t) * 2.0

    return PathData(t, x, y, h, v)


class TestFullWorkflow2LapMeanCourse(unittest.TestCase):
    """
    Critical test: 2-lap mean course with 3-lap visualization.

    This is the test that would have caught the "stuck segment" bug!
    Following CLAUDE.md guidelines:
    - Simulate actual user workflow
    - Use actual detected boundaries
    - Test correctness, not just validity
    """

    def test_2_lap_mean_course_with_3_lap_path(self):
        """
        User workflow:
        1. Record 3 laps
        2. Compute mean course from first 2 laps
        3. Segment mean course
        4. Assign segments to all 3 laps
        5. CRITICAL: Verify lap 1 has multiple segments (not stuck!)
        """
        # Step 1: Load 3-lap data
        path_data = create_synthetic_3lap_oval(points_per_lap=100)

        # Step 2: Detect all laps
        detector = YCrossingLapDetector()
        multilap_data = MultiLapData.from_source(
            type('Source', (), {'load': lambda: path_data})(),
            detector
        )

        self.assertGreaterEqual(multilap_data.num_laps, 3,
                               "Should detect 3 laps")

        # Step 3: Build mean from ONLY 2 laps (user selects this in UI)
        # Create limited MultiLapData with only first 2 laps
        limited_boundaries = multilap_data.lap_boundaries[:2]
        limited_data = MultiLapData(path_data, limited_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(limited_data)

        self.assertIsInstance(mean_course, MeanCourse)

        # Step 4: Segment mean course
        segmenter = CourseSegmenter(GradientSegmentation())
        segmentation = segmenter.segment(mean_course)

        self.assertGreater(segmentation.num_segments, 1,
                          "Mean course should have multiple segments")

        # Step 5: Assign to FULL 3-lap path
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        self.assertEqual(len(segment_ids), len(path_data),
                        "Should assign segment to every point")

        # CRITICAL TEST: Verify lap 1 has multiple segments
        # Use ACTUAL detected boundary (not guessed!)
        lap1_end = multilap_data.lap_boundaries[0].end_index

        lap1_segments = set(segment_ids[:lap1_end + 1])

        self.assertGreater(len(lap1_segments), 1,
                          f"BUG: Lap 1 stuck at segment {lap1_segments}! "
                          f"Should traverse multiple segments.")

        # Additional correctness check: segments should progress
        # (not jump randomly)
        segment_transitions = np.diff(segment_ids[:lap1_end + 1])
        num_transitions = np.sum(segment_transitions != 0)

        self.assertGreater(num_transitions, 0,
                          "Lap 1 should have segment transitions")

    def test_drift_detector_with_mean_course(self):
        """Test full workflow with drift detector"""
        path_data = create_synthetic_3lap_oval(points_per_lap=150)

        # Use drift detector
        detector = DriftLapDetector()
        multilap_data = MultiLapData.from_source(
            type('Source', (), {'load': lambda: path_data})(),
            detector
        )

        self.assertGreaterEqual(multilap_data.num_laps, 2,
                               "Drift detector should find laps")

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap_data)

        # Segment
        segmenter = CourseSegmenter(GradientSegmentation())
        segmentation = segmenter.segment(mean_course)

        self.assertGreater(segmentation.num_segments, 1)


class TestCSVLoadingWorkflow(unittest.TestCase):
    """Test loading from actual CSV files"""

    def setUp(self):
        """Create temp CSV with realistic data"""
        self.temp_dir = tempfile.mkdtemp()
        self.csv_path = os.path.join(self.temp_dir, '3_laps.csv')

        path_data = create_synthetic_3lap_oval()

        df = pd.DataFrame({
            't': path_data.timestamp,
            'x': path_data.x,
            'y': path_data.y,
            'h': path_data.heading,
            'v': path_data.velocity
        })
        df.to_csv(self.csv_path, index=False)

    def tearDown(self):
        """Clean up"""
        if os.path.exists(self.csv_path):
            os.remove(self.csv_path)
        os.rmdir(self.temp_dir)

    def test_full_pipeline_from_csv(self):
        """
        Complete pipeline: CSV → laps → mean course → segments → assign
        """
        # Load from CSV
        source = CSVPathDataSource(self.csv_path)
        path_data = source.load()

        # Detect laps
        detector = YCrossingLapDetector()
        multilap_data = MultiLapData.from_source(source, detector)

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap_data)

        # Segment
        segmenter = CourseSegmenter(GradientSegmentation())
        segmentation = segmenter.segment(mean_course)

        # Assign
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Validate
        self.assertEqual(len(segment_ids), len(path_data))
        self.assertGreater(len(set(segment_ids)), 1,
                          "Should have multiple segments")


class TestSegmentationCorrectness(unittest.TestCase):
    """Test that segmentation produces correct results"""

    def test_straight_detection(self):
        """Test that straight sections are detected correctly"""
        # Create path with clear straight section
        t = np.linspace(0, 10, 100)
        x = t  # Straight line
        y = np.zeros_like(t)
        h = np.zeros_like(t)
        v = np.ones_like(t)

        path_data = PathData(t, x, y, h, v)

        # Create fake multilap (just one "lap")
        from donkeycar.parts.course_analysis.lap_detection import LapBoundary
        boundary = LapBoundary(0, len(t)-1, t[0], t[-1])
        multilap = MultiLapData(path_data, [boundary])

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        # Segment
        segmenter = CourseSegmenter(GradientSegmentation())
        segmentation = segmenter.segment(mean_course)

        # Should detect as straight
        from donkeycar.parts.course_analysis import SegmentType
        straight_segments = [s for s in segmentation.segments
                            if s.segment_type == SegmentType.STRAIGHT]

        self.assertGreater(len(straight_segments), 0,
                          "Should detect straight segments")


class TestNoMagicNumbers(unittest.TestCase):
    """Verify all magic numbers have been extracted"""

    def test_lap_detector_params(self):
        """Drift detector should have all magic numbers in params"""
        detector = DriftLapDetector()

        required_params = [
            'weighted_avg_weights',
            'reversal_tolerance',
            'vicinity_window',
            'time_factor_weight',
            'distance_factor_weight',
        ]

        for param in required_params:
            self.assertIn(param, detector.params,
                         f"Magic number {param} not in params!")

    def test_mean_course_params(self):
        """MeanCourseBuilder should have all params"""
        builder = MeanCourseBuilder()

        required_params = [
            'resampling_interval',
            'position_smoothing_window',
            'heading_smoothing_window',
            'loop_closure_pct',
        ]

        for param in required_params:
            self.assertIn(param, builder.params,
                         f"Magic number {param} not in params!")

    def test_segmenter_params(self):
        """CourseSegmenter should have all params"""
        segmenter = CourseSegmenter(GradientSegmentation())

        required_params = [
            'curvature_window',
            'straight_curvature_threshold',
            'inflection_chicane_threshold',
            'gradient_prominence',
        ]

        for param in required_params:
            self.assertIn(param, segmenter.params,
                         f"Magic number {param} not in params!")

    def test_assigner_params(self):
        """SegmentAssigner should have all tolerances as params"""
        # Create minimal segmentation for testing
        path_data = create_synthetic_3lap_oval(points_per_lap=50)
        from donkeycar.parts.course_analysis.lap_detection import LapBoundary
        boundary = LapBoundary(0, len(path_data)-1,
                              path_data.timestamp[0],
                              path_data.timestamp[-1])
        multilap = MultiLapData(path_data, [boundary])

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(GradientSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)

        required_params = [
            'boundary_distance_tolerance',
            'crossing_zero_tolerance',
            'crossing_t_tolerance',
            'normal_limit_factor',
        ]

        for param in required_params:
            self.assertIn(param, assigner.params,
                         f"Magic number {param} not in params!")


if __name__ == '__main__':
    unittest.main()
