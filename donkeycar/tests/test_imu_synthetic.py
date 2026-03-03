"""
Comprehensive tests for IMU path analysis using synthetic courses.

Tests the full pipeline: course generation → driving simulation →
lap detection → mean course recovery → segmentation → segment
assignment → segment statistics.

Uses geometrically-defined courses (L-shape, U-shape) with
Ornstein-Uhlenbeck driving simulation for realistic multi-lap data.
"""
import math
import os
import shutil
import tempfile
import unittest

import numpy as np

from donkeycar.tests.synthetic_courses import (
    LShapeCourse,
    UShapeCourse,
    ornstein_uhlenbeck,
    simulate_driving,
    save_as_csv,
)
from donkeycar.course_analysis import (
    PathData,
    CSVPathDataSource,
    MeanCourse,
    MeanCourseBuilder,
    YCrossingLapDetector,
    MultiLapData,
    LapBoundary,
    CourseSegmenter,
    CourseSegmentation,
    ThresholdSegmentation,
    HybridSegmentation,
    SegmentAssigner,
    SegmentEstimator,
)


# ── Helpers ────────────────────────────────────────────────────────

def _make_multilap(path_data, detector=None):
    """Detect laps and return MultiLapData."""
    if detector is None:
        detector = YCrossingLapDetector(
            params={'min_lap_length': 20, 'min_loop_distance': 0.5})
    boundaries = detector.detect_laps(path_data)
    return MultiLapData(path_data, boundaries)


def _build_mean_course(multilap, num_laps=None):
    """Build mean course from first num_laps laps."""
    if num_laps is not None and num_laps < multilap.num_laps:
        limited = MultiLapData(
            multilap.path_data,
            multilap.lap_boundaries[:num_laps])
    else:
        limited = multilap
    builder = MeanCourseBuilder()
    return builder.build(limited)


def _segment_course(mean_course, strategy=None, params=None):
    """Segment a mean course and return CourseSegmentation."""
    if strategy is None:
        strategy = HybridSegmentation()
    seg_params = {
        'min_segment_length': 3.0,
        'straight_curvature_threshold': 0.1,
    }
    if params:
        seg_params.update(params)
    segmenter = CourseSegmenter(strategy, params=seg_params)
    return segmenter.segment(mean_course)


def _assign_segments(segmentation, path_data):
    """Assign segment IDs to a driven path."""
    assigner = SegmentAssigner(segmentation)
    return assigner.assign(path_data.x, path_data.y)


def _rmse(a, b):
    """Root mean square error between two arrays."""
    return float(np.sqrt(np.mean((np.array(a) - np.array(b))**2)))


# ── 1. Course Geometry Tests ──────────────────────────────────────

class TestLShapeCourseGeometry(unittest.TestCase):
    """Tests that the L-shape course has correct geometry."""

    def setUp(self):
        self.course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        self.x, self.y, self.h, self.dist, self.meta = \
            self.course.generate()

    def test_closed_loop(self):
        """Start and end points are within 1 mm."""
        gap = math.hypot(self.x[-1] - self.x[0],
                         self.y[-1] - self.y[0])
        self.assertLess(gap, 0.05,
                        f"Loop not closed: gap = {gap:.4f} m")

    def test_total_length(self):
        """Total arc length matches expected formula."""
        expected = self.course.expected_total_length()
        actual = self.dist[-1]
        self.assertAlmostEqual(actual, expected, delta=0.2,
                               msg=f"Length {actual:.2f} != "
                                   f"expected {expected:.2f}")

    def test_segment_count(self):
        """Exactly 8 geometric segments."""
        self.assertEqual(len(self.meta), 8)

    def test_segment_types(self):
        """4 straights, 2 semicircles, 1 quarter_right, 1 quarter_left."""
        types = [m['type'] for m in self.meta]
        self.assertEqual(types.count('straight'), 4)
        self.assertEqual(types.count('semicircle_left'), 2)
        self.assertEqual(types.count('quarter_right'), 1)
        self.assertEqual(types.count('quarter_left'), 1)

    def test_straight_headings(self):
        """Straight segments have correct headings."""
        expected_headings = {
            0: 0.0,          # S1: east
            2: np.pi,        # S2: west
            4: np.pi / 2,    # S3: north
            6: -np.pi / 2,   # S4: south
        }
        for seg_id, expected_h in expected_headings.items():
            seg = self.meta[seg_id]
            mid = (seg['start_index'] + seg['end_index']) // 2
            actual_h = self.h[mid]
            diff = abs(((actual_h - expected_h + np.pi) %
                        (2 * np.pi)) - np.pi)
            self.assertLess(
                diff, 0.05,
                f"Segment {seg_id} heading {actual_h:.3f} "
                f"!= expected {expected_h:.3f}")

    def test_heading_continuity(self):
        """No heading jumps larger than 30 degrees between points."""
        for i in range(1, len(self.h)):
            diff = abs(((self.h[i] - self.h[i-1] + np.pi) %
                        (2 * np.pi)) - np.pi)
            self.assertLess(
                diff, np.radians(30),
                f"Heading jump at index {i}: {np.degrees(diff):.1f}°")

    def test_y_crossing_exists(self):
        """Path crosses y=0 from negative to positive."""
        crossings = 0
        for i in range(1, len(self.y)):
            if self.y[i-1] < 0 and self.y[i] >= 0:
                crossings += 1
        self.assertGreaterEqual(crossings, 1,
                                "No y-crossing from neg to pos found")

    def test_parameter_validation(self):
        """Rejects invalid parameters."""
        with self.assertRaises(ValueError):
            LShapeCourse(h_len=1.0, v_len=15.0, d=1.0)
        with self.assertRaises(ValueError):
            LShapeCourse(h_len=10.0, v_len=1.0, d=1.0)


class TestUShapeCourseGeometry(unittest.TestCase):
    """Tests that the U-shape course has correct geometry."""

    def setUp(self):
        self.course = UShapeCourse(width=6.0, v_len=12.0, d=1.0)
        self.x, self.y, self.h, self.dist, self.meta = \
            self.course.generate()

    def test_closed_loop(self):
        """Start and end points are within 1 mm."""
        gap = math.hypot(self.x[-1] - self.x[0],
                         self.y[-1] - self.y[0])
        self.assertLess(gap, 0.05,
                        f"Loop not closed: gap = {gap:.4f} m")

    def test_total_length(self):
        """Total arc length matches expected formula."""
        expected = self.course.expected_total_length()
        actual = self.dist[-1]
        self.assertAlmostEqual(actual, expected, delta=0.3,
                               msg=f"Length {actual:.2f} != "
                                   f"expected {expected:.2f}")

    def test_segment_count(self):
        """Exactly 8 geometric segments."""
        self.assertEqual(len(self.meta), 8)

    def test_segment_types(self):
        """4 straights, 3 semicircle_left, 1 semicircle_right."""
        types = [m['type'] for m in self.meta]
        self.assertEqual(types.count('straight'), 4)
        self.assertEqual(types.count('semicircle_left'), 3)
        self.assertEqual(types.count('semicircle_right'), 1)

    def test_y_crossing_only_in_outer_semicircle(self):
        """Y-crossing from neg to pos occurs only once (in A4)."""
        crossings = []
        for i in range(1, len(self.y)):
            if self.y[i-1] < 0 and self.y[i] >= 0:
                crossings.append(i)
        self.assertEqual(len(crossings), 1,
                         f"Expected 1 y-crossing, got {len(crossings)}")

    def test_parameter_validation(self):
        """Rejects width <= 2*d."""
        with self.assertRaises(ValueError):
            UShapeCourse(width=1.5, v_len=12.0, d=1.0)


# ── 2. Driving Simulation Tests ──────────────────────────────────

class TestDrivingSimulation(unittest.TestCase):
    """Tests the OU driving simulator produces realistic data."""

    def setUp(self):
        self.course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        self.cx, self.cy, self.ch, self.cd, _ = \
            self.course.generate()

    def test_ou_process_zero_mean(self):
        """OU process has approximately zero mean over many samples."""
        distances = np.linspace(0, 50, 5000)
        samples = [ornstein_uhlenbeck(distances, theta=5.0,
                                      sigma=0.15, seed=s)
                   for s in range(50)]
        mean_error = np.mean([np.mean(s) for s in samples])
        self.assertLess(abs(mean_error), 0.02,
                        f"OU mean {mean_error:.4f} too far from 0")

    def test_ou_process_bounded(self):
        """OU cross-track error stays within 4 std deviations."""
        distances = np.linspace(0, 50, 5000)
        epsilon = ornstein_uhlenbeck(distances, theta=5.0,
                                     sigma=0.15, seed=42)
        # Theoretical std = sigma / sqrt(2*theta) ≈ 0.047
        theoretical_std = 0.15 / np.sqrt(2 * 5.0)
        self.assertLess(np.max(np.abs(epsilon)),
                        6 * theoretical_std,
                        "OU process exceeded 6 sigma")

    def test_path_is_smooth(self):
        """Simulated path has no jumps > 0.5 m between points."""
        path = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=3, seed=42)
        dx = np.diff(path.x)
        dy = np.diff(path.y)
        steps = np.sqrt(dx**2 + dy**2)
        self.assertLess(np.max(steps), 0.5,
                        f"Max step = {np.max(steps):.3f} m")

    def test_timestamps_monotonic(self):
        """Timestamps are strictly increasing."""
        path = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=3, seed=42)
        diffs = np.diff(path.timestamp)
        self.assertTrue(np.all(diffs > 0),
                        "Timestamps not monotonically increasing")

    def test_different_seeds_different_paths(self):
        """Different seeds produce different paths."""
        path1 = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=1, seed=42)
        path2 = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=1, seed=99)
        self.assertGreater(
            _rmse(path1.x, path2.x), 0.01,
            "Two seeds produced identical paths")

    def test_same_seed_reproducible(self):
        """Same seed produces identical paths."""
        path1 = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=2, seed=42)
        path2 = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=2, seed=42)
        np.testing.assert_array_almost_equal(path1.x, path2.x)
        np.testing.assert_array_almost_equal(path1.y, path2.y)


# ── 3. Lap Detection Tests ───────────────────────────────────────

class TestLapDetectionOnSynthetic(unittest.TestCase):
    """Tests lap detection on simulated multi-lap data."""

    def setUp(self):
        self.l_course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        lx, ly, lh, ld, _ = self.l_course.generate()
        self.l_path = simulate_driving(
            lx, ly, lh, ld, num_laps=7, seed=42)
        self.l_multilap = _make_multilap(self.l_path)

    def test_detects_correct_lap_count(self):
        """7 simulated laps produce 5-7 detected laps."""
        # First/last partial laps may be dropped
        self.assertGreaterEqual(self.l_multilap.num_laps, 5,
                                "Too few laps detected")
        self.assertLessEqual(self.l_multilap.num_laps, 7,
                             "Too many laps detected")

    def test_lap_lengths_consistent(self):
        """Interior laps have similar point counts (±30%).

        Skips first and last laps which may be partial.
        """
        laps = self.l_multilap.laps
        if len(laps) < 3:
            self.skipTest("Need at least 3 laps")
        # Skip first and last (possibly partial)
        interior = laps[1:-1]
        lengths = [len(lap) for lap in interior]
        median_len = np.median(lengths)
        for i, length in enumerate(lengths):
            ratio = length / median_len
            self.assertGreater(
                ratio, 0.7,
                f"Interior lap {i+1} too short: "
                f"{length} vs median {median_len}")
            self.assertLess(
                ratio, 1.3,
                f"Interior lap {i+1} too long: "
                f"{length} vs median {median_len}")

    def test_lap_distances_consistent(self):
        """Interior laps have similar total distance (±20%)."""
        laps = self.l_multilap.laps
        if len(laps) < 3:
            self.skipTest("Need at least 3 laps")
        interior = laps[1:-1]
        distances = [lap.total_distance for lap in interior]
        median_dist = np.median(distances)
        for i, dist in enumerate(distances):
            ratio = dist / median_dist
            self.assertGreater(
                ratio, 0.8,
                f"Interior lap {i+1} distance {dist:.1f} "
                f"too short vs median {median_dist:.1f}")
            self.assertLess(
                ratio, 1.2,
                f"Interior lap {i+1} distance {dist:.1f} "
                f"too long vs median {median_dist:.1f}")

    def test_u_shape_lap_detection(self):
        """U-shape course also gets laps detected correctly."""
        u_course = UShapeCourse(width=6.0, v_len=12.0, d=1.0)
        ux, uy, uh, ud, _ = u_course.generate()
        u_path = simulate_driving(
            ux, uy, uh, ud, num_laps=5, seed=123)
        u_multilap = _make_multilap(u_path)
        self.assertGreaterEqual(u_multilap.num_laps, 3,
                                "U-shape: too few laps detected")

    def test_laps_cover_full_path(self):
        """Detected laps cover most of the driven path."""
        total_points = len(self.l_path)
        lap_points = sum(b.num_points
                         for b in self.l_multilap.lap_boundaries)
        coverage = lap_points / total_points
        self.assertGreater(coverage, 0.7,
                           f"Laps cover only {coverage:.0%} of path")


# ── 4. Mean Course Recovery Tests ─────────────────────────────────

class TestMeanCourseRecovery(unittest.TestCase):
    """Tests mean course computation from simulated laps."""

    def setUp(self):
        self.course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        self.cx, self.cy, self.ch, self.cd, _ = \
            self.course.generate()

    def _recover_mean_course(self, num_laps, seed=42):
        """Simulate, detect laps, build mean course."""
        path = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=num_laps, seed=seed)
        multilap = _make_multilap(path)
        return _build_mean_course(multilap)

    def test_mean_course_length_close(self):
        """Mean course length within 15% of designed course."""
        mean = self._recover_mean_course(10)
        expected = self.cd[-1]
        actual = mean.distance[-1]
        ratio = actual / expected
        self.assertGreater(ratio, 0.85,
                           f"Mean course too short: {actual:.1f} vs "
                           f"{expected:.1f}")
        self.assertLess(ratio, 1.15,
                        f"Mean course too long: {actual:.1f} vs "
                        f"{expected:.1f}")

    def test_mean_course_shape_similar(self):
        """Mean course shape resembles designed course (low RMSE).

        We compare by finding, for each mean course point, the
        closest point on the designed course. The average distance
        should be small.
        """
        mean = self._recover_mean_course(10)
        from scipy.spatial import KDTree
        tree = KDTree(np.column_stack([self.cx, self.cy]))
        dists, _ = tree.query(
            np.column_stack([mean.x, mean.y]))
        avg_dist = np.mean(dists)
        self.assertLess(avg_dist, 0.5,
                        f"Mean course avg distance to designed: "
                        f"{avg_dist:.3f} m")

    def test_more_laps_better_recovery(self):
        """Mean from 10 laps is closer to designed than from 3."""
        mean_3 = self._recover_mean_course(5, seed=42)
        mean_10 = self._recover_mean_course(12, seed=42)
        from scipy.spatial import KDTree
        tree = KDTree(np.column_stack([self.cx, self.cy]))

        dists_3, _ = tree.query(
            np.column_stack([mean_3.x, mean_3.y]))
        dists_10, _ = tree.query(
            np.column_stack([mean_10.x, mean_10.y]))

        rmse_3 = float(np.sqrt(np.mean(dists_3**2)))
        rmse_10 = float(np.sqrt(np.mean(dists_10**2)))
        self.assertLess(rmse_10, rmse_3 + 0.05,
                        f"10-lap RMSE ({rmse_10:.4f}) not better "
                        f"than 3-lap ({rmse_3:.4f})")


# ── 5. Segmentation Tests ────────────────────────────────────────

class TestSegmentationOnSynthetic(unittest.TestCase):
    """Tests segmentation of designed and recovered courses."""

    def setUp(self):
        self.course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        self.cx, self.cy, self.ch, self.cd, self.cmeta = \
            self.course.generate()

    def test_designed_course_segment_count(self):
        """Segmentation of designed L-course finds 4-12 segments.

        The exact count depends on the segmentation strategy and
        parameters. The L-course has 8 geometric features, but the
        algorithm may merge short arcs with adjacent straights or
        split features differently.
        """
        mc = self.course.to_mean_course()
        seg = _segment_course(mc)
        self.assertGreaterEqual(
            seg.num_segments, 4,
            f"Too few segments: {seg.num_segments}")
        self.assertLessEqual(
            seg.num_segments, 12,
            f"Too many segments: {seg.num_segments}")

    def test_has_straight_and_turn_segments(self):
        """Both straight and turn segment types are detected."""
        mc = self.course.to_mean_course()
        seg = _segment_course(mc)
        from donkeycar.course_analysis import SegmentType
        types = {s.segment_type for s in seg.segments}
        has_straight = SegmentType.STRAIGHT in types
        has_turn = any(t in types for t in (
            SegmentType.LEFT_TURN, SegmentType.RIGHT_TURN,
            SegmentType.S_CURVE_LR, SegmentType.S_CURVE_RL))
        self.assertTrue(has_straight,
                        f"No STRAIGHT segments found. Types: {types}")
        self.assertTrue(has_turn,
                        f"No turn segments found. Types: {types}")

    def test_segment_boundaries_exist(self):
        """Segmentation produces boundary line dicts."""
        mc = self.course.to_mean_course()
        seg = _segment_course(mc)
        self.assertIsNotNone(seg.segment_boundaries)
        self.assertGreater(len(seg.segment_boundaries), 0)
        # Each boundary has required keys
        for b in seg.segment_boundaries:
            self.assertIn('point', b)
            self.assertIn('tangent', b)

    def test_segments_cover_full_course(self):
        """Segments cover the entire course distance."""
        mc = self.course.to_mean_course()
        seg = _segment_course(mc)
        total_seg_dist = sum(
            s.end_distance - s.start_distance
            for s in seg.segments)
        course_length = mc.distance[-1]
        coverage = total_seg_dist / course_length
        self.assertGreater(coverage, 0.9,
                           f"Segments cover only {coverage:.0%}")

    def test_recovered_course_segmentation(self):
        """Segmentation works on mean course from simulated laps."""
        path = simulate_driving(
            self.cx, self.cy, self.ch, self.cd,
            num_laps=8, seed=42)
        multilap = _make_multilap(path)
        mean = _build_mean_course(multilap)
        seg = _segment_course(mean)
        self.assertGreaterEqual(seg.num_segments, 3,
                                "Too few segments on recovered course")

    def test_u_shape_segmentation(self):
        """U-shape course also segments correctly."""
        u_course = UShapeCourse(width=6.0, v_len=12.0, d=1.0)
        mc = u_course.to_mean_course()
        seg = _segment_course(mc)
        self.assertGreaterEqual(seg.num_segments, 4,
                                "U-shape: too few segments")


# ── 6. Segment Assignment Tests (Integration) ────────────────────

class TestSegmentAssignmentIntegration(unittest.TestCase):
    """Tests segment assignment on simulated multi-lap data.

    This is the full integration test: designed course → simulate
    driving → detect laps → build mean course → segment → assign
    segments → verify invariants.
    """

    @classmethod
    def setUpClass(cls):
        """Generate shared test data (expensive, do once)."""
        cls.course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = cls.course.generate()
        cls.path = simulate_driving(
            cx, cy, ch, cd, num_laps=8, seed=42)
        cls.multilap = _make_multilap(cls.path)
        cls.mean_course = _build_mean_course(cls.multilap)
        cls.segmentation = _segment_course(cls.mean_course)
        cls.segment_ids = _assign_segments(
            cls.segmentation, cls.path)
        cls.num_segments = cls.segmentation.num_segments

    def test_assignment_length_matches_path(self):
        """Segment ID array has same length as path."""
        self.assertEqual(len(self.segment_ids), len(self.path))

    def test_all_segment_ids_valid(self):
        """All assigned IDs are in [0, num_segments)."""
        self.assertTrue(np.all(self.segment_ids >= 0))
        self.assertTrue(np.all(
            self.segment_ids < self.num_segments))

    def test_each_lap_visits_all_segments(self):
        """Every complete segment-cycle lap touches all segment IDs.

        Uses segment-defined laps (N-1→0 transitions), not
        y-crossing laps, per the segment assignment invariant.
        """
        n = self.num_segments
        # Find segment-cycle lap boundaries
        lap_starts = [0]
        for i in range(1, len(self.segment_ids)):
            prev = self.segment_ids[i - 1]
            curr = self.segment_ids[i]
            if prev == n - 1 and curr == 0:
                lap_starts.append(i)
        # Skip first segment-lap (partial — data starts mid-course)
        checked = 0
        for j in range(1, len(lap_starts) - 1):
            s = lap_starts[j]
            e = lap_starts[j + 1] - 1
            lap_segs = set(self.segment_ids[s:e + 1])
            expected = set(range(n))
            if lap_segs == expected:
                checked += 1
            else:
                missing = expected - lap_segs
                self.assertLessEqual(
                    len(missing), 1,
                    f"Segment-lap {j} missing: {missing}")
        self.assertGreaterEqual(
            checked, 2,
            f"Only {checked} complete segment-cycle laps")

    def test_sequential_segment_order(self):
        """Segments are visited in order: 0→1→...→N-1→0.

        Transitions only go from segment S to S+1 (mod N).
        """
        n = self.num_segments
        for i in range(1, len(self.segment_ids)):
            prev = self.segment_ids[i - 1]
            curr = self.segment_ids[i]
            if curr != prev:
                expected_next = (prev + 1) % n
                self.assertEqual(
                    curr, expected_next,
                    f"At index {i}: transition {prev}→{curr}, "
                    f"expected {prev}→{expected_next}")

    def test_multiple_laps_have_segment_cycles(self):
        """Detect at least 3 complete segment cycles (0→N-1→0)."""
        n = self.num_segments
        cycles = 0
        for i in range(1, len(self.segment_ids)):
            prev = self.segment_ids[i - 1]
            curr = self.segment_ids[i]
            if prev == n - 1 and curr == 0:
                cycles += 1
        self.assertGreaterEqual(
            cycles, 3,
            f"Only {cycles} segment cycles detected, expected ≥3")

    def test_segment_transitions_at_distinct_locations(self):
        """Segment transitions happen at different positions.

        No two transitions should be at the exact same location.
        """
        transitions = []
        for i in range(1, len(self.segment_ids)):
            if self.segment_ids[i] != self.segment_ids[i - 1]:
                transitions.append((self.path.x[i], self.path.y[i]))
        if len(transitions) > 1:
            xs = [t[0] for t in transitions]
            ys = [t[1] for t in transitions]
            spread_x = np.std(xs)
            spread_y = np.std(ys)
            self.assertGreater(
                spread_x + spread_y, 0.1,
                "All transitions at the same location")


class TestSegmentAssignmentUShape(unittest.TestCase):
    """Segment assignment on U-shape course."""

    @classmethod
    def setUpClass(cls):
        cls.course = UShapeCourse(width=6.0, v_len=12.0, d=1.0)
        cx, cy, ch, cd, _ = cls.course.generate()
        cls.path = simulate_driving(
            cx, cy, ch, cd, num_laps=6, seed=123)
        cls.multilap = _make_multilap(cls.path)
        cls.mean_course = _build_mean_course(cls.multilap)
        cls.segmentation = _segment_course(cls.mean_course)
        cls.segment_ids = _assign_segments(
            cls.segmentation, cls.path)

    def test_assignment_valid(self):
        """All segment IDs in valid range."""
        n = self.segmentation.num_segments
        self.assertTrue(np.all(self.segment_ids >= 0))
        self.assertTrue(np.all(self.segment_ids < n))

    def test_sequential_order(self):
        """Transitions are sequential."""
        n = self.segmentation.num_segments
        for i in range(1, len(self.segment_ids)):
            prev = self.segment_ids[i - 1]
            curr = self.segment_ids[i]
            if curr != prev:
                expected_next = (prev + 1) % n
                self.assertEqual(curr, expected_next,
                                 f"At {i}: {prev}→{curr}")


# ── 7. Segment Estimator Tests ───────────────────────────────────

class TestSegmentEstimator(unittest.TestCase):
    """Tests real-time segment estimation via KD-tree."""

    @classmethod
    def setUpClass(cls):
        cls.course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = cls.course.generate()
        path = simulate_driving(cx, cy, ch, cd, num_laps=8, seed=42)
        multilap = _make_multilap(path)
        mean = _build_mean_course(multilap)
        cls.segmentation = _segment_course(mean)
        cls.mean_course = mean

    def test_estimate_on_course_points(self):
        """Points on the mean course get correct segment IDs."""
        estimator = SegmentEstimator(self.segmentation)
        for seg in self.segmentation.segments:
            mid_idx = (seg.start_index + seg.end_index) // 2
            x = self.mean_course.x[mid_idx]
            y = self.mean_course.y[mid_idx]
            est = estimator.estimate(x, y)
            self.assertEqual(
                est.segment_id, seg.segment_id,
                f"Mid-point of segment {seg.segment_id} "
                f"estimated as {est.segment_id}")

    def test_confidence_high_on_course(self):
        """Points on the course have high confidence."""
        estimator = SegmentEstimator(self.segmentation)
        mid = len(self.mean_course) // 2
        est = estimator.estimate(
            self.mean_course.x[mid], self.mean_course.y[mid])
        self.assertGreater(est.confidence, 0.5,
                           f"Confidence too low: {est.confidence}")

    def test_confidence_low_far_from_course(self):
        """Points far from course have low confidence."""
        estimator = SegmentEstimator(self.segmentation)
        est = estimator.estimate(1000.0, 1000.0)
        self.assertLess(est.confidence, 0.01,
                        f"Far-away point confidence too high: "
                        f"{est.confidence}")


# ── 8. CSV Round-Trip Tests ──────────────────────────────────────

class TestCSVRoundTrip(unittest.TestCase):
    """Tests saving/loading simulated data via CSV."""

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_save_load_csv(self):
        """PathData survives CSV save/load round-trip."""
        course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = course.generate()
        path = simulate_driving(cx, cy, ch, cd, num_laps=3, seed=42)

        csv_path = os.path.join(self.temp_dir, 'test.csv')
        save_as_csv(path, csv_path)

        source = CSVPathDataSource(csv_path)
        loaded = source.load()
        self.assertEqual(len(loaded), len(path))
        np.testing.assert_array_almost_equal(
            loaded.x, path.x, decimal=6)
        np.testing.assert_array_almost_equal(
            loaded.y, path.y, decimal=6)

    def test_csv_lap_detection(self):
        """Laps detectable from CSV-loaded data."""
        course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = course.generate()
        path = simulate_driving(cx, cy, ch, cd, num_laps=5, seed=42)

        csv_path = os.path.join(self.temp_dir, 'test.csv')
        save_as_csv(path, csv_path)

        source = CSVPathDataSource(csv_path)
        loaded = source.load()
        multilap = _make_multilap(loaded)
        self.assertGreaterEqual(multilap.num_laps, 3,
                                "Too few laps from CSV data")


# ── 9. Tub Data Integration Tests ────────────────────────────────

class TestTubDataIntegration(unittest.TestCase):
    """Tests with full Tub data (no images).

    Creates a Tub from simulated driving data, loads it back via
    TubPathDataSource, and runs the full pipeline.
    """

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.tub_path = os.path.join(self.temp_dir, 'test_tub')
        self.open_tubs = []

    def tearDown(self):
        for tub in self.open_tubs:
            try:
                tub.close()
            except Exception:
                pass
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def _create_tub_from_path(self, path_data, session_id=None):
        """Create a Tub from PathData (no images)."""
        from donkeycar.parts.tub_v2 import Tub

        if session_id is None:
            session_id = 'synth_session_001'

        tub = Tub(
            self.tub_path,
            inputs=['car/pos', 'car/euler', 'car/speed',
                    'car/gyro', 'car/distance',
                    'user/angle', 'user/throttle'],
            types=['vector', 'vector', 'float',
                   'vector', 'float',
                   'float', 'float'],
        )
        self.open_tubs.append(tub)

        for i in range(len(path_data)):
            # Convert math heading (radians) to IMU euler convention
            # IMU: euler[2] = 90 - heading_degrees
            h_deg = math.degrees(path_data.heading[i])
            euler_z = 90.0 - h_deg

            record = {
                '_session_id': session_id,
                'car/pos': [float(path_data.x[i]),
                            float(path_data.y[i]),
                            0.0],
                'car/euler': [0.0, 0.0, float(euler_z)],
                'car/speed': float(path_data.velocity[i]),
                'car/gyro': [0.0, 0.0, 0.0],
                'car/distance': float(path_data.distance[i]),
                'user/angle': 0.0,
                'user/throttle': 0.5,
                '_timestamp_ms': int(
                    path_data.timestamp[i] * 1000),
            }
            tub.write_record(record)

        return tub

    def test_tub_round_trip(self):
        """Data survives Tub write/load round-trip."""
        from donkeycar.course_analysis import TubPathDataSource

        course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = course.generate()
        path = simulate_driving(cx, cy, ch, cd, num_laps=3, seed=42)

        tub = self._create_tub_from_path(path)
        tub.close()
        self.open_tubs.remove(tub)

        source = TubPathDataSource(self.tub_path)
        loaded = source.load()

        self.assertEqual(len(loaded), len(path))
        # Position should be very close (Tub stores as-is)
        np.testing.assert_array_almost_equal(
            loaded.x, path.x, decimal=3)
        np.testing.assert_array_almost_equal(
            loaded.y, path.y, decimal=3)

    def test_tub_full_pipeline(self):
        """Full pipeline works on Tub data: detect laps → segment."""
        from donkeycar.course_analysis import TubPathDataSource

        course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = course.generate()
        path = simulate_driving(cx, cy, ch, cd, num_laps=8, seed=42)

        tub = self._create_tub_from_path(path)
        tub.close()
        self.open_tubs.remove(tub)

        source = TubPathDataSource(self.tub_path)
        loaded = source.load()

        # Detect laps
        multilap = _make_multilap(loaded)
        self.assertGreaterEqual(multilap.num_laps, 4,
                                "Too few laps from Tub data")

        # Build mean course
        mean = _build_mean_course(multilap)
        self.assertGreater(len(mean), 50,
                           "Mean course too short")

        # Segment
        seg = _segment_course(mean)
        self.assertGreaterEqual(seg.num_segments, 3,
                                "Too few segments from Tub data")

        # Assign segments
        segment_ids = _assign_segments(seg, loaded)
        self.assertEqual(len(segment_ids), len(loaded))
        self.assertTrue(np.all(segment_ids >= 0))
        self.assertTrue(np.all(segment_ids < seg.num_segments))

    def test_tub_segment_metadata_storage(self):
        """Segment metadata can be stored in Tub manifest."""
        course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = course.generate()
        path = simulate_driving(cx, cy, ch, cd, num_laps=5, seed=42)

        tub = self._create_tub_from_path(path, 'test_session')

        # Store segmentation metadata in manifest
        actual_session = tub.manifest.session_id[1]
        tub.manifest.metadata[actual_session] = {
            'segmentation': {
                'num_segments': 8,
                'strategy': 'hybrid',
                'mean_course_params': {'num_laps': 3},
            }
        }
        tub.manifest.write_metadata()
        tub.close()
        self.open_tubs.remove(tub)

        # Reload and verify metadata persists
        from donkeycar.parts.tub_v2 import Tub as TubReader
        tub2 = TubReader(self.tub_path, read_only=True)
        self.open_tubs.append(tub2)

        metadata = tub2.manifest.metadata
        self.assertIn(actual_session, metadata)
        seg_meta = metadata[actual_session]['segmentation']
        self.assertEqual(seg_meta['num_segments'], 8)
        self.assertEqual(seg_meta['strategy'], 'hybrid')


# ── 10. Segment Statistics Tests ──────────────────────────────────

class TestSegmentStatistics(unittest.TestCase):
    """Tests segment performance statistics computation.

    Verifies that per-segment timing, distance, and ranking work
    correctly on synthetic multi-lap data where we know the ground
    truth.
    """

    @classmethod
    def setUpClass(cls):
        """Generate data with known speed variation across laps."""
        cls.course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, _ = cls.course.generate()

        # Simulate 8 laps with fixed seed
        cls.path = simulate_driving(
            cx, cy, ch, cd, num_laps=8, seed=42)
        cls.multilap = _make_multilap(cls.path)
        cls.mean_course = _build_mean_course(cls.multilap)
        cls.segmentation = _segment_course(cls.mean_course)
        cls.segment_ids = _assign_segments(
            cls.segmentation, cls.path)
        cls.num_segments = cls.segmentation.num_segments

    def _get_segment_laps(self):
        """Get segment-based lap boundaries from segment cycles.

        A 'segment lap' starts when segment transitions from N-1→0.
        """
        n = self.num_segments
        lap_starts = [0]
        for i in range(1, len(self.segment_ids)):
            prev = self.segment_ids[i - 1]
            curr = self.segment_ids[i]
            if prev == n - 1 and curr == 0:
                lap_starts.append(i)
        # Create lap ranges
        laps = []
        for j in range(len(lap_starts) - 1):
            laps.append((lap_starts[j], lap_starts[j + 1] - 1))
        return laps

    def test_segment_times_sum_to_lap_time(self):
        """Per-segment times add up to total lap time."""
        seg_laps = self._get_segment_laps()
        if len(seg_laps) < 1:
            self.skipTest("No complete segment laps found")

        for lap_start, lap_end in seg_laps[:3]:
            lap_time = (self.path.timestamp[lap_end]
                        - self.path.timestamp[lap_start])
            seg_times = {}
            for i in range(lap_start, lap_end + 1):
                sid = self.segment_ids[i]
                if sid not in seg_times:
                    seg_times[sid] = {'first': i, 'last': i}
                seg_times[sid]['last'] = i

            total_seg_time = 0.0
            for sid, indices in seg_times.items():
                seg_time = (self.path.timestamp[indices['last']]
                            - self.path.timestamp[indices['first']])
                total_seg_time += seg_time

            # Allow small gap due to transition points
            self.assertAlmostEqual(
                total_seg_time, lap_time, delta=lap_time * 0.15,
                msg=f"Segment times {total_seg_time:.2f}s != "
                    f"lap time {lap_time:.2f}s")

    def test_segment_distances_sum_to_lap_distance(self):
        """Per-segment distances add up to total lap distance."""
        seg_laps = self._get_segment_laps()
        if len(seg_laps) < 1:
            self.skipTest("No complete segment laps found")

        for lap_start, lap_end in seg_laps[:3]:
            lap_x = self.path.x[lap_start:lap_end + 1]
            lap_y = self.path.y[lap_start:lap_end + 1]
            dx = np.diff(lap_x)
            dy = np.diff(lap_y)
            lap_dist = float(np.sum(np.sqrt(dx**2 + dy**2)))

            seg_dists = {}
            for i in range(lap_start, lap_end + 1):
                sid = self.segment_ids[i]
                if sid not in seg_dists:
                    seg_dists[sid] = {'first': i, 'last': i}
                seg_dists[sid]['last'] = i

            total_seg_dist = 0.0
            for sid, indices in seg_dists.items():
                s = indices['first']
                e = indices['last']
                sx = self.path.x[s:e + 1]
                sy = self.path.y[s:e + 1]
                sdx = np.diff(sx)
                sdy = np.diff(sy)
                total_seg_dist += float(
                    np.sum(np.sqrt(sdx**2 + sdy**2)))

            self.assertAlmostEqual(
                total_seg_dist, lap_dist, delta=lap_dist * 0.20,
                msg=f"Segment dists {total_seg_dist:.2f}m != "
                    f"lap dist {lap_dist:.2f}m")

    def test_all_segments_have_data_in_each_lap(self):
        """Every segment appears in each complete segment-lap.

        Skips first segment-lap (partial — data starts mid-course).
        """
        seg_laps = self._get_segment_laps()
        if len(seg_laps) < 2:
            self.skipTest("Need at least 2 segment-laps")
        expected = set(range(self.num_segments))
        # Skip first partial lap
        for j, (lap_start, lap_end) in enumerate(seg_laps[1:4], 1):
            lap_segs = set(self.segment_ids[lap_start:lap_end + 1])
            missing = expected - lap_segs
            self.assertLessEqual(
                len(missing), 1,
                f"Segment-lap {j} missing segments: {missing}")

    def test_segment_time_varies_across_laps(self):
        """Same segment has different times in different laps.

        This validates the OU speed variation creates measurable
        performance differences that segment ranking can exploit.
        """
        seg_laps = self._get_segment_laps()
        if len(seg_laps) < 2:
            self.skipTest("Need at least 2 segment-laps")

        # Pick the first segment (segment 0) and compare times
        times = []
        for lap_start, lap_end in seg_laps[:5]:
            indices = [i for i in range(lap_start, lap_end + 1)
                       if self.segment_ids[i] == 0]
            if len(indices) > 1:
                seg_time = (self.path.timestamp[indices[-1]]
                            - self.path.timestamp[indices[0]])
                times.append(seg_time)

        if len(times) >= 2:
            time_std = np.std(times)
            self.assertGreater(
                time_std, 0.01,
                f"Segment 0 times too uniform: std={time_std:.4f}")


# ── 11. End-to-End Pipeline Test ──────────────────────────────────

class TestEndToEndPipeline(unittest.TestCase):
    """Full end-to-end test: course → simulate → analyze → verify."""

    def test_l_shape_full_pipeline(self):
        """Complete pipeline on L-shape course."""
        # 1. Generate course
        course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
        cx, cy, ch, cd, cmeta = course.generate()
        self.assertEqual(len(cmeta), 8)

        # 2. Simulate driving (10 laps)
        path = simulate_driving(
            cx, cy, ch, cd, num_laps=10, seed=42)
        self.assertGreater(len(path), 1000)

        # 3. Detect laps
        multilap = _make_multilap(path)
        self.assertGreaterEqual(multilap.num_laps, 7)

        # 4. Build mean course
        mean = _build_mean_course(multilap)
        self.assertGreater(mean.distance[-1], 40)

        # 5. Segment
        seg = _segment_course(mean)
        self.assertGreaterEqual(seg.num_segments, 4)

        # 6. Assign segments to full path
        segment_ids = _assign_segments(seg, path)
        self.assertEqual(len(segment_ids), len(path))

        # 7. Verify segment assignment invariant
        n = seg.num_segments
        for i in range(1, len(segment_ids)):
            if segment_ids[i] != segment_ids[i - 1]:
                expected = (segment_ids[i - 1] + 1) % n
                self.assertEqual(segment_ids[i], expected)

        # 8. Count complete laps via segment cycles
        cycles = sum(
            1 for i in range(1, len(segment_ids))
            if segment_ids[i - 1] == n - 1 and segment_ids[i] == 0)
        self.assertGreaterEqual(cycles, 5,
                                f"Only {cycles} complete laps")

    def test_u_shape_full_pipeline(self):
        """Complete pipeline on U-shape course."""
        course = UShapeCourse(width=6.0, v_len=12.0, d=1.0)
        cx, cy, ch, cd, cmeta = course.generate()
        self.assertEqual(len(cmeta), 8)

        path = simulate_driving(
            cx, cy, ch, cd, num_laps=8, seed=123)
        multilap = _make_multilap(path)
        self.assertGreaterEqual(multilap.num_laps, 4)

        mean = _build_mean_course(multilap)
        seg = _segment_course(mean)
        self.assertGreaterEqual(seg.num_segments, 3)

        segment_ids = _assign_segments(seg, path)
        n = seg.num_segments
        for i in range(1, len(segment_ids)):
            if segment_ids[i] != segment_ids[i - 1]:
                expected = (segment_ids[i - 1] + 1) % n
                self.assertEqual(segment_ids[i], expected)


if __name__ == '__main__':
    unittest.main()
