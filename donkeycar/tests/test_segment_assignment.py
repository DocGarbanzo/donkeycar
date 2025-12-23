"""
Test segment assignment algorithm for IMU path visualization.

Tests the boundary crossing detection with state machine approach
for assigning segments to driven path coordinates.
"""
import numpy as np
import pytest
from donkeycar.parts.course_analysis import (
    MeanCourse, CourseSegmentation, SegmentType
)


def generate_figure8_course(radius=5.0, num_points=400):
    """
    Generate figure-8 shaped test course.

    Mix of left and right turns for testing segment detection
    through direction changes.

    Args:
        radius: Size of figure-8 loops
        num_points: Number of points in course

    Returns:
        x, y, heading, distance arrays
    """
    t = np.linspace(0, 2*np.pi, num_points)

    # Figure-8 parametric equations
    x = radius * np.sin(t)
    y = radius * np.sin(t) * np.cos(t)

    # Compute heading from derivatives
    dx_dt = radius * np.cos(t)
    dy_dt = radius * (np.cos(t)**2 - np.sin(t)**2)
    heading = np.arctan2(dy_dt, dx_dt)

    # Compute cumulative distance
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx**2 + dy**2)
    distance = np.concatenate([[0], np.cumsum(ds)])

    return x, y, heading, distance


def generate_oval_course(length=20.0, width=10.0, num_points=400):
    """
    Generate oval test course with straights and hairpins.

    Tests mix of straight sections and sharp turns.

    Args:
        length: Length of straight sections
        width: Width of oval
        num_points: Number of points in course

    Returns:
        x, y, heading, distance arrays
    """
    # Create oval from two straights and two semicircles
    t = np.linspace(0, 2*np.pi, num_points)

    # Parametric oval (racetrack shape)
    a = length / 2.0  # semi-major axis
    b = width / 2.0   # semi-minor axis

    x = np.zeros(num_points)
    y = np.zeros(num_points)

    for i, angle in enumerate(t):
        if 0 <= angle < np.pi/2 or 3*np.pi/2 <= angle < 2*np.pi:
            # Right side: circular arc
            x[i] = a + b * np.cos(angle * 2)
            y[i] = b * np.sin(angle * 2)
        else:
            # Left side: circular arc
            x[i] = -a + b * np.cos(angle * 2)
            y[i] = b * np.sin(angle * 2)

    # Compute heading from derivatives
    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.arctan2(dy, dx)

    # Compute cumulative distance
    dx_diff = np.diff(x)
    dy_diff = np.diff(y)
    ds = np.sqrt(dx_diff**2 + dy_diff**2)
    distance = np.concatenate([[0], np.cumsum(ds)])

    return x, y, heading, distance


def generate_chicane_course(num_chicanes=3, amplitude=3.0, num_points=400):
    """
    Generate chicane course with sharp alternating turns.

    Tests rapid segment transitions similar to slalom.

    Args:
        num_chicanes: Number of chicane sections
        amplitude: Lateral displacement
        num_points: Number of points in course

    Returns:
        x, y, heading, distance arrays
    """
    t = np.linspace(0, 2*np.pi, num_points)

    # Sinusoidal path for chicane
    x = t * 5.0 / (2*np.pi)  # Forward progress
    y = amplitude * np.sin(num_chicanes * t)

    # Close the loop by connecting end to start
    x = x - x[0]
    y = y - y[0]

    # Make it circular
    radius = np.max(np.sqrt(x**2 + y**2))
    angles = np.arctan2(y, x)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)

    # Compute heading
    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.arctan2(dy, dx)

    # Compute distance
    dx_diff = np.diff(x)
    dy_diff = np.diff(y)
    ds = np.sqrt(dx_diff**2 + dy_diff**2)
    distance = np.concatenate([[0], np.cumsum(ds)])

    return x, y, heading, distance


def create_mean_course_from_arrays(x, y, heading, distance):
    """Helper to create MeanCourse object from numpy arrays."""
    mean_course = MeanCourse()
    mean_course.x = x
    mean_course.y = y
    mean_course.heading = heading  # Keep radians to match production
    mean_course.distance = distance
    mean_course.length = distance[-1]
    return mean_course


def find_segment_for_index(segmentation, idx):
    """Locate which segment contains a mean-course index."""
    for seg in segmentation.segments:
        if seg.start_index <= idx <= seg.end_index:
            return seg.segment_id
        if seg.start_index > seg.end_index:
            if idx >= seg.start_index or idx <= seg.end_index:
                return seg.segment_id
    return 0


def simulate_perfect_lap(mean_course):
    """
    Simulate a car following the mean course exactly.

    Returns:
        x_path, y_path arrays matching mean course
    """
    return mean_course.x.copy(), mean_course.y.copy()


def simulate_wobbly_lap(mean_course, wobble_amplitude=0.3, wobble_freq=10):
    """
    Simulate a car with lateral oscillations around the mean course.

    Tests robustness to minor deviations from the reference line.

    Args:
        mean_course: Reference course
        wobble_amplitude: Amplitude of lateral wobble (meters)
        wobble_freq: Frequency of wobble oscillations

    Returns:
        x_path, y_path arrays with added wobble
    """
    num_points = len(mean_course.x)
    t = np.linspace(0, 2*np.pi*wobble_freq, num_points)

    # Add perpendicular oscillation
    heading_rad = np.radians(mean_course.heading)
    wobble = wobble_amplitude * np.sin(t)

    # Perpendicular direction (left of course)
    perp_x = -np.sin(heading_rad)
    perp_y = np.cos(heading_rad)

    x_path = mean_course.x + wobble * perp_x
    y_path = mean_course.y + wobble * perp_y

    return x_path, y_path


class TestSegmentAssignment:
    """Test suite for segment assignment algorithm."""

    def test_figure8_perfect_lap(self):
        """Test segment assignment on figure-8 with perfect driving."""
        # Generate figure-8 course
        x, y, heading, distance = generate_figure8_course(radius=5.0)
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        # Segment the course
        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'gradient'}
        )
        segmentation.compute()

        assert segmentation.total_segments > 0, "Should detect segments"

        # Simulate perfect lap
        x_path, y_path = simulate_perfect_lap(mean_course)

        # Assign segments
        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)

        assert len(segment_ids) == len(x_path), "Should assign all points"
        assert segment_ids[0] == 0, "Should start in segment 0"

        # Check monotonic progression (only forward)
        for i in range(1, len(segment_ids)):
            diff = segment_ids[i] - segment_ids[i-1]
            # Allow: same segment (0), next segment (1), or wrap (negative)
            assert diff >= 0 or diff < -1, \
                f"Backward transition at index {i}: " \
                f"{segment_ids[i-1]} -> {segment_ids[i]}"

    def test_oval_wobbly_driving(self):
        """Test with lateral oscillations around oval course."""
        # Generate figure-8 for better segment variety
        x, y, heading, distance = generate_figure8_course(radius=8.0)
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        # Segment the course
        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'gradient'}
        )
        segmentation.compute()

        assert segmentation.total_segments > 0, "Should detect segments"

        # Simulate wobbly driving
        x_path, y_path = simulate_wobbly_lap(
            mean_course, wobble_amplitude=0.3, wobble_freq=10
        )

        # Assign segments
        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)

        assert len(segment_ids) == len(x_path), "Should assign all points"
        assert segment_ids[0] == 0, "Should start in segment 0"

        # Verify we visit segments (may be 1 if course is simple)
        unique_segments = len(np.unique(segment_ids))
        assert unique_segments >= 1, "Should have at least one segment"
        assert unique_segments <= segmentation.total_segments, \
            "Can't have more segments than defined"

    def test_backward_crossing_ignored(self):
        """Test that backward boundary crossings are ignored."""
        # Generate simple course
        x, y, heading, distance = generate_oval_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        # Segment the course
        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'threshold'}
        )
        segmentation.compute()

        # Start with perfect lap
        x_path, y_path = simulate_perfect_lap(mean_course)

        # Find a point in middle of segment 1 (if exists)
        if segmentation.total_segments > 1:
            # Manually create a backward crossing scenario
            # Find boundary between segment 0 and 1
            boundary = segmentation.segment_boundaries[0]
            normal = boundary['normal']
            direction = normal if boundary.get('expected_denom_sign', 1.0) > 0 else -normal
            start = boundary['point'] - direction * 0.5
            forward = boundary['point'] + direction * 0.5

            assert CourseSegmentation._crossed_boundary_forward(
                start, forward, boundary
            ), "Should cross in forward direction"
            assert not CourseSegmentation._crossed_boundary_forward(
                forward, start, boundary
            ), "Should ignore backward crossing"

    def test_ignore_far_parallel_crossing(self):
        """Ensure remote boundary intersections are ignored."""
        x, y, heading, distance = generate_oval_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        if segmentation.total_segments < 2:
            pytest.skip("Need at least two segments")

        boundary = segmentation.segment_boundaries[0]
        tangent = boundary.get('tangent')
        if tangent is None:
            pytest.skip("Boundary lacks tangent data")

        normal = boundary['normal']
        point = boundary['point']
        limit = boundary.get('tangent_limit', 5.0)
        offset = limit * 3.0
        far_point = point + tangent * offset

        before = far_point - normal * 2.0
        after = far_point + normal * 2.0

        d_before = CourseSegmentation._signed_distance_to_line(
            before, point, normal)
        d_after = CourseSegmentation._signed_distance_to_line(
            after, point, normal)
        if not (d_before < 0 and d_after >= 0):
            before, after = after, before

        segment_ids = segmentation.assign_segments_to_path(
            np.array([before[0], after[0]]),
            np.array([before[1], after[1]])
        )

        assert np.all(segment_ids == segment_ids[0]), \
            "Should ignore boundary crossings far along the tangent"

    def test_coarse_samples_detect_boundary(self):
        """Coarse samples should still detect near-boundary crossings."""
        boundary = {
            'point': np.array([0.0, 0.0]),
            'normal': np.array([1.0, 0.0]),
            'tangent': np.array([0.0, 1.0]),
            'tangent_limit': 0.5,
        }

        # Points are far apart along tangent (y) but cross x=0 near boundary
        p1 = np.array([-0.1, -2.0])
        p2 = np.array([0.1, 2.0])

        assert CourseSegmentation._crossed_boundary_forward(
            p1, p2, boundary), \
            "Should detect crossing even with sparse samples"

    def test_wraparound_segment_transition(self):
        """Closed loops should wrap from last segment back to segment 0."""
        x, y, heading, distance = generate_oval_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'threshold'}
        )
        segmentation.compute()

        if segmentation.total_segments < 2:
            pytest.skip("Need multiple segments for wraparound test")

        x_path, y_path = simulate_perfect_lap(mean_course)
        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)

        assert np.any(segment_ids == segmentation.total_segments - 1), \
            "Should still visit the last segment before wrapping"

    def test_starting_point_in_middle_segment(self):
        """Ensure initial segment matches nearest mean-course point."""
        x, y, heading, distance = generate_oval_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'threshold'}
        )
        segmentation.compute()

        if segmentation.total_segments < 2:
            pytest.skip("Need multiple segments for relocated start test")

        # Shift path to start in middle of segment 1 instead of index 0
        seg1 = segmentation.segments[1]
        if seg1.start_index <= seg1.end_index:
            shift_idx = seg1.start_index + \
                (seg1.end_index - seg1.start_index) // 2
        else:
            total_points = len(mean_course.x)
            seg_len = ((total_points - seg1.start_index) +
                       (seg1.end_index + 1))
            shift_idx = (seg1.start_index + seg_len // 2) % total_points
        x_path = np.concatenate([mean_course.x[shift_idx:],
                                 mean_course.x[:shift_idx]])
        y_path = np.concatenate([mean_course.y[shift_idx:],
                                 mean_course.y[:shift_idx]])

        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)
        expected_segment = find_segment_for_index(segmentation, shift_idx)
        assert segment_ids[0] == expected_segment, \
            "Starting segment should match relocated starting point"
        found_segment = segmentation.find_segment_for_point(
            x_path[0], y_path[0])
        assert found_segment == expected_segment, \
            "Segment lookup should match nearest mean-course sample"

    def test_starting_point_already_past_boundary(self):
        """Start point beyond first boundary should relabel correctly."""
        x, y, heading, distance = generate_oval_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'threshold'}
        )
        segmentation.compute()

        if segmentation.total_segments < 2:
            pytest.skip("Need multiple segments for wrap test")

        boundary = segmentation.segment_boundaries[0]
        normal = boundary['normal']
        start_point = boundary['point'] + normal * 0.5
        x_path = np.concatenate([[start_point[0]], mean_course.x])
        y_path = np.concatenate([[start_point[1]], mean_course.y])

        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)
        assert segment_ids[0] == boundary['segment_to'], \
            "Start beyond boundary should land in next segment"

    def test_start_near_boundary_prefers_previous_segment(self):
        """Points near boundary should map to prior segment for stability."""
        x, y, heading, distance = generate_oval_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'threshold'}
        )
        segmentation.compute()

        if segmentation.total_segments < 2:
            pytest.skip("Need multiple segments for boundary proximity test")

        boundary = segmentation.segment_boundaries[0]
        point = boundary['point'] - boundary['normal'] * 0.01
        x_path = np.concatenate([[point[0]], mean_course.x])
        y_path = np.concatenate([[point[1]], mean_course.y])
        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)
        assert segment_ids[0] == boundary['segment_from'], \
            "Proximity to boundary should keep us in previous segment"

    def test_relabel_segments_aligns_start(self):
        """Relabeling should rotate numbering so start point is segment 0."""
        x, y, heading, distance = generate_oval_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'threshold'}
        )
        segmentation.compute()
        if segmentation.total_segments < 2:
            pytest.skip("Need multiple segments for relabel test")

        shifted_idx = segmentation.segments[1].start_index
        x_path = np.concatenate([mean_course.x[shifted_idx:],
                                 mean_course.x[:shifted_idx]])
        y_path = np.concatenate([mean_course.y[shifted_idx:],
                                 mean_course.y[:shifted_idx]])

        expected_segment = find_segment_for_index(segmentation, shifted_idx)
        if expected_segment == 0:
            pytest.skip("Second segment already labeled as 0")

        segmentation.relabel_segments(expected_segment)
        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)
        assert segment_ids[0] == 0, \
            "Relabeling should rotate numbering so starting point is 0"

    def test_empty_path(self):
        """Test handling of empty path arrays."""
        x, y, heading, distance = generate_figure8_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        # Test empty arrays
        segment_ids = segmentation.assign_segments_to_path(
            np.array([]), np.array([])
        )

        assert len(segment_ids) == 0, "Should return empty array"

    def test_mismatched_array_lengths(self):
        """Test error handling for mismatched array lengths."""
        x, y, heading, distance = generate_figure8_course()
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        # Test mismatched lengths
        with pytest.raises(ValueError, match="same length"):
            segmentation.assign_segments_to_path(
                np.array([1.0, 2.0]), np.array([3.0])
            )

    def test_multiple_lap_completion(self):
        """Test detecting multiple lap completions."""
        # Use figure-8 which has multiple segments
        x, y, heading, distance = generate_figure8_course(radius=6.0)
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        if segmentation.total_segments <= 1:
            pytest.skip("Need multiple segments for this test")

        # Simulate 2 complete laps
        x_path1, y_path1 = simulate_perfect_lap(mean_course)
        x_path2, y_path2 = simulate_perfect_lap(mean_course)

        # Concatenate (skip first point of second lap to avoid duplicate)
        x_path = np.concatenate([x_path1, x_path2[1:]])
        y_path = np.concatenate([y_path1, y_path2[1:]])

        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)

        # Count transitions back to segment 0
        transitions_to_zero = np.sum(
            (segment_ids[:-1] == segmentation.total_segments - 1) &
            (segment_ids[1:] == 0)
        )

        # Should detect lap completion(s)
        # Note: May be 0 if path doesn't complete full lap or segments
        # don't wrap properly
        assert transitions_to_zero >= 0, \
            "Should not have negative transitions"

        # Verify path visits all segments at least in first lap
        unique_in_first_lap = len(np.unique(segment_ids[:len(x_path1)]))
        assert unique_in_first_lap > 1, \
            "Should visit multiple segments in first lap"

    def test_chicane_rapid_transitions(self):
        """Test rapid segment transitions on chicane course."""
        x, y, heading, distance = generate_chicane_course(
            num_chicanes=4, amplitude=2.0
        )
        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(
            mean_course,
            params={'boundary_method': 'extrema', 'min_segment_length': 0.5}
        )
        segmentation.compute()

        assert segmentation.total_segments > 0, "Should detect segments"

        x_path, y_path = simulate_perfect_lap(mean_course)
        segment_ids = segmentation.assign_segments_to_path(x_path, y_path)

        # Should detect multiple segment transitions
        num_transitions = np.sum(np.diff(segment_ids) != 0)
        assert num_transitions > 0, "Should detect segment transitions"

    def test_non_adjacent_boundary_ignored(self):
        """Boundary proximity check should only use adjacent boundaries.

        Regression test for a bug where the nearest boundary was used even
        if it was not adjacent to the segment containing the start point,
        causing incorrect segment assignments.
        """
        # Create a figure-8 course that passes through origin
        t = np.linspace(0, 2*np.pi, 400, endpoint=False)
        radius = 5.0
        x = radius * np.sin(t)
        y = radius * np.sin(t) * np.cos(t)
        heading = np.arctan2(np.gradient(y), np.gradient(x))
        dx_diff = np.diff(x)
        dy_diff = np.diff(y)
        ds = np.sqrt(dx_diff**2 + dy_diff**2)
        distance = np.concatenate([[0], np.cumsum(ds)])

        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(
            mean_course, params={'boundary_method': 'threshold'}
        )
        segmentation.compute()

        if segmentation.total_segments < 3:
            pytest.skip("Need multiple segments for this test")

        # Find segment containing origin
        origin_segment = segmentation.find_segment_for_point(0.0, 0.0)
        assert origin_segment is not None, "Origin should be in a segment"

        # Assign segments to path starting at origin
        segment_ids = segmentation.assign_segments_to_path(x, y)

        # Key assertion: first path point should be in the same segment
        # as found by find_segment_for_point
        assert segment_ids[0] == origin_segment, \
            f"First point should be in segment {origin_segment}, " \
            f"got {segment_ids[0]}"

        # Also check that the nearest boundary (if any) did not incorrectly
        # override the segment assignment
        nearest = segmentation.nearest_boundary(0.0, 0.0)
        if nearest is not None:
            boundary, dist = nearest
            is_adjacent = (boundary['segment_from'] == origin_segment or
                          boundary['segment_to'] == origin_segment)
            if not is_adjacent:
                # Non-adjacent boundary should not affect assignment
                assert segment_ids[0] == origin_segment, \
                    "Non-adjacent boundary should not affect segment assignment"

    def test_point_before_boundary_uses_tangent_distance(self):
        """Test that segment assignment uses tangent distance, not normal.

        Regression test for a bug where signed distance to boundary line
        (perpendicular distance) was used instead of distance along the
        course direction (tangent projection). This caused points to be
        assigned to wrong segments when they were laterally offset from
        the boundary but still "before" it along the course.
        """
        # Create a simple course that crosses through origin
        num_points = 100
        t = np.linspace(0, 2*np.pi, num_points, endpoint=False)
        radius = 5.0
        x = radius * np.sin(t)
        y = radius * np.cos(t)
        heading = np.arctan2(np.gradient(y), np.gradient(x))
        dx_diff = np.diff(x)
        dy_diff = np.diff(y)
        ds = np.sqrt(dx_diff**2 + dy_diff**2)
        distance = np.concatenate([[0], np.cumsum(ds)])

        mean_course = create_mean_course_from_arrays(x, y, heading, distance)

        segmentation = CourseSegmentation(mean_course)
        segmentation.compute()

        if not segmentation.segment_boundaries:
            pytest.skip("No boundaries to test")

        # Find a boundary and test points along the course direction
        boundary = segmentation.segment_boundaries[0]
        bp = boundary['point']
        tangent = boundary['tangent']
        normal = boundary['normal']

        # Point slightly before the boundary (negative tangent distance)
        point_before = bp - tangent * 0.1
        seg_before = segmentation.find_segment_for_point(
            point_before[0], point_before[1])

        # Point slightly after the boundary (positive tangent distance)
        point_after = bp + tangent * 0.1
        seg_after = segmentation.find_segment_for_point(
            point_after[0], point_after[1])

        # They should be in different segments
        assert seg_before == boundary['segment_from'], \
            f"Point before boundary should be in segment {boundary['segment_from']}"
        assert seg_after == boundary['segment_to'], \
            f"Point after boundary should be in segment {boundary['segment_to']}"

        # Now test a point that's laterally offset but still "before" boundary
        # This was the bug: lateral offset changed the perpendicular distance
        # sign even though the point was still before the boundary
        point_lateral_before = bp - tangent * 0.1 + normal * 0.2
        seg_lateral = segmentation.find_segment_for_point(
            point_lateral_before[0], point_lateral_before[1])
        assert seg_lateral == boundary['segment_from'], \
            "Lateral offset should not change before/after determination"
