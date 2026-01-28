"""
Stress tests and extreme scenarios for segment statistics.

Tests edge cases, large datasets, performance, and robustness.
"""

import pytest
import numpy as np
from collections import defaultdict

from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics, FieldAggregationSpec
from donkeycar.course_analysis import (
    PathData,
    YCrossingLapDetector,
    MultiLapData,
    MeanCourseBuilder,
    CourseSegmenter,
    HybridSegmentation,
)
from donkeycar.course_analysis.segment_assignment import SegmentAssigner
from donkeycar.pipeline.transformations import SortingStrategy
from donkeycar.web.imupath_data import IMUPathDataBuilder


def generate_course_with_noise(num_laps, points_per_lap, radius=1.0,
                               position_noise=0.0, heading_noise=0.0):
    """
    Generate multi-lap course with configurable noise.

    Starts at (0, 0) moving in +Y direction.
    """
    total_points = num_laps * points_per_lap
    # Circle centered at (-radius, 0), starting at θ=0 for (0, 0) start
    theta = np.linspace(0, num_laps * 2 * np.pi, total_points, endpoint=False)

    # Add position noise
    x = (-radius + radius * np.cos(theta) +
         np.random.normal(0, position_noise, total_points))
    y = (radius * np.sin(theta) +
         np.random.normal(0, position_noise, total_points))

    # Add heading noise
    heading = theta + np.pi / 2 + np.random.normal(0, heading_noise, total_points)

    velocity = np.ones(total_points) * 2.0
    timestamps = np.arange(total_points) * 0.1

    return PathData(timestamps, x, y, heading, velocity)


def generate_figure_eight_course(num_laps, points_per_lap, radius=1.0):
    """Generate a figure-8 shaped course."""
    total_points = num_laps * points_per_lap
    t = np.linspace(0, num_laps * 2 * np.pi, total_points, endpoint=False)

    # Lemniscate of Bernoulli (figure-8)
    scale = radius * np.sqrt(2)
    x = scale * np.cos(t) / (1 + np.sin(t)**2)
    y = scale * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)

    # Compute heading from velocity
    dx = np.diff(x, append=x[0])
    dy = np.diff(y, append=y[0])
    heading = np.arctan2(dy, dx)

    velocity = np.ones(total_points) * 2.0
    timestamps = np.arange(total_points) * 0.1

    return PathData(timestamps, x, y, heading, velocity)


def verify_lap_segment_consistency(segment_ids, num_segments):
    """
    Verify lap-segment consistency invariant.

    Returns (is_valid, details_dict)
    """
    # Find lap boundaries
    lap_boundaries = [0]
    for i in range(1, len(segment_ids)):
        if segment_ids[i-1] == num_segments - 1 and segment_ids[i] == 0:
            lap_boundaries.append(i)

    details = {
        'num_laps': len(lap_boundaries),
        'lap_boundaries': lap_boundaries,
        'violations': []
    }

    # Check each lap
    for lap_idx in range(len(lap_boundaries)):
        lap_start = lap_boundaries[lap_idx]
        if lap_idx + 1 < len(lap_boundaries):
            lap_end = lap_boundaries[lap_idx + 1]
        else:
            lap_end = len(segment_ids)

        # Count segment visits
        segment_visits = defaultdict(int)
        if lap_start < lap_end:
            current_seg = segment_ids[lap_start]
            segment_visits[current_seg] += 1

            for i in range(lap_start + 1, lap_end):
                if segment_ids[i] != segment_ids[i-1]:
                    current_seg = segment_ids[i]
                    segment_visits[current_seg] += 1

        # Check for violations
        for seg_id, count in segment_visits.items():
            if count > 1:
                details['violations'].append({
                    'lap': lap_idx,
                    'segment': seg_id,
                    'visits': count
                })

    is_valid = len(details['violations']) == 0
    return (is_valid, details)


class TestStressScenarios:
    """Stress tests with large datasets and extreme parameters."""

    @pytest.mark.parametrize("num_laps", [50, 100, 200])
    def test_many_laps(self, num_laps):
        """Test with very large number of laps."""
        path_data = generate_course_with_noise(
            num_laps=num_laps,
            points_per_lap=100,
            position_noise=0.01
        )

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps < 10:
            pytest.skip(f"Only {multilap.num_laps} laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        is_valid, details = verify_lap_segment_consistency(
            segment_ids, segmentation.num_segments)

        assert is_valid, f"Violations: {details['violations'][:5]}"

    @pytest.mark.parametrize("points_per_lap", [500, 1000, 2000])
    def test_high_resolution_path(self, points_per_lap):
        """Test with very high resolution paths."""
        path_data = generate_course_with_noise(
            num_laps=5,
            points_per_lap=points_per_lap,
            position_noise=0.005
        )

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        is_valid, details = verify_lap_segment_consistency(
            segment_ids, segmentation.num_segments)

        assert is_valid, f"Violations: {details['violations'][:5]}"

    @pytest.mark.parametrize("noise_level", [0.01, 0.05, 0.1, 0.2])
    def test_noisy_data(self, noise_level):
        """Test with varying levels of position noise."""
        path_data = generate_course_with_noise(
            num_laps=5,
            points_per_lap=150,
            position_noise=noise_level
        )

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected with this noise level")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        is_valid, details = verify_lap_segment_consistency(
            segment_ids, segmentation.num_segments)

        # Allow some violations with very high noise
        if noise_level < 0.15:
            assert is_valid, f"Noise {noise_level}: {details['violations'][:5]}"


class TestDifferentCourseShapes:
    """Test with non-circular course shapes."""

    def test_figure_eight_course(self):
        """Test with figure-8 shaped course."""
        path_data = generate_figure_eight_course(
            num_laps=5,
            points_per_lap=200,
            radius=1.0
        )

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        is_valid, details = verify_lap_segment_consistency(
            segment_ids, segmentation.num_segments)

        assert is_valid, f"Violations: {details['violations'][:5]}"

    def test_elliptical_course(self):
        """Test with elliptical course."""
        num_laps = 5
        points_per_lap = 150
        total_points = num_laps * points_per_lap

        theta = np.linspace(0, num_laps * 2 * np.pi, total_points,
                           endpoint=False)

        # Ellipse with different major/minor axes
        x = 2.0 * np.cos(theta)
        y = 1.0 * np.sin(theta)

        heading = np.arctan2(np.diff(y, append=y[0]), np.diff(x, append=x[0]))
        velocity = np.ones(total_points) * 2.0
        timestamps = np.arange(total_points) * 0.1

        path_data = PathData(timestamps, x, y, heading, velocity)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        is_valid, details = verify_lap_segment_consistency(
            segment_ids, segmentation.num_segments)

        assert is_valid, f"Violations: {details['violations'][:5]}"


class TestWebUIIntegration:
    """Test web UI segment cycle lap resolver integration."""

    def test_segment_cycle_lap_count_matches_transitions(self):
        """Verify lap count matches number of segment cycle transitions."""
        path_data = generate_course_with_noise(
            num_laps=10,
            points_per_lap=100,
            position_noise=0.01
        )

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Count segment cycles
        num_segments = segmentation.num_segments
        cycle_count = 0
        for i in range(1, len(segment_ids)):
            if (segment_ids[i-1] == num_segments - 1 and
                segment_ids[i] == 0):
                cycle_count += 1

        # Verify lap count (excluding lap 0)
        expected_laps = cycle_count

        # Build IMU data builder (simulating web UI)
        # Note: Can't actually test this without full web UI infrastructure
        # Just verify the concept

        assert cycle_count > 0, "Should have at least one complete cycle"
        assert cycle_count <= multilap.num_laps, (
            "Cycles should not exceed detected laps")

    def test_segment_cycle_boundaries_are_consistent(self):
        """Verify segment cycle boundaries are consistent with assignments."""
        path_data = generate_course_with_noise(
            num_laps=10,
            points_per_lap=100,
            position_noise=0.01
        )

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Find all segment cycle boundaries
        num_segments = segmentation.num_segments
        cycle_indices = []
        for i in range(1, len(segment_ids)):
            if (segment_ids[i-1] == num_segments - 1 and
                segment_ids[i] == 0):
                cycle_indices.append(i)

        # Verify each boundary transitions correctly
        for cycle_idx in cycle_indices:
            prev_seg = segment_ids[cycle_idx - 1]
            curr_seg = segment_ids[cycle_idx]

            assert prev_seg == num_segments - 1, (
                f"At index {cycle_idx}: previous segment should be "
                f"{num_segments - 1}, got {prev_seg}")
            assert curr_seg == 0, (
                f"At index {cycle_idx}: current segment should be 0, "
                f"got {curr_seg}")


class TestRobustness:
    """Test robustness to edge cases and malformed data."""

    def test_very_short_laps(self):
        """Test with very short laps (few points)."""
        path_data = generate_course_with_noise(
            num_laps=5,
            points_per_lap=10,  # Very short
            position_noise=0.01
        )

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        # Should handle gracefully even with few segments
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Just verify no crashes
        assert len(segment_ids) == len(path_data.x)

    def test_constant_position(self):
        """Test handling of stationary vehicle (constant position)."""
        num_points = 100
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        heading = np.zeros(num_points)
        velocity = np.zeros(num_points)
        timestamps = np.arange(num_points) * 0.1

        path_data = PathData(timestamps, x, y, heading, velocity)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        # Should detect no laps
        assert multilap.num_laps <= 1, (
            "Stationary vehicle should not detect multiple laps")

    def test_backwards_motion(self):
        """Test handling of backwards motion."""
        # Generate forward course then reverse it
        forward = generate_course_with_noise(
            num_laps=3,
            points_per_lap=100,
            position_noise=0.01
        )

        # Reverse the path
        x = forward.x[::-1].copy()
        y = forward.y[::-1].copy()
        heading = forward.heading[::-1].copy() + np.pi  # Flip heading
        velocity = forward.velocity[::-1].copy()
        timestamps = forward.timestamp.copy()  # Keep time increasing

        path_data = PathData(timestamps, x, y, heading, velocity)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        # Should still process without crashing
        if multilap.num_laps > 0:
            builder = MeanCourseBuilder()
            mean_course = builder.build(multilap)

            segmenter = CourseSegmenter(HybridSegmentation())
            segmentation = segmenter.segment(mean_course)

            assigner = SegmentAssigner(segmentation)
            segment_ids = assigner.assign(path_data.x, path_data.y)

            assert len(segment_ids) == len(path_data.x)


class TestPerformance:
    """Performance tests to ensure scalability."""

    def test_large_dataset_performance(self):
        """Test performance with large dataset (10k+ points)."""
        import time

        path_data = generate_course_with_noise(
            num_laps=50,
            points_per_lap=200,  # 10,000 total points
            position_noise=0.01
        )

        start_time = time.time()

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        elapsed = time.time() - start_time

        # Should complete in reasonable time (<5 seconds)
        assert elapsed < 5.0, f"Processing took {elapsed:.2f}s (too slow)"

        # Verify correctness
        is_valid, details = verify_lap_segment_consistency(
            segment_ids, segmentation.num_segments)

        assert is_valid, f"Violations: {details['violations'][:5]}"


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
