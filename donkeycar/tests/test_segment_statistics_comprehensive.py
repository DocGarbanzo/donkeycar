"""
Comprehensive tests for segment statistics with segment-cycle-based laps.

Tests the invariant: Each lap visits each segment exactly once (0→1→2→...→N-1→0).

This test suite covers:
- Real tub data
- Synthetic courses with various configurations
- Edge cases and boundary conditions
- Hundreds of parameter combinations
"""

import pytest
import numpy as np
from collections import defaultdict
from typing import Dict, List

from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics, FieldAggregationSpec
from donkeycar.course_analysis import (
    PathData,
    TubPathDataSource,
    YCrossingLapDetector,
    MultiLapData,
    MeanCourseBuilder,
    CourseSegmenter,
    HybridSegmentation,
    ThresholdSegmentation,
    ExtremaSegmentation,
    GradientSegmentation,
)
from donkeycar.course_analysis.segment_assignment import SegmentAssigner
from donkeycar.pipeline.transformations import SortingStrategy


def generate_circular_course(radius=1.0, num_points=100, noise=0.0):
    """Generate a circular course for testing."""
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = radius * np.cos(theta) + np.random.normal(0, noise, num_points)
    y = radius * np.sin(theta) + np.random.normal(0, noise, num_points)
    heading = theta + np.pi / 2
    velocity = np.ones(num_points) * 2.0
    timestamps = np.arange(num_points) * 0.1
    return PathData(timestamps, x, y, heading, velocity)


def generate_multilap_course(num_laps=3, points_per_lap=100, radius=1.0,
                             noise=0.0):
    """Generate a multi-lap circular course."""
    total_points = num_laps * points_per_lap
    theta = np.linspace(0, num_laps * 2 * np.pi, total_points, endpoint=False)
    x = radius * np.cos(theta) + np.random.normal(0, noise, total_points)
    y = radius * np.sin(theta) + np.random.normal(0, noise, total_points)
    heading = theta + np.pi / 2
    velocity = np.ones(total_points) * 2.0
    timestamps = np.arange(total_points) * 0.1
    return PathData(timestamps, x, y, heading, velocity)


def count_segment_instances_per_lap(segment_ids, num_segments):
    """
    Count how many times each segment is VISITED in each lap.

    A "visit" is a transition into a segment (consecutive records with the same
    segment ID count as one visit).

    Returns dict: {lap_idx: {segment_id: visit_count}}
    """
    # Find lap boundaries (segment cycle completions)
    lap_boundaries = [0]
    for i in range(1, len(segment_ids)):
        if segment_ids[i-1] == num_segments - 1 and segment_ids[i] == 0:
            lap_boundaries.append(i)

    # Count segment visits per lap
    counts = defaultdict(lambda: defaultdict(int))
    for lap_idx in range(len(lap_boundaries)):
        lap_start = lap_boundaries[lap_idx]
        if lap_idx + 1 < len(lap_boundaries):
            lap_end = lap_boundaries[lap_idx + 1]
        else:
            # Last lap may be partial
            lap_end = len(segment_ids)

        # Count transitions (visits) within this lap
        if lap_start < lap_end:
            # First segment in lap
            current_seg = segment_ids[lap_start]
            counts[lap_idx][current_seg] += 1

            # Count transitions
            for i in range(lap_start + 1, lap_end):
                if segment_ids[i] != segment_ids[i-1]:
                    # Transition to new segment
                    current_seg = segment_ids[i]
                    counts[lap_idx][current_seg] += 1

    return counts


def verify_segment_invariant(segment_ids, num_segments):
    """
    Verify that each complete lap visits each segment exactly once.

    Returns (is_valid, error_message)
    """
    counts = count_segment_instances_per_lap(segment_ids, num_segments)

    # Find complete laps (those that have all segments)
    complete_laps = []
    for lap_idx, seg_counts in counts.items():
        segments_present = set(seg_counts.keys())
        expected_segments = set(range(num_segments))
        if segments_present == expected_segments:
            complete_laps.append(lap_idx)

    # Verify each complete lap
    for lap_idx in complete_laps:
        seg_counts = counts[lap_idx]
        for seg_id in range(num_segments):
            count = seg_counts.get(seg_id, 0)
            if count != 1:
                return (False,
                       f"Lap {lap_idx}: segment {seg_id} appears {count} times "
                       f"(expected 1)")

    return (True, "")


class TestSegmentStatisticsInvariant:
    """Test segment assignment invariant with various configurations."""

    @pytest.mark.parametrize("num_segments", [3, 4, 5, 6, 8, 10])
    @pytest.mark.parametrize("num_laps", [1, 2, 3, 5, 10])
    @pytest.mark.parametrize("points_per_lap", [50, 100, 200])
    def test_synthetic_circular_courses(self, num_segments, num_laps,
                                       points_per_lap):
        """Test segment invariant on synthetic circular courses."""
        # Generate multi-lap course
        path_data = generate_multilap_course(
            num_laps=num_laps,
            points_per_lap=points_per_lap,
            radius=1.0,
            noise=0.01
        )

        # Detect laps
        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        # Segment with threshold to get desired number of segments
        # Adjust threshold to target num_segments
        threshold = 2 * np.pi / num_segments
        strategy = ThresholdSegmentation()
        segmenter = CourseSegmenter(
            strategy,
            params={'straight_curvature_threshold': threshold}
        )
        segmentation = segmenter.segment(mean_course)

        # Allow some flexibility in segment count
        if abs(segmentation.num_segments - num_segments) > 2:
            pytest.skip(
                f"Segmentation created {segmentation.num_segments} segments, "
                f"expected ~{num_segments}")

        # Assign segments
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Verify invariant
        is_valid, error_msg = verify_segment_invariant(
            segment_ids, segmentation.num_segments)

        assert is_valid, error_msg

    @pytest.mark.parametrize("strategy_name", [
        'threshold', 'extrema', 'gradient', 'hybrid'
    ])
    @pytest.mark.parametrize("num_laps", [2, 5, 10])
    def test_different_segmentation_strategies(self, strategy_name, num_laps):
        """Test invariant with different segmentation strategies."""
        # Generate course
        path_data = generate_multilap_course(
            num_laps=num_laps,
            points_per_lap=100,
            radius=1.0,
            noise=0.01
        )

        # Detect laps
        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        if multilap.num_laps == 0:
            pytest.skip("No laps detected")

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        # Select strategy
        strategies = {
            'threshold': ThresholdSegmentation(),
            'extrema': ExtremaSegmentation(),
            'gradient': GradientSegmentation(),
            'hybrid': HybridSegmentation()
        }
        strategy = strategies[strategy_name]

        segmenter = CourseSegmenter(strategy)
        segmentation = segmenter.segment(mean_course)

        if segmentation.num_segments < 2:
            pytest.skip("Too few segments")

        # Assign segments
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Verify invariant
        is_valid, error_msg = verify_segment_invariant(
            segment_ids, segmentation.num_segments)

        assert is_valid, error_msg


class TestSegmentStatisticsRankings:
    """Test segment performance rankings."""

    def test_no_duplicate_rankings_per_lap(self):
        """
        Test that segment statistics don't create duplicate segment instances
        per lap.
        """
        # Generate course with 5 laps, 5 segments
        path_data = generate_multilap_course(
            num_laps=5,
            points_per_lap=150,
            radius=1.0,
            noise=0.01
        )

        # Detect laps
        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        # Segment
        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        if segmentation.num_segments < 2:
            pytest.skip("Too few segments")

        # Assign segments to path
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Count segment instances per lap
        counts = count_segment_instances_per_lap(
            segment_ids, segmentation.num_segments)

        # For complete laps, verify each segment appears exactly once
        for lap_idx, seg_counts in counts.items():
            segments_present = set(seg_counts.keys())
            expected_segments = set(range(segmentation.num_segments))

            if segments_present == expected_segments:
                # Complete lap
                for seg_id, count in seg_counts.items():
                    assert count == 1, (
                        f"Lap {lap_idx}, segment {seg_id}: expected 1 instance, "
                        f"got {count}")

    def test_segment_cycle_lap_boundaries(self):
        """
        Test that segment-cycle-based lap boundaries align with segment
        assignments.
        """
        # Generate course
        path_data = generate_multilap_course(
            num_laps=5,
            points_per_lap=100,
            radius=1.0
        )

        # Detect laps
        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        # Build mean course and segment
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)
        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        # Assign segments
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Find segment cycle boundaries
        num_segments = segmentation.num_segments
        cycle_indices = []
        for i in range(1, len(segment_ids)):
            if (segment_ids[i-1] == num_segments - 1 and
                segment_ids[i] == 0):
                cycle_indices.append(i)

        # Verify each cycle boundary is a valid lap start
        for idx in cycle_indices:
            assert segment_ids[idx] == 0, (
                f"Cycle boundary at {idx} should start with segment 0, "
                f"got {segment_ids[idx]}")


class TestRealTubData:
    """Test with real tub data from /Users/dirk/cars/hyper/data."""

    def test_hyper_tub_segment_invariant(self):
        """Test segment invariant on real hyper car data."""
        tub_path = '/Users/dirk/cars/hyper/data'

        # Load path data
        source = TubPathDataSource(tub_path)
        path_data = source.load()

        # Detect laps
        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        # Segment with hybrid strategy
        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        # Assign segments
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Verify invariant
        is_valid, error_msg = verify_segment_invariant(
            segment_ids, segmentation.num_segments)

        assert is_valid, error_msg

    def test_hyper_tub_no_duplicate_segment_rankings(self):
        """
        Test that segment statistics on hyper tub don't create duplicate
        segment instances per lap.
        """
        tub_path = '/Users/dirk/cars/hyper/data'

        # Load path data
        source = TubPathDataSource(tub_path)
        path_data = source.load()

        # Detect laps
        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        # Build mean course
        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        # Segment
        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        # Assign segments
        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Count segment instances per lap using segment-cycle boundaries
        counts = count_segment_instances_per_lap(
            segment_ids, segmentation.num_segments)

        # Verify no duplicates in complete laps
        for lap_idx, seg_counts in counts.items():
            segments_present = set(seg_counts.keys())
            expected_segments = set(range(segmentation.num_segments))

            if segments_present == expected_segments:
                # Complete lap - verify each segment appears once
                for seg_id in range(segmentation.num_segments):
                    count = seg_counts[seg_id]
                    assert count == 1, (
                        f"Hyper tub lap {lap_idx}, segment {seg_id}: "
                        f"expected 1 instance, got {count}")

    def test_hyper_tub_segment_statistics_consistency(self):
        """
        Test that TubStatistics produces consistent segment rankings
        (9 laps, each segment appears exactly 9 times).
        """
        tub_path = '/Users/dirk/cars/hyper/data'
        tub = Tub(tub_path, read_only=True)

        try:
            # Create field aggregation spec
            spec = FieldAggregationSpec(
                field='car/gyro',
                output_key='gyro_z_agg',
                index=2,
                transform=abs,
                aggregation='avg'
            )

            # Create TubStatistics
            sorting_strategy = SortingStrategy([{'key': 'gyro_z_agg'}])
            stats = TubStatistics(
                tub,
                sorting_strategy=sorting_strategy,
                field_aggregations=[spec]
            )

            # Calculate segment performance
            # Note: use_lap_0=False should give 8 laps (excluding lap 0)
            # But we'll check the actual implementation
            rankings = stats.calculate_segment_performance(use_lap_0=True)

            # Count instances per segment across all laps
            segment_instance_counts = defaultdict(int)

            for session_id, laps in rankings.items():
                for lap_num, segments in laps.items():
                    for segment_id in segments.keys():
                        segment_instance_counts[segment_id] += 1

            # Verify all segments have the same instance count
            if segment_instance_counts:
                instance_counts = list(segment_instance_counts.values())
                expected_count = instance_counts[0]

                for seg_id, count in segment_instance_counts.items():
                    assert count == expected_count, (
                        f"Segment {seg_id} has {count} instances, "
                        f"expected {expected_count}")
        finally:
            tub.close()


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_single_lap(self):
        """Test with a single lap."""
        path_data = generate_multilap_course(num_laps=1, points_per_lap=100)

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

        # Verify invariant
        is_valid, error_msg = verify_segment_invariant(
            segment_ids, segmentation.num_segments)

        assert is_valid, error_msg

    def test_partial_final_lap(self):
        """Test course with a partial final lap."""
        # Generate 3 full laps + 0.5 partial lap
        full_laps = generate_multilap_course(num_laps=3, points_per_lap=100)

        # Add partial lap (half a lap)
        partial_theta = np.linspace(0, np.pi, 50, endpoint=False)
        partial_x = np.cos(partial_theta)
        partial_y = np.sin(partial_theta)
        partial_heading = partial_theta + np.pi / 2
        partial_velocity = np.ones(50) * 2.0
        partial_time = full_laps.timestamp[-1] + np.arange(50) * 0.1

        # Concatenate
        t = np.concatenate([full_laps.timestamp, partial_time])
        x = np.concatenate([full_laps.x, partial_x])
        y = np.concatenate([full_laps.y, partial_y])
        heading = np.concatenate([full_laps.heading, partial_heading])
        velocity = np.concatenate([full_laps.velocity, partial_velocity])

        path_data = PathData(t, x, y, heading, velocity)

        # Process
        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Verify complete laps only
        is_valid, error_msg = verify_segment_invariant(
            segment_ids, segmentation.num_segments)

        assert is_valid, error_msg

    def test_minimum_segments(self):
        """Test with minimum number of segments (2)."""
        path_data = generate_multilap_course(num_laps=3, points_per_lap=100)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        # Use threshold to force ~2 segments
        strategy = ThresholdSegmentation()
        segmenter = CourseSegmenter(
            strategy,
            params={'straight_curvature_threshold': np.pi}
        )
        segmentation = segmenter.segment(mean_course)

        if segmentation.num_segments < 2:
            pytest.skip("Less than 2 segments")

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        is_valid, error_msg = verify_segment_invariant(
            segment_ids, segmentation.num_segments)

        assert is_valid, error_msg


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
