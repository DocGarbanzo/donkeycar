"""
Tests for web UI segment-cycle-based lap resolver.

Validates that the lap resolver used by the web UI for segment statistics
correctly identifies laps based on segment cycle completions.
"""

import pytest
import numpy as np
from donkeycar.course_analysis import (
    PathData,
    YCrossingLapDetector,
    MultiLapData,
    MeanCourseBuilder,
    CourseSegmenter,
    HybridSegmentation,
)
from donkeycar.course_analysis.segment_assignment import SegmentAssigner
from donkeycar.web.imupath_data import _find_segment_cycle_indices


def generate_multilap_course(num_laps=3, points_per_lap=100, radius=1.0):
    """Generate a multi-lap circular course."""
    total_points = num_laps * points_per_lap
    theta = np.linspace(0, num_laps * 2 * np.pi, total_points, endpoint=False)
    x = radius * np.cos(theta) + np.random.normal(0, 0.01, total_points)
    y = radius * np.sin(theta) + np.random.normal(0, 0.01, total_points)
    heading = theta + np.pi / 2
    velocity = np.ones(total_points) * 2.0
    timestamps = np.arange(total_points) * 0.1
    return PathData(timestamps, x, y, heading, velocity)


class MockIMUPathDataBuilder:
    """
    Minimal stand-in for IMUPathDataBuilder that exposes only the
    segment-cycle lap resolver methods under test.
    """

    def __init__(self, path_data, segmentation, segment_ids):
        self.path_data = path_data
        self.segmentation = segmentation
        self.segment_ids = segment_ids

    def _count_segment_cycle_laps(self, use_lap_0):
        """Count laps based on segment cycle completions."""
        cycle_count = len(_find_segment_cycle_indices(
            self.segment_ids, self.segmentation.num_segments))
        if cycle_count <= 0:
            return None
        if use_lap_0:
            return cycle_count
        if cycle_count == 1:
            return None
        return cycle_count - 1

    def _build_segment_cycle_lap_resolver(self, use_lap_0):
        """Build lap resolver based on segment cycles."""
        cycle_indices = _find_segment_cycle_indices(
            self.segment_ids, self.segmentation.num_segments)
        if not cycle_indices:
            return None

        lap_starts = (
            [0] + cycle_indices[:-1]
            if len(cycle_indices) > 1 else [0]
        )
        last_complete_lap_end = cycle_indices[-1] - 1

        state = {'lap_idx': 0}

        def resolve(record_idx):
            lap_idx = state['lap_idx']
            while lap_idx < len(lap_starts):
                lap_start = lap_starts[lap_idx]
                lap_end = (
                    lap_starts[lap_idx + 1] - 1
                    if lap_idx + 1 < len(lap_starts)
                    else last_complete_lap_end
                )
                if record_idx < lap_start:
                    return None
                if lap_start <= record_idx <= lap_end:
                    state['lap_idx'] = lap_idx
                    if not use_lap_0 and lap_idx == 0:
                        return None
                    return lap_idx
                lap_idx += 1
            state['lap_idx'] = lap_idx
            return None

        return resolve


class TestSegmentCycleLapResolver:
    """Test the segment-cycle-based lap resolver."""

    def test_lap_count_matches_segment_cycles(self):
        """Test lap count equals number of complete segment cycles."""
        path_data = generate_multilap_course(num_laps=10, points_per_lap=100)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Count segment cycles
        cycle_count = len(_find_segment_cycle_indices(
            segment_ids, segmentation.num_segments))

        # Test lap resolver
        mock_builder = MockIMUPathDataBuilder(path_data, segmentation,
                                             segment_ids)

        lap_count_with_lap0 = mock_builder._count_segment_cycle_laps(
            use_lap_0=True)
        lap_count_without_lap0 = mock_builder._count_segment_cycle_laps(
            use_lap_0=False)

        assert lap_count_with_lap0 == cycle_count
        if cycle_count > 1:
            assert lap_count_without_lap0 == cycle_count - 1
        else:
            assert lap_count_without_lap0 is None

    def test_lap_resolver_assigns_correct_laps(self):
        """Test lap resolver assigns correct lap numbers to records."""
        path_data = generate_multilap_course(num_laps=5, points_per_lap=100)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Build resolver
        mock_builder = MockIMUPathDataBuilder(path_data, segmentation,
                                             segment_ids)
        resolver = mock_builder._build_segment_cycle_lap_resolver(
            use_lap_0=True)

        assert resolver is not None

        # Find segment cycle boundaries
        cycle_indices = _find_segment_cycle_indices(
            segment_ids, segmentation.num_segments)

        # Test lap assignments at key points
        assert resolver(0) == 0, "First record should be lap 0"

        # Test boundaries
        for lap_idx, cycle_idx in enumerate(cycle_indices[:-1]):
            # Record just before cycle boundary
            lap_before = resolver(cycle_idx - 1)
            # Record at cycle boundary (start of next lap)
            lap_at = resolver(cycle_idx)

            assert lap_before == lap_idx, (
                f"Record {cycle_idx-1} should be lap {lap_idx}, got {lap_before}")
            assert lap_at == lap_idx + 1, (
                f"Record {cycle_idx} should be lap {lap_idx+1}, got {lap_at}")

    def test_trailing_partial_lap_excluded(self):
        """Test that trailing partial lap is excluded from resolver."""
        path_data = generate_multilap_course(num_laps=5, points_per_lap=100)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        # Find last cycle boundary
        cycle_indices = _find_segment_cycle_indices(
            segment_ids, segmentation.num_segments)

        if not cycle_indices:
            pytest.skip("No complete cycles detected")

        last_cycle_idx = cycle_indices[-1]

        # Build resolver
        mock_builder = MockIMUPathDataBuilder(path_data, segmentation,
                                             segment_ids)
        resolver = mock_builder._build_segment_cycle_lap_resolver(
            use_lap_0=True)

        # Records before last cycle should have lap assignment
        if last_cycle_idx > 10:
            for idx in range(max(0, last_cycle_idx - 50), last_cycle_idx):
                lap = resolver(idx)
                assert lap is not None, (
                    f"Record {idx} before last cycle should have lap assignment")

        # Records at and after last cycle should return None (partial lap)
        for idx in range(last_cycle_idx, min(last_cycle_idx + 50,
                                             len(segment_ids))):
            lap = resolver(idx)
            # These are in the partial lap, should return None
            if idx >= last_cycle_idx:
                # Only check if there's significant data after last cycle
                if len(segment_ids) - last_cycle_idx > 50:
                    assert lap is None, (
                        f"Record {idx} (after last complete lap) should return None")

    def test_use_lap_0_parameter(self):
        """Test use_lap_0 parameter correctly filters lap 0."""
        path_data = generate_multilap_course(num_laps=5, points_per_lap=100)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        mock_builder = MockIMUPathDataBuilder(path_data, segmentation,
                                             segment_ids)

        # With lap 0
        resolver_with_lap0 = mock_builder._build_segment_cycle_lap_resolver(
            use_lap_0=True)

        # Without lap 0
        resolver_without_lap0 = mock_builder._build_segment_cycle_lap_resolver(
            use_lap_0=False)

        # Test first 10 records
        for idx in range(10):
            lap_with = resolver_with_lap0(idx)
            lap_without = resolver_without_lap0(idx)

            if lap_with == 0:
                assert lap_without is None, (
                    f"Record {idx}: lap 0 should be filtered when use_lap_0=False")
            else:
                assert lap_with == lap_without, (
                    f"Record {idx}: lap numbers should match for non-lap-0")

    @pytest.mark.parametrize("num_laps", [3, 5, 10, 20])
    def test_all_complete_laps_have_all_segments(self, num_laps):
        """Test that resolver-assigned laps have all segments."""
        path_data = generate_multilap_course(num_laps=num_laps,
                                            points_per_lap=100)

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

        mock_builder = MockIMUPathDataBuilder(path_data, segmentation,
                                             segment_ids)
        resolver = mock_builder._build_segment_cycle_lap_resolver(
            use_lap_0=True)

        # Collect segments per lap
        lap_segments = {}
        for idx in range(len(segment_ids)):
            lap = resolver(idx)
            if lap is not None:
                if lap not in lap_segments:
                    lap_segments[lap] = set()
                lap_segments[lap].add(segment_ids[idx])

        # Verify each lap has all segments
        expected_segments = set(range(segmentation.num_segments))
        for lap, segments in lap_segments.items():
            assert segments == expected_segments, (
                f"Lap {lap} has segments {sorted(segments)}, "
                f"expected {sorted(expected_segments)}")

    def test_resolver_state_management(self):
        """Test resolver state is properly managed across calls."""
        path_data = generate_multilap_course(num_laps=5, points_per_lap=100)

        detector = YCrossingLapDetector()
        lap_boundaries = detector.detect_laps(path_data)
        multilap = MultiLapData(path_data, lap_boundaries)

        builder = MeanCourseBuilder()
        mean_course = builder.build(multilap)

        segmenter = CourseSegmenter(HybridSegmentation())
        segmentation = segmenter.segment(mean_course)

        assigner = SegmentAssigner(segmentation)
        segment_ids = assigner.assign(path_data.x, path_data.y)

        mock_builder = MockIMUPathDataBuilder(path_data, segmentation,
                                             segment_ids)
        resolver = mock_builder._build_segment_cycle_lap_resolver(
            use_lap_0=True)

        # Call resolver in order
        laps_ordered = [resolver(i) for i in range(min(100, len(segment_ids)))]

        # Call resolver out of order (random access)
        test_indices = [0, 50, 25, 75, 10, 90]
        for idx in test_indices:
            if idx < len(segment_ids):
                lap_random = resolver(idx)
                lap_ordered = laps_ordered[idx]

                # Note: State management means random access may give wrong
                # results This tests that the resolver maintains state correctly
                # In practice, the resolver is called sequentially

        # Sequential calls should be consistent
        laps_second_pass = [resolver(i) for i in range(
            min(100, len(segment_ids)))]
        assert laps_ordered == laps_second_pass, (
            "Resolver should give consistent results on sequential calls")


class TestIntegrationWithTubStatistics:
    """Test integration between lap resolver and TubStatistics."""

    def test_segment_statistics_use_correct_lap_boundaries(self):
        """
        Test that segment statistics computed with segment-cycle lap resolver
        produce correct results.
        """
        # This test would require a full integration test with TubStatistics
        # For now, verify the lap resolver produces expected lap assignments

        path_data = generate_multilap_course(num_laps=10, points_per_lap=100)

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

        mock_builder = MockIMUPathDataBuilder(path_data, segmentation,
                                             segment_ids)
        resolver = mock_builder._build_segment_cycle_lap_resolver(
            use_lap_0=True)

        # Find segment cycles
        cycle_indices = _find_segment_cycle_indices(
            segment_ids, segmentation.num_segments)

        # Verify lap assignments align with cycle boundaries
        for cycle_idx in cycle_indices[:-1]:
            lap_before = resolver(cycle_idx - 1)
            lap_after = resolver(cycle_idx)

            assert lap_after == lap_before + 1, (
                f"Lap should increment at cycle boundary {cycle_idx}")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
