"""
Integration test for segment initial detection bug.

Reproduces the UI bug where segment assignment gives wrong results.
This test actually checks CORRECTNESS, not just validity.
"""
import pytest
import numpy as np
from donkeycar.course_analysis import (
    CourseSegmenter, ThresholdSegmentation, ExtremaSegmentation,
    SegmentAssigner
)
from donkeycar.tests.course_test_fixtures import create_figure8_course


def test_initial_segment_matches_mean_course_segment():
    """
    CRITICAL REGRESSION TEST: Segment assignment must be CORRECT.

    When assigning a point that's ON the mean course at a known index,
    the assigned segment must match the segment that contains that index.

    This test FAILS with the current buggy implementation!
    """
    mean_course = create_figure8_course(radius=5.0, num_points=400)

    segmenter = CourseSegmenter(
        ThresholdSegmentation(),
        params={'min_segment_length': 0.3}
    )
    segmentation = segmenter.segment(mean_course)

    assigner = SegmentAssigner(segmentation)

    # Test multiple points along the course
    test_indices = [0, 50, 100, 150, 200, 250, 300, 350]

    failures = []

    for idx in test_indices:
        # Get point on mean course
        test_x = mean_course.x[idx]
        test_y = mean_course.y[idx]

        # Find which segment this index belongs to (ground truth)
        expected_segment = None
        for seg in segmentation.segments:
            if seg.start_index <= idx <= seg.end_index:
                expected_segment = seg.segment_id
                break

        # Handle wrap-around case
        if expected_segment is None:
            for seg in segmentation.segments:
                if seg.start_index > seg.end_index:  # Wrap-around segment
                    if idx >= seg.start_index or idx <= seg.end_index:
                        expected_segment = seg.segment_id
                        break

        # Assign using algorithm
        detected_segment = assigner.assign(
            np.array([test_x]),
            np.array([test_y])
        )[0]

        if detected_segment != expected_segment:
            failures.append({
                'index': idx,
                'position': (test_x, test_y),
                'expected': expected_segment,
                'detected': detected_segment
            })

    if failures:
        msg = "Segment assignment failures:\n"
        for f in failures:
            msg += (f"  Index {f['index']}: expected segment {f['expected']}, "
                   f"got {f['detected']} at position ({f['position'][0]:.2f}, {f['position'][1]:.2f})\n")
        assert False, msg


def test_no_duplicate_boundaries():
    """
    Test that SegmentAssigner uses ONLY segmentation boundaries.

    REGRESSION TEST: Previously SegmentAssigner created its own duplicate
    boundary set (self.boundaries), causing bugs. Now it should only use
    self.segmentation.segment_boundaries - no duplication!
    """
    mean_course = create_figure8_course(radius=5.0)

    segmenter = CourseSegmenter(
        ThresholdSegmentation(),
        params={'min_segment_length': 0.3}
    )
    segmentation = segmenter.segment(mean_course)

    assigner = SegmentAssigner(segmentation)

    # CRITICAL: Should NOT have self.boundaries anymore!
    assert not hasattr(assigner, 'boundaries'), \
        "BUG: SegmentAssigner should NOT create duplicate boundaries!"

    # Should use segmentation.segment_boundaries
    assert hasattr(assigner.segmentation, 'segment_boundaries'), \
        "Missing self.segmentation.segment_boundaries"

    # Verify boundaries are valid
    assert len(assigner.segmentation.segment_boundaries) > 0, \
        "Should have segment boundaries"
