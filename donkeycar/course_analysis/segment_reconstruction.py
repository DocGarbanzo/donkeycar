"""
Shared utilities for reconstructing SegmentAssigner from tub metadata.

This module provides functions used by both TubStatistics and TubDataset
to reconstruct a SegmentAssigner from segmentation metadata stored in
manifest.json.

Design principles:
- Single source of truth for reconstruction logic
- Pure functions (no hidden state)
- Testable with mock metadata
"""
import numpy as np

from .mean_course import MeanCourse
from .segmentation import Segment, SegmentType, CourseSegmentation
from .segment_assignment import SegmentAssigner


def reconstruct_segment_assigner(segmentation_metadata):
    """
    Reconstruct SegmentAssigner from segmentation metadata.

    Creates a functional SegmentAssigner from the mean course and boundary
    data stored in manifest metadata, enabling on-the-fly segment ID
    computation.

    :param segmentation_metadata: Dictionary containing 'mean_course',
                                  'segments', and 'segment_boundaries' keys
    :return: SegmentAssigner or None if metadata is invalid/missing
    """
    if not segmentation_metadata:
        return None

    seg_data = segmentation_metadata

    # Reconstruct mean course from stored arrays
    mc_data = seg_data.get('mean_course')
    if not mc_data:
        return None

    mean_course = MeanCourse(
        x=np.array(mc_data['x']),
        y=np.array(mc_data['y']),
        heading=np.array(mc_data['heading']),
        distance=np.array(mc_data['distance']),
        metadata={}
    )

    # Reconstruct segments list
    segments_data = seg_data.get('segments', [])
    segments = [
        Segment(
            segment_id=s['segment_id'],
            segment_type=SegmentType.STRAIGHT,  # Default, not used
            start_index=s['start_index'],
            end_index=s['end_index'],
            start_distance=0.0,
            end_distance=0.0,
            curvature_stats={}
        )
        for s in segments_data
    ]

    # Reconstruct segment boundaries for crossing detection
    boundaries_data = seg_data.get('segment_boundaries', [])
    segment_boundaries = [
        {
            'point': np.array(b['point']),
            'tangent': np.array(b['tangent']),
            'tangent_limit': b['tangent_limit'],
            'expected_denom_sign': b['expected_denom_sign'],
            'segment_from': b['segment_from'],
            'segment_to': b['segment_to'],
        }
        for b in boundaries_data
    ]

    # Create CourseSegmentation with reconstructed data
    segmentation = CourseSegmentation(
        segments=segments,
        mean_course=mean_course,
        params={},
        segment_boundaries=segment_boundaries
    )

    return SegmentAssigner(segmentation)


def compute_segment_id(position, assigner, prev_segment):
    """
    Compute segment ID for a position using boundary crossing detection.

    Uses the SegmentAssigner's boundary crossing logic to determine if
    the position has moved into a new segment.

    :param position: Position data (list/tuple with x, y as first 2 elements)
    :param assigner: SegmentAssigner for boundary crossing detection
    :param prev_segment: Previous segment ID for state tracking
    :return: Current segment ID (same as prev_segment if no crossing)
    """
    if position is None or len(position) < 2:
        return prev_segment

    p1 = np.array([position[0], position[1]])
    current_seg = prev_segment

    # Check if we crossed the boundary from current segment
    if assigner._crossed_boundary(p1, p1, current_seg):
        num_segs = assigner.segmentation.num_segments
        current_seg = (current_seg + 1) % num_segs

    return current_seg


def get_or_compute_segment_id(session_id, position, metadata_dict,
                               assigners, prev_segments):
    """
    Get or compute segment ID with lazy initialization of assigner.

    Handles lazy initialization of SegmentAssigner from metadata and
    computes segment ID using boundary crossing detection.

    :param session_id: Session identifier for assigner cache lookup
    :param position: Position data (car/pos) to assign segment for
    :param metadata_dict: Tub manifest metadata dictionary
    :param assigners: Cache dict {session_id: SegmentAssigner}
    :param prev_segments: State dict {session_id: previous_segment_id}
    :return: Computed segment ID or None if no segmentation data
    """
    if session_id not in assigners:
        session_data = metadata_dict.get(session_id, {})
        seg_data = session_data.get('segmentation')
        assigners[session_id] = reconstruct_segment_assigner(seg_data)
        prev_segments[session_id] = 0

    assigner = assigners[session_id]
    if assigner is None:
        return None

    segment = compute_segment_id(
        position, assigner, prev_segments[session_id])
    prev_segments[session_id] = segment
    return segment
