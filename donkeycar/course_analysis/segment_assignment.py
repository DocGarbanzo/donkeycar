"""
Segment assignment to driven paths.

Assigns segment IDs to vehicle positions based on course segmentation.

Design principles:
- Pure functions: assign() returns segment IDs, no state
- Simple algorithm: tangent projection for boundary crossing
- Testable: Works with synthetic paths

Phase 5 of IMU path refactoring.
"""

from typing import Optional
from dataclasses import dataclass
import numpy as np

from .segmentation import CourseSegmentation


@dataclass
class SegmentEstimate:
    """Result of segment estimation for a position"""
    segment_id: int
    confidence: float
    cross_track_error: float
    heading_error: float


class SegmentAssigner:
    """
    Assigns segments to driven path.

    Uses boundary crossing detection to determine which segment
    each point belongs to.

    Algorithm:
    1. Find initial segment using nearest-neighbor to mean course
    2. For each subsequent point, check if boundary was crossed
    3. Boundary crossing: dot(pos - boundary, tangent) transitions neg->pos
    """

    def __init__(self, segmentation: CourseSegmentation,
                 cfg=None, params=None):
        """
        Create segment assigner.

        Args:
            segmentation: Course segmentation
            cfg: Config object (unused, kept for API compatibility)
            params: Parameter overrides (unused, kept for API compatibility)
        """
        self.segmentation = segmentation

    def assign(self, x_path: np.ndarray, y_path: np.ndarray) -> np.ndarray:
        """
        Assign segment IDs to path.

        Args:
            x_path: X coordinates of driven path
            y_path: Y coordinates of driven path

        Returns:
            Array of segment IDs (one per point)
        """
        if len(x_path) != len(y_path):
            raise ValueError("x_path and y_path must have same length")
        if len(x_path) == 0:
            return np.array([], dtype=int)

        segment_ids = np.zeros(len(x_path), dtype=int)
        current_seg = self._find_initial_segment(x_path[0], y_path[0])
        segment_ids[0] = current_seg

        for i in range(1, len(x_path)):
            p1 = np.array([x_path[i-1], y_path[i-1]])
            p2 = np.array([x_path[i], y_path[i]])
            if self._crossed_boundary(p1, p2, current_seg):
                num_segs = self.segmentation.num_segments
                current_seg = (current_seg + 1) % num_segs
            segment_ids[i] = current_seg

        return segment_ids

    def _find_initial_segment(self, x: float, y: float) -> int:
        """
        Find initial segment using nearest-neighbor to mean course.

        Algorithm:
        1. Find nearest point on mean course to starting position
        2. Look up which segment that mean course point belongs to
        3. Return that segment ID

        Args:
            x: X coordinate of starting position
            y: Y coordinate of starting position

        Returns:
            Segment ID that the position belongs to
        """
        point = np.array([x, y])
        mean = self.segmentation.mean_course
        course_points = np.column_stack([mean.x, mean.y])
        distances = np.linalg.norm(course_points - point, axis=1)
        nearest_idx = np.argmin(distances)

        for seg in self.segmentation.segments:
            if self._is_index_in_segment(nearest_idx, seg):
                return seg.segment_id

        raise ValueError(f"No segment found for index {nearest_idx}")

    def _is_index_in_segment(self, idx: int, seg) -> bool:
        """
        Check if index falls within segment boundaries.

        Args:
            idx: Index to check
            seg: Segment object with start_index and end_index

        Returns:
            True if index is within segment boundaries
        """
        if seg.start_index <= seg.end_index:
            return seg.start_index <= idx <= seg.end_index
        return idx >= seg.start_index or idx <= seg.end_index

    def _crossed_boundary(self, p1: np.ndarray, p2: np.ndarray,
                          current_seg: int) -> bool:
        """
        Check if path segment crossed segment boundary.

        Boundary crossing occurs when dot(pos - boundary, tangent)
        transitions from negative to positive.

        Args:
            p1: Start point
            p2: End point
            current_seg: Current segment ID

        Returns:
            True if boundary was crossed
        """
        boundary = self._get_boundary_from_segment(current_seg)
        if boundary is None:
            return False

        b_point = boundary['point']
        b_tangent = boundary['tangent']
        d1 = np.dot(p1 - b_point, b_tangent)
        d2 = np.dot(p2 - b_point, b_tangent)
        return d1 <= 0 < d2

    def _get_boundary_from_segment(self, segment_id: int) -> Optional[dict]:
        """Get boundary exiting the given segment."""
        for b in self.segmentation.segment_boundaries:
            if b.get('segment_from') == segment_id:
                return b
        return None


class SegmentEstimator:
    """
    Real-time segment estimation from vehicle position.

    Uses KD-tree for fast nearest-neighbor search.
    """

    def __init__(self, segmentation: CourseSegmentation):
        """
        Create segment estimator.

        Args:
            segmentation: Course segmentation
        """
        self.segmentation = segmentation
        mean = segmentation.mean_course
        self.course_points = np.column_stack([mean.x, mean.y])

        self.point_segments = np.zeros(len(mean), dtype=int)
        for seg in segmentation.segments:
            start, end = seg.start_index, seg.end_index + 1
            self.point_segments[start:end] = seg.segment_id

        from scipy.spatial import KDTree
        self.kdtree = KDTree(self.course_points)

    def estimate(self, x: float, y: float,
                 heading: Optional[float] = None) -> SegmentEstimate:
        """
        Estimate segment from position.

        Args:
            x: X position
            y: Y position
            heading: Heading angle (optional, improves confidence)

        Returns:
            SegmentEstimate with segment ID and confidence
        """
        pos = np.array([x, y])
        dist, idx = self.kdtree.query(pos)
        segment_id = int(self.point_segments[idx])
        confidence = 1.0 / (1.0 + dist)
        cross_track_error = float(dist)

        heading_error = 0.0
        if heading is not None:
            course_heading = self.segmentation.mean_course.heading[idx]
            heading_error = abs(heading - course_heading)
            if heading_error > np.pi:
                heading_error = 2 * np.pi - heading_error

        return SegmentEstimate(
            segment_id=segment_id,
            confidence=confidence,
            cross_track_error=cross_track_error,
            heading_error=heading_error
        )
