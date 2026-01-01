"""
Segment assignment to driven paths.

Assigns segment IDs to vehicle positions based on course segmentation.
All magic numbers extracted to DEFAULT_PARAMS.

Design principles:
- Pure functions: assign() returns segment IDs, no state
- No magic numbers: All tolerances configurable
- Testable: Works with synthetic paths

Phase 5 of IMU path refactoring.
"""

from typing import Optional, Dict, Any
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
    """

    DEFAULT_PARAMS = {
        'boundary_distance_tolerance': 0.05,  # Meters
        'boundary_tangent_limit': 0.8,
        'crossing_zero_tolerance': 1e-9,
        'crossing_t_tolerance': 1e-6,
        'normal_limit_factor': 1.5,
        'parallel_tolerance': 1e-6,
        'proximity_factor': 0.5,
        'reanchor_interval': 50,  # Re-detect segment every N points if stuck
    }

    def __init__(self, segmentation: CourseSegmentation,
                 cfg=None, params: Optional[Dict[str, Any]] = None):
        """
        Create segment assigner.

        Args:
            segmentation: Course segmentation
            cfg: Config object (optional)
            params: Explicit parameter overrides (optional)
        """
        self.segmentation = segmentation
        self.params = self.DEFAULT_PARAMS.copy()

        if cfg is not None:
            self._load_params_from_config(cfg)

        if params is not None:
            self.params.update(params)

        # REMOVED: self._compute_segment_boundaries()
        # Use segmentation.segment_boundaries directly - no duplication!

    def _load_params_from_config(self, cfg):
        """Load parameters from config object."""
        assign_params = getattr(cfg, 'SEGMENT_ASSIGNMENT_PARAMS', {})
        overrides = {k: assign_params[k] for k in self.params.keys()
                     if k in assign_params}
        self.params.update(overrides)

    def assign(self, x_path: np.ndarray, y_path: np.ndarray) -> np.ndarray:
        """
        Assign segment IDs to path.

        Pure function - returns segment ID array.

        Uses boundary crossing detection with periodic re-anchoring to handle
        paths that deviate from the mean course. Re-anchoring occurs after
        lap wrap-around or when boundary detection fails for too long.

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

        points_since_crossing = 0
        reanchor_interval = self.params.get('reanchor_interval', 50)

        for i in range(1, len(x_path)):
            p1 = np.array([x_path[i-1], y_path[i-1]])
            p2 = np.array([x_path[i], y_path[i]])

            prev_seg = current_seg
            crossed = self._crossed_boundary(p1, p2, current_seg)

            if crossed:
                current_seg, points_since_crossing = \
                    self._handle_boundary_crossing(
                        prev_seg, x_path[i], y_path[i])
            else:
                current_seg, points_since_crossing = \
                    self._handle_no_crossing(
                        current_seg, points_since_crossing,
                        reanchor_interval, x_path[i], y_path[i])

            segment_ids[i] = current_seg

        return segment_ids

    def _handle_boundary_crossing(self, prev_seg: int,
                                   x: float, y: float) -> tuple:
        """
        Handle segment advancement when boundary is crossed.

        Returns:
            Tuple of (new_segment_id, points_since_crossing)
        """
        current_seg = (prev_seg + 1) % self.segmentation.num_segments
        points_since_crossing = 0

        # After lap wrap-around, re-anchor to handle offset paths
        is_wrap = (prev_seg == self.segmentation.num_segments - 1 and
                   current_seg == 0)
        if not is_wrap:
            return current_seg, points_since_crossing

        current_seg = self._find_initial_segment(x, y)
        return current_seg, points_since_crossing

    def _handle_no_crossing(self, current_seg: int,
                            points_since_crossing: int,
                            reanchor_interval: int,
                            x: float, y: float) -> tuple:
        """
        Handle periodic re-anchoring when no boundary crossed.

        Returns:
            Tuple of (segment_id, points_since_crossing)
        """
        points_since_crossing += 1

        if points_since_crossing < reanchor_interval:
            return current_seg, points_since_crossing

        detected = self._find_initial_segment(x, y)
        if not self._is_segment_ahead(current_seg, detected):
            return current_seg, points_since_crossing

        return detected, 0

    def _is_segment_ahead(self, current: int, candidate: int) -> bool:
        """
        Check if candidate segment is ahead of current in course order.

        For re-anchoring purposes, only allows strict forward progression
        (no wrap-around). Wrap-around from last segment to segment 0
        must happen via actual boundary crossing, not via re-anchoring.
        """
        return current < candidate

    def _find_initial_segment(self, x: float, y: float) -> int:
        """
        Find initial segment using nearest-neighbor to mean course.

        CRITICAL: For INITIAL segment detection only, we use nearest-neighbor
        to find the closest point on the mean course, then look up which
        segment that point belongs to.

        Incremental crossing detection (while driving) uses tangent projection.

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

        # Build array of mean course points
        course_points = np.column_stack([mean.x, mean.y])

        # Find nearest point on mean course
        distances = np.linalg.norm(course_points - point, axis=1)
        nearest_idx = np.argmin(distances)

        # Find which segment contains this index
        for seg in self.segmentation.segments:
            if seg.start_index <= nearest_idx <= seg.end_index:
                return seg.segment_id

            # Handle wrap-around case
            if seg.start_index > seg.end_index:
                if nearest_idx >= seg.start_index or nearest_idx <= seg.end_index:
                    return seg.segment_id

        # Fallback to segment 0
        return 0

    @staticmethod
    def _signed_distance_along_normal(point: np.ndarray,
                                      boundary: dict) -> float:
        """
        Compute signed distance from point to boundary along normal.

        Positive distance means point is on the "positive" side of the boundary
        (past the boundary in the direction of course progression).

        Args:
            point: Point to test (2D array)
            boundary: Boundary dict with 'point' and 'normal'

        Returns:
            Signed distance along normal (negative = before, positive = after)
        """
        vec = point - boundary['point']
        return np.dot(vec, boundary['normal'])

    @staticmethod
    def _signed_distance_along_tangent(point: np.ndarray,
                                       boundary: dict) -> float:
        """
        Compute signed distance from point to boundary along course tangent.

        Measures how far before (negative) or after (positive) the boundary
        point the given point lies, projected onto the course direction.

        Args:
            point: Point to test (2D array)
            boundary: Boundary dict with 'point' and 'tangent'

        Returns:
            Signed distance along tangent (negative = before, positive = after)
        """
        vec = point - boundary['point']
        return np.dot(vec, boundary['tangent'])

    def _crossed_boundary(self, p1: np.ndarray, p2: np.ndarray,
                         current_seg: int) -> bool:
        """
        Check if path segment crossed segment boundary.

        Uses two methods:
        1. Line intersection (strict, requires crossing near boundary point)
        2. Signed distance (relaxed, checks if points crossed from before to
           after boundary)

        Args:
            p1: Start point
            p2: End point
            current_seg: Current segment ID

        Returns:
            True if boundary was crossed
        """
        # Find boundary exiting current_seg
        boundary = None
        for b in self.segmentation.segment_boundaries:
            if b.get('segment_from') == current_seg:
                boundary = b
                break

        if boundary is None:
            return False

        # Method 1: Line intersection (for nearby crossings)
        if self._line_intersection_crossing(p1, p2, boundary):
            return True

        # Method 2: Signed distance along tangent (for offset paths)
        # This catches crossings that occur far from the boundary point
        return self._signed_distance_crossing(p1, p2, boundary)

    def _line_intersection_crossing(self, p1: np.ndarray, p2: np.ndarray,
                                     boundary: dict) -> bool:
        """Check crossing via line intersection (original method)."""
        b_pos = boundary['point']  # Use 'point', not 'position'
        b_normal = boundary['normal']

        path_vec = p2 - p1
        path_len = np.linalg.norm(path_vec)

        zero_tol = self.params['crossing_zero_tolerance']
        if path_len < zero_tol:
            return False

        cross = np.cross(path_vec, b_normal)
        parallel_tol = self.params['parallel_tolerance']

        if np.abs(cross) < parallel_tol:
            return False

        to_p1 = p1 - b_pos
        t = np.cross(to_p1, path_vec) / cross
        s = np.cross(to_p1, b_normal) / cross

        t_tol = self.params['crossing_t_tolerance']
        normal_limit = self.params['normal_limit_factor']

        return 0 <= s <= 1 + t_tol and np.abs(t) < normal_limit

    def _signed_distance_crossing(self, p1: np.ndarray, p2: np.ndarray,
                                   boundary: dict) -> bool:
        """
        Check crossing via signed distance along NORMAL.

        CRITICAL: Uses normal projection for tangent projection algorithm.

        Detects when path moves from before boundary (negative side) to
        after boundary (positive side) by checking signed distance along
        the boundary normal vector.

        Args:
            p1: Start point of path segment
            p2: End point of path segment
            boundary: Boundary dict with 'point' and 'normal'

        Returns:
            True if crossed from negative to positive side
        """
        b_point = boundary['point']
        b_normal = boundary['normal']

        # Signed distance along normal: negative = before, positive = after
        d1 = np.dot(p1 - b_point, b_normal)
        d2 = np.dot(p2 - b_point, b_normal)

        # Crossed if we went from before (d1 <= 0) to after (d2 > 0)
        return d1 <= 0 < d2


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

        # Build KD-tree from mean course
        mean = segmentation.mean_course
        self.course_points = np.column_stack([mean.x, mean.y])

        # Build segment lookup (which segment each point belongs to)
        self.point_segments = np.zeros(len(mean), dtype=int)
        for seg in segmentation.segments:
            self.point_segments[seg.start_index:seg.end_index+1] = \
                seg.segment_id

        # Create KD-tree
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

        # Find nearest course point
        dist, idx = self.kdtree.query(pos)

        # Get segment
        segment_id = int(self.point_segments[idx])

        # Calculate confidence based on distance
        confidence = 1.0 / (1.0 + dist)

        # Cross-track error
        cross_track_error = float(dist)

        # Heading error (if provided)
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
