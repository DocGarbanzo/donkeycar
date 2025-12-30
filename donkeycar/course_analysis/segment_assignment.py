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

        # Load parameters
        self.params = self.DEFAULT_PARAMS.copy()

        if cfg is not None:
            assign_params = getattr(cfg, 'SEGMENT_ASSIGNMENT_PARAMS', {})
            for key in self.params.keys():
                if key in assign_params:
                    self.params[key] = assign_params[key]

        if params is not None:
            self.params.update(params)

        # Compute segment boundaries once
        self._compute_segment_boundaries()

    def assign(self, x_path: np.ndarray, y_path: np.ndarray) -> np.ndarray:
        """
        Assign segment IDs to path.

        Pure function - returns segment ID array.

        Args:
            x_path: X coordinates of driven path
            y_path: Y coordinates of driven path

        Returns:
            Array of segment IDs (one per point)
        """
        if len(x_path) != len(y_path):
            raise ValueError("x_path and y_path must have same length")

        segment_ids = np.zeros(len(x_path), dtype=int)
        current_seg = self._find_initial_segment(x_path[0], y_path[0])

        for i in range(len(x_path)):
            if i > 0:
                p1 = np.array([x_path[i-1], y_path[i-1]])
                p2 = np.array([x_path[i], y_path[i]])

                # Check if crossed boundary
                if self._crossed_boundary(p1, p2, current_seg):
                    current_seg = (current_seg + 1) % \
                        self.segmentation.num_segments

            segment_ids[i] = current_seg

        return segment_ids

    def _compute_segment_boundaries(self):
        """Compute boundary lines between segments"""
        self.boundaries = []
        mean = self.segmentation.mean_course

        for i, seg in enumerate(self.segmentation.segments):
            # Boundary is at end of segment
            idx = seg.end_index
            if idx >= len(mean):
                idx = len(mean) - 1

            # Get position
            x = mean.x[idx]
            y = mean.y[idx]

            # Get tangent (from heading)
            h = mean.heading[idx]
            tangent = np.array([np.cos(h), np.sin(h)])

            # Normal is perpendicular
            normal = np.array([-tangent[1], tangent[0]])

            self.boundaries.append({
                'position': np.array([x, y]),
                'normal': normal,
                'tangent': tangent,
            })

    def _find_initial_segment(self, x: float, y: float) -> int:
        """Find closest segment to starting position"""
        mean = self.segmentation.mean_course
        pos = np.array([x, y])

        min_dist = float('inf')
        closest_seg = 0

        for i, seg in enumerate(self.segmentation.segments):
            # Check distance to segment midpoint
            mid_idx = (seg.start_index + seg.end_index) // 2
            seg_pos = np.array([mean.x[mid_idx], mean.y[mid_idx]])
            dist = np.linalg.norm(pos - seg_pos)

            if dist < min_dist:
                min_dist = dist
                closest_seg = i

        return closest_seg

    def _crossed_boundary(self, p1: np.ndarray, p2: np.ndarray,
                         current_seg: int) -> bool:
        """
        Check if path segment crossed segment boundary.

        Args:
            p1: Start point
            p2: End point
            current_seg: Current segment ID

        Returns:
            True if boundary was crossed
        """
        if current_seg >= len(self.boundaries):
            return False

        boundary = self.boundaries[current_seg]
        b_pos = boundary['position']
        b_normal = boundary['normal']

        # Vector from p1 to p2
        path_vec = p2 - p1
        path_len = np.linalg.norm(path_vec)

        zero_tol = self.params['crossing_zero_tolerance']
        if path_len < zero_tol:
            return False

        # Check if path crosses boundary line
        # Line: b_pos + t * b_normal
        # Path: p1 + s * path_vec
        # Solve for intersection

        # Check if parallel
        cross = np.cross(path_vec, b_normal)
        parallel_tol = self.params['parallel_tolerance']

        if np.abs(cross) < parallel_tol:
            return False  # Parallel, no crossing

        # Solve for t
        to_p1 = p1 - b_pos
        t = np.cross(to_p1, path_vec) / cross

        # Solve for s
        s = np.cross(to_p1, b_normal) / cross

        t_tol = self.params['crossing_t_tolerance']

        # Check if intersection is on path segment (0 <= s <= 1)
        # and reasonably close to boundary (|t| small)
        normal_limit = self.params['normal_limit_factor']

        if 0 <= s <= 1 + t_tol and np.abs(t) < normal_limit:
            return True

        return False


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
