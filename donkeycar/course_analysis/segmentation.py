"""
Course segmentation strategies.

Provides pluggable segmentation algorithms using Strategy pattern.
Segments mean course into geometric features (straights, turns, etc.).

Design principles:
- Strategy pattern: Swap segmentation algorithms
- No magic numbers: All thresholds configurable
- Pure functions where possible
- Testable with synthetic courses

Phase 4 of IMU path refactoring.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
import numpy as np
from scipy.signal import find_peaks

from .mean_course import MeanCourse


def filter_short_boundaries(boundaries: List[int], distance: np.ndarray,
                            min_length: float) -> List[int]:
    """
    Filter out segments shorter than minimum length by merging.

    Standalone utility function used by both old and new APIs.
    Short segments are merged with adjacent segments by removing
    their end boundaries. For closed loops, also checks and merges
    short wrap-around segments.

    Note: Boundaries don't include index 0, but we need to check
    the segment from 0 to first boundary.

    Args:
        boundaries: List of boundary indices (not including 0)
        distance: Distance array
        min_length: Minimum segment length

    Returns:
        Filtered list of boundaries (not including 0)
    """
    if len(boundaries) == 0:
        return boundaries

    # Check first segment (from 0 to first boundary)
    merged = []
    start_idx = 0

    for i in range(len(boundaries)):
        seg_end_idx = boundaries[i]
        seg_length = distance[seg_end_idx] - distance[start_idx]
        if seg_length < min_length: continue  # Skip short segments

        merged.append(boundaries[i])
        start_idx = boundaries[i]

    # Check wrap-around segment length for closed loops
    if len(merged) < 1:
        return _handle_empty_merged(boundaries)

    total_distance = distance[-1]
    last_boundary_idx = merged[-1]
    wrap_length = total_distance - distance[last_boundary_idx]

    # Merge wrap-around if too short and we have > 1 boundary
    if wrap_length < min_length and len(merged) > 1:
        merged.pop()

    # Handle empty merged list after wrap-around check
    if len(merged) > 0:
        return merged

    return _handle_empty_merged(boundaries)


def _handle_empty_merged(boundaries: List[int]) -> List[int]:
    """Handle case when no boundaries meet min_length."""
    if len(boundaries) < 1:
        return boundaries

    mid_idx = len(boundaries) // 2
    return [boundaries[mid_idx]]


class SegmentType(Enum):
    """Course segment types"""
    STRAIGHT = "straight"
    LEFT_TURN = "left_turn"
    RIGHT_TURN = "right_turn"
    S_CURVE_LR = "s_curve_lr"
    S_CURVE_RL = "s_curve_rl"
    CHICANE = "chicane"


@dataclass
class Segment:
    """Individual course segment"""
    segment_id: int
    segment_type: SegmentType
    start_index: int
    end_index: int
    start_distance: float
    end_distance: float
    curvature_stats: Dict[str, float]


class SegmentationStrategy(ABC):
    """Base class for boundary detection strategies"""

    @abstractmethod
    def detect_boundaries(self, curvature: np.ndarray,
                         distance: np.ndarray,
                         params: Dict[str, Any]) -> List[int]:
        """
        Detect segment boundaries.

        Args:
            curvature: Curvature values (rad/m)
            distance: Distance values (m)
            params: Strategy-specific parameters

        Returns:
            List of boundary indices
        """
        pass


class ThresholdSegmentation(SegmentationStrategy):
    """Detect boundaries at curvature threshold crossings"""

    def detect_boundaries(self, curvature, distance, params):
        """Detect where curvature crosses threshold"""
        threshold = params.get('straight_curvature_threshold', 0.08)
        boundaries = []

        is_straight = np.abs(curvature) < threshold

        for i in range(1, len(is_straight)):
            if is_straight[i] != is_straight[i-1]:
                boundaries.append(i)

        return boundaries


class ExtremaSegmentation(SegmentationStrategy):
    """Detect boundaries at curvature peaks/valleys"""

    def detect_boundaries(self, curvature, distance, params):
        """Detect local maxima and minima in curvature"""
        prominence = params.get('extrema_prominence', 0.02)

        # Find peaks (left turns)
        peaks, _ = find_peaks(curvature, prominence=prominence)

        # Find valleys (right turns)
        valleys, _ = find_peaks(-curvature, prominence=prominence)

        # Combine and sort
        boundaries = sorted(list(peaks) + list(valleys))
        return boundaries


class GradientSegmentation(SegmentationStrategy):
    """Detect boundaries at curvature change points"""

    def detect_boundaries(self, curvature, distance, params):
        """Detect where curvature gradient is large"""
        prominence = params.get('gradient_prominence', 0.1)

        # Calculate curvature gradient
        grad = np.abs(np.gradient(curvature))

        # Find peaks in gradient
        peaks, _ = find_peaks(grad, prominence=prominence)

        return list(peaks)


class HybridSegmentation(SegmentationStrategy):
    """Combines threshold + extrema methods"""

    def detect_boundaries(self, curvature, distance, params):
        """Use both threshold and extrema detection"""
        threshold_seg = ThresholdSegmentation()
        extrema_seg = ExtremaSegmentation()

        boundaries_thresh = threshold_seg.detect_boundaries(
            curvature, distance, params)
        boundaries_extrema = extrema_seg.detect_boundaries(
            curvature, distance, params)

        # Combine and remove duplicates
        all_boundaries = set(boundaries_thresh + boundaries_extrema)
        return sorted(list(all_boundaries))


class CourseSegmenter:
    """
    Creates segmentation from mean course.

    Orchestrates curvature calculation, boundary detection,
    segment creation, and classification.
    """

    DEFAULT_PARAMS = {
        # Curvature calculation
        'curvature_window': 5,
        'curvature_smoothing_window': 21,

        # Boundary detection
        'straight_curvature_threshold': 0.08,  # rad/m
        'min_segment_length': 0.8,             # meters
        'gradient_prominence': 0.1,
        'extrema_prominence': 0.02,

        # Segment classification
        'inflection_threshold': 0.05,          # rad/m
        'inflection_chicane_threshold': 3,     # count
        'inflection_scurve_min': 1,            # count
        'inflection_scurve_long': 2,           # count
        'scurve_length_threshold': 10,         # meters
        'classification_window': 5,            # points

        # Adaptive threshold
        'use_adaptive_threshold': True,
        'adaptive_percentile': 20,             # percentile
        'adaptive_min_threshold': 0.05,        # rad/m
        'adaptive_max_threshold': 2.0,         # rad/m
    }

    def __init__(self, strategy: SegmentationStrategy,
                 cfg=None, params: Optional[Dict[str, Any]] = None):
        """
        Create course segmenter.

        Args:
            strategy: Segmentation strategy to use
            cfg: Config object (optional)
            params: Explicit parameter overrides (optional)
        """
        self.strategy = strategy

        # Load parameters
        self.params = self.DEFAULT_PARAMS.copy()

        # Apply config parameters if provided
        if cfg is not None:
            seg_params = getattr(cfg, 'SEGMENTATION_PARAMS', {})
            self._apply_config_params(seg_params)

        # Apply explicit parameter overrides
        if params is not None:
            self.params.update(params)

    def _apply_config_params(self, seg_params: Dict[str, Any]):
        """Apply config parameters to self.params."""
        updates = {k: v for k, v in seg_params.items() if k in self.params}
        self.params.update(updates)

    def segment(self, mean_course: MeanCourse) -> 'CourseSegmentation':
        """
        Segment mean course.

        Pure function - returns new CourseSegmentation.

        Args:
            mean_course: Mean course to segment

        Returns:
            CourseSegmentation with detected segments
        """
        # Calculate curvature
        curvature = self._calculate_curvature(mean_course)

        # Optionally adjust threshold
        if self.params['use_adaptive_threshold']:
            self._adjust_threshold_adaptive(curvature)

        # Detect boundaries
        boundary_indices = self.strategy.detect_boundaries(
            curvature, mean_course.distance, self.params)

        # Filter short segments by merging
        boundary_indices = filter_short_boundaries(
            boundary_indices, mean_course.distance,
            self.params['min_segment_length'])

        # Create segments
        segments = self._create_segments(
            boundary_indices, mean_course, curvature)

        # Merge adjacent same-type segments
        segments = self._merge_adjacent_segments(segments)

        # Compute segment boundaries
        boundaries = self._compute_segment_boundaries(segments, mean_course)

        return CourseSegmentation(segments, mean_course, self.params.copy(),
                                 boundaries)

    def _calculate_curvature(self, mean_course: MeanCourse) -> np.ndarray:
        """Calculate curvature from course"""
        window = self.params['curvature_window']

        dx = np.gradient(mean_course.x, mean_course.distance)
        dy = np.gradient(mean_course.y, mean_course.distance)

        ddx = np.gradient(dx, mean_course.distance)
        ddy = np.gradient(dy, mean_course.distance)

        curvature = (dx * ddy - dy * ddx) / \
            np.power(dx**2 + dy**2, 1.5)

        # Smooth curvature
        if window > 1:
            kernel = np.ones(window) / window
            curvature = np.convolve(curvature, kernel, mode='same')

        return curvature

    def _adjust_threshold_adaptive(self, curvature: np.ndarray):
        """Adjust threshold based on curvature distribution"""
        percentile = self.params['adaptive_percentile']
        abs_curv = np.abs(curvature)
        threshold = np.percentile(abs_curv, percentile)

        # Clamp to bounds
        min_t = self.params['adaptive_min_threshold']
        max_t = self.params['adaptive_max_threshold']
        threshold = np.clip(threshold, min_t, max_t)

        self.params['straight_curvature_threshold'] = threshold

    def _create_segments(self, boundaries: List[int],
                        mean_course: MeanCourse,
                        curvature: np.ndarray) -> List[Segment]:
        """Create segments from boundaries"""
        segments = []

        # Add start and end boundaries
        all_boundaries = [0] + sorted(boundaries) + [len(mean_course) - 1]

        for i in range(len(all_boundaries) - 1):
            start_idx = all_boundaries[i]
            end_idx = all_boundaries[i + 1]

            # Get segment curvature
            seg_curv = curvature[start_idx:end_idx+1]
            curv_stats = {
                'mean': np.mean(seg_curv),
                'max': np.max(np.abs(seg_curv)),
                'std': np.std(seg_curv),
            }

            # Classify segment
            seg_type = self._classify_segment(seg_curv, mean_course,
                                              start_idx, end_idx)

            segments.append(Segment(
                segment_id=i,
                segment_type=seg_type,
                start_index=start_idx,
                end_index=end_idx,
                start_distance=mean_course.distance[start_idx],
                end_distance=mean_course.distance[end_idx],
                curvature_stats=curv_stats
            ))

        return segments

    def _classify_segment(self, curvature: np.ndarray,
                         mean_course: MeanCourse,
                         start_idx: int, end_idx: int) -> SegmentType:
        """Classify segment type from curvature"""
        mean_curv = np.mean(curvature)
        max_abs_curv = np.max(np.abs(curvature))
        threshold = self.params['straight_curvature_threshold']

        # Count inflection points
        inflection_thresh = self.params['inflection_threshold']
        sign_changes = np.diff(np.sign(curvature - inflection_thresh))
        inflection_count = np.sum(np.abs(sign_changes) > 0)

        # Segment length
        seg_length = mean_course.distance[end_idx] - \
            mean_course.distance[start_idx]

        # Classify
        if max_abs_curv < threshold:
            return SegmentType.STRAIGHT

        # Check for chicane or S-curve
        chicane_thresh = self.params['inflection_chicane_threshold']
        scurve_min = self.params['inflection_scurve_min']
        scurve_long = self.params['inflection_scurve_long']
        scurve_len = self.params['scurve_length_threshold']

        if inflection_count >= chicane_thresh:
            return SegmentType.CHICANE
        elif inflection_count >= scurve_min:
            if inflection_count >= scurve_long or seg_length > scurve_len:
                # Determine direction
                if mean_curv > 0:
                    return SegmentType.S_CURVE_LR
                else:
                    return SegmentType.S_CURVE_RL

        # Simple turn
        if mean_curv > threshold / 2:
            return SegmentType.LEFT_TURN
        elif mean_curv < -threshold / 2:
            return SegmentType.RIGHT_TURN
        else:
            return SegmentType.STRAIGHT

    def _merge_adjacent_segments(self, segments: List[Segment]
                                 ) -> List[Segment]:
        """Merge adjacent segments of same type (conservative)"""
        if len(segments) <= 1:
            return segments

        merged = [segments[0]]

        for seg in segments[1:]:
            prev = merged[-1]

            # Only merge straights
            if (prev.segment_type == SegmentType.STRAIGHT and
                seg.segment_type == SegmentType.STRAIGHT):

                # Merge by updating end of previous segment
                merged[-1] = Segment(
                    segment_id=prev.segment_id,
                    segment_type=prev.segment_type,
                    start_index=prev.start_index,
                    end_index=seg.end_index,
                    start_distance=prev.start_distance,
                    end_distance=seg.end_distance,
                    curvature_stats=prev.curvature_stats
                )
            else:
                # Renumber and add
                seg = Segment(
                    segment_id=len(merged),
                    segment_type=seg.segment_type,
                    start_index=seg.start_index,
                    end_index=seg.end_index,
                    start_distance=seg.start_distance,
                    end_distance=seg.end_distance,
                    curvature_stats=seg.curvature_stats
                )
                merged.append(seg)

        return merged

    def _compute_segment_boundaries(self, segments: List[Segment],
                                    mean_course: MeanCourse) \
            -> List[Dict[str, Any]]:
        """
        Compute perpendicular boundary lines at each segment transition.

        Creates boundary line representations for detecting when a driven
        path crosses from one segment to the next. Each boundary is
        perpendicular to the mean course at the transition point.

        For closed loops:
        - Skips boundary at index 0 (arbitrary start point)
        - Creates boundary at last index for final segment to complete loop

        Args:
            segments: List of segments
            mean_course: Mean course that was segmented

        Returns:
            List of boundary dicts with point, normal, tangent, segment IDs
        """
        if not segments:
            return []

        boundaries = []
        num_points = len(mean_course.x)
        base_tangent_limit = max(
            0.3, self.params.get('boundary_tangent_limit', 2.0))

        for i, segment in enumerate(segments):
            # Boundary at end of this segment (start of next)
            boundary_idx = segment.end_index

            # Skip boundary at index 0 (arbitrary loop start)
            if boundary_idx == 0:
                continue

            # Position on mean course
            x_bound = mean_course.x[boundary_idx]
            y_bound = mean_course.y[boundary_idx]

            # Heading at boundary (tangent to course) - already in radians
            heading_rad = mean_course.heading[boundary_idx]

            # Normal vector perpendicular to heading (rotate 90 degrees)
            # Tangent direction: (cos(h), sin(h))
            # Normal (left of course): (-sin(h), cos(h))
            normal_x = -np.sin(heading_rad)
            normal_y = np.cos(heading_rad)
            tangent_x = np.cos(heading_rad)
            tangent_y = np.sin(heading_rad)

            next_segment = (i + 1) % len(segments)
            next_seg = segments[next_segment]

            # Calculate segment lengths
            seg_length = segment.end_distance - segment.start_distance
            next_length = next_seg.end_distance - next_seg.start_distance

            # Keep crossings local to the boundary position
            avg_length = 0.25 * (seg_length + next_length)
            adaptive_limit = avg_length if avg_length > 0 \
                else base_tangent_limit
            tangent_limit = max(
                0.3, min(base_tangent_limit, adaptive_limit))

            # Line equation parameters
            slope = None
            intercept = None
            x_offset = None
            if abs(tangent_x) > 1e-6:
                slope = tangent_y / tangent_x
                intercept = y_bound - slope * x_bound
            else:
                x_offset = x_bound

            # Determine expected sign for crossing detection
            prev_idx = (boundary_idx - 1) % num_points
            next_idx = (boundary_idx + 1) % num_points
            prev_point = np.array([
                mean_course.x[prev_idx],
                mean_course.y[prev_idx]
            ])
            next_point = np.array([
                mean_course.x[next_idx],
                mean_course.y[next_idx]
            ])

            mean_vec = next_point - prev_point
            denom_sign = np.dot(
                np.array([tangent_x, tangent_y]), mean_vec)
            expected_sign = 1.0 if denom_sign >= 0 else -1.0

            boundaries.append({
                'point': np.array([x_bound, y_bound]),
                'normal': np.array([normal_x, normal_y]),
                'tangent': np.array([tangent_x, tangent_y]),
                'tangent_limit': tangent_limit,
                'expected_denom_sign': expected_sign,
                'slope': slope,
                'intercept': intercept,
                'x_offset': x_offset,
                'segment_from': i,
                'segment_to': next_segment
            })

        return boundaries


class CourseSegmentation:
    """
    Immutable segmented course.

    Contains list of segments with their classifications and
    geometric properties.
    """

    def __init__(self, segments: List[Segment],
                 mean_course: MeanCourse,
                 params: Dict[str, Any],
                 segment_boundaries: List[Dict[str, Any]] = None):
        """
        Create course segmentation.

        Args:
            segments: List of detected segments
            mean_course: Mean course that was segmented
            params: Parameters used for segmentation
            segment_boundaries: List of boundary dicts with point, normal,
                              tangent, segment_from, segment_to (optional)
        """
        self._segments = list(segments)
        self._mean_course = mean_course
        self._params = dict(params)
        self._segment_boundaries = list(segment_boundaries) \
            if segment_boundaries else []

    @property
    def segments(self) -> List[Segment]:
        """List of segments (read-only copy)"""
        return list(self._segments)

    @property
    def mean_course(self) -> MeanCourse:
        """Mean course (immutable)"""
        return self._mean_course

    @property
    def params(self) -> Dict[str, Any]:
        """Segmentation parameters (read-only copy)"""
        return dict(self._params)

    @property
    def num_segments(self) -> int:
        """Number of segments"""
        return len(self._segments)

    @property
    def total_segments(self) -> int:
        """Total number of segments (alias for num_segments)"""
        return len(self._segments)

    @property
    def segment_boundaries(self) -> List[Dict[str, Any]]:
        """
        Segment boundary lines (read-only copy).

        Each boundary is a dict with:
        - 'point': np.array([x, y]) - position on mean course
        - 'normal': np.array([nx, ny]) - unit normal vector
        - 'tangent': np.array([tx, ty]) - unit tangent vector
        - 'segment_from': int - segment ID before boundary
        - 'segment_to': int - segment ID after boundary
        - Additional fields for crossing detection
        """
        return list(self._segment_boundaries)

    def get_segment(self, segment_id: int) -> Segment:
        """Get segment by ID"""
        if 0 <= segment_id < len(self._segments):
            return self._segments[segment_id]
        raise IndexError(f"Segment {segment_id} out of range")
