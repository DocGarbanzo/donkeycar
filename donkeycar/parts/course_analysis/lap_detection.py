"""
Lap detection algorithms for multi-lap course data.

Provides pluggable lap detection strategies using the Strategy pattern.
All magic numbers extracted to DEFAULT_PARAMS for configurability.

Design principles:
- Strategy pattern: Swap detection algorithms
- Pure functions: No hidden state
- No magic numbers: All parameters named and documented
- Testable: Works with synthetic data

Phase 2 of IMU path refactoring.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
import numpy as np

from .data_loader import PathData


@dataclass
class LapBoundary:
    """
    Container for lap boundary information.

    Attributes:
        start_index: Index where lap starts
        end_index: Index where lap ends (inclusive)
        start_time: Timestamp at lap start
        end_time: Timestamp at lap end
    """
    start_index: int
    end_index: int
    start_time: float
    end_time: float

    @property
    def duration(self) -> float:
        """Lap duration in seconds"""
        return self.end_time - self.start_time

    @property
    def num_points(self) -> int:
        """Number of data points in lap"""
        return self.end_index - self.start_index + 1


class LapDetector(ABC):
    """
    Abstract base class for lap detection algorithms.

    Concrete implementations detect lap boundaries using different methods:
    - YCrossingLapDetector: Detect when y crosses from negative to positive
    - DriftLapDetector: Detect using weighted average reversal point

    Configuration hierarchy:
    - DEFAULT_PARAMS: Class-level defaults
    - cfg: Config file overrides (if provided)
    - params: Explicit parameter overrides (highest priority)
    """

    DEFAULT_PARAMS: Dict[str, Any] = {}

    def __init__(self, cfg=None, params: Optional[Dict[str, Any]] = None):
        """
        Create lap detector.

        Args:
            cfg: Config object (optional, for config file params)
            params: Explicit parameter overrides (optional)
        """
        # Start with defaults
        self.params = self.DEFAULT_PARAMS.copy()

        # Override from config if provided
        if cfg is not None:
            lap_params = getattr(cfg, 'LAP_DETECTION_PARAMS', {})
            for key in self.params.keys():
                if key in lap_params:
                    self.params[key] = lap_params[key]

        # Override from explicit params (highest priority)
        if params is not None:
            self.params.update(params)

    @abstractmethod
    def detect_laps(self, path_data: PathData) -> List[LapBoundary]:
        """
        Detect lap boundaries in path data.

        Args:
            path_data: Path data to analyze

        Returns:
            List of LapBoundary objects, one per detected lap
        """
        pass


class YCrossingLapDetector(LapDetector):
    """
    Detect laps by y-axis crossing.

    Finds laps by detecting when y coordinate crosses from negative
    to positive. Works well for closed loops starting/ending near y=0.

    Algorithm:
    1. Start from beginning of path
    2. Find next point where y crosses from negative to positive
    3. That's the end of current lap / start of next lap
    4. Repeat from that point
    """

    DEFAULT_PARAMS = {
        'y_threshold': 0.1,        # Threshold around zero (meters)
        'min_loop_distance': 1.0,  # Minimum distance before considering
                                   # crossing (meters)
        'min_lap_length': 50,      # Minimum points per lap
    }

    def detect_laps(self, path_data: PathData) -> List[LapBoundary]:
        """
        Detect laps using y-axis crossing method.

        Args:
            path_data: Path data to analyze

        Returns:
            List of detected lap boundaries
        """
        boundaries = []
        current_start = 0
        y_threshold = self.params['y_threshold']
        min_lap_length = self.params['min_lap_length']

        while current_start < len(path_data) - min_lap_length:
            # Find next crossing from negative to positive
            crossing_idx = self._find_y_crossing(
                path_data.y, current_start, y_threshold)

            if crossing_idx is None:
                break

            # Check if lap is long enough
            lap_length = crossing_idx - current_start
            if lap_length >= min_lap_length:
                boundaries.append(LapBoundary(
                    start_index=current_start,
                    end_index=crossing_idx - 1,
                    start_time=path_data.timestamp[current_start],
                    end_time=path_data.timestamp[crossing_idx - 1]
                ))

            current_start = crossing_idx

        return boundaries

    def _find_y_crossing(self, y: np.ndarray, start_idx: int,
                        threshold: float) -> Optional[int]:
        """
        Find next point where y crosses from negative to positive.

        Args:
            y: Y coordinates
            start_idx: Index to start searching from
            threshold: Threshold around zero

        Returns:
            Index of crossing, or None if not found
        """
        for i in range(start_idx, len(y) - 1):
            if y[i] < -threshold and y[i + 1] >= -threshold:
                return i + 1

        return None


class DriftLapDetector(LapDetector):
    """
    Detect laps using weighted average reversal point detection.

    More sophisticated than y-crossing, handles courses with drift.
    Finds the reversal point where the car stops approaching the
    start position and begins moving away.

    Algorithm:
    1. Travel minimum distance from loop start
    2. Find when we get close to loop start (within max_closure_distance)
    3. Find reversal point using weighted average of distances
    4. Score potential reversals by time and distance factors
    5. Select best reversal point as lap boundary

    All magic numbers from original find_loop_end_index() extracted
    to DEFAULT_PARAMS.
    """

    DEFAULT_PARAMS = {
        'min_loop_distance': 5.0,              # Minimum travel distance (m)
        'max_closure_distance': 1.0,           # Max distance to start (m)
        'weighted_avg_weights': [0.25, 0.5,    # Weights for distance
                                0.25],          # averaging
        'reversal_tolerance': 1.001,           # Multiplier for reversal
        'vicinity_window': 2000,               # Window size for scoring
        'time_factor_weight': 0.7,             # Weight for time in score
        'distance_factor_weight': 0.3,         # Weight for distance
        'min_points_for_reversal': 7,          # Minimum points needed
        'window_around_current': 3,            # Points before/after
        'max_distance_multiplier': 2,          # For good reversals
        'min_lap_length': 50,                  # Minimum points per lap
    }

    def detect_laps(self, path_data: PathData) -> List[LapBoundary]:
        """
        Detect laps using drift/reversal method.

        Args:
            path_data: Path data to analyze

        Returns:
            List of detected lap boundaries
        """
        boundaries = []
        current_start = 0
        min_lap_length = self.params['min_lap_length']

        while current_start < len(path_data) - min_lap_length:
            # Find end of current lap
            lap_end = self._find_loop_end_index(
                path_data.x, path_data.y, current_start)

            if lap_end is None:
                break

            # Check if lap is long enough
            lap_length = lap_end - current_start
            if lap_length >= min_lap_length:
                boundaries.append(LapBoundary(
                    start_index=current_start,
                    end_index=lap_end,
                    start_time=path_data.timestamp[current_start],
                    end_time=path_data.timestamp[lap_end]
                ))

            current_start = lap_end + 1

        return boundaries

    def _find_loop_end_index(self, x: np.ndarray, y: np.ndarray,
                             start_idx: int) -> Optional[int]:
        """
        Find loop end using reversal point detection.

        This is the refactored version of find_loop_end_index() with
        all magic numbers extracted to params.

        Args:
            x: X coordinates
            y: Y coordinates
            start_idx: Index to start searching from

        Returns:
            Index of loop end, or None if not found
        """
        window = self.params['window_around_current']
        if start_idx >= len(x) - window:
            return None

        loop_start_pos = np.array([x[start_idx], y[start_idx]])

        # Calculate cumulative distance from loop start
        dx = np.diff(x[start_idx:])
        dy = np.diff(y[start_idx:])
        distances = np.sqrt(dx**2 + dy**2)
        cumulative_distance = np.concatenate([[0], np.cumsum(distances)])

        # Phase 1: Travel minimum distance
        min_distance_idx = self._find_min_distance_point(
            cumulative_distance, start_idx)
        if min_distance_idx is None:
            return None

        # Phase 2: Find when we get close to loop start
        vicinity_start_idx = self._find_vicinity_start(
            x, y, loop_start_pos, start_idx, min_distance_idx,
            cumulative_distance)
        if vicinity_start_idx is None:
            return None

        # Phase 3: Find reversal point
        return self._find_reversal_point(
            x, y, loop_start_pos, vicinity_start_idx)

    def _find_min_distance_point(self, cumulative_distance: np.ndarray,
                                 start_idx: int) -> Optional[int]:
        """Find point where minimum distance has been traveled"""
        min_loop_distance = self.params['min_loop_distance']

        for i in range(len(cumulative_distance)):
            if cumulative_distance[i] >= min_loop_distance:
                return start_idx + i

        return None

    def _find_vicinity_start(self, x: np.ndarray, y: np.ndarray,
                            loop_start_pos: np.ndarray,
                            start_idx: int, min_distance_idx: int,
                            cumulative_distance: np.ndarray
                            ) -> Optional[int]:
        """Find when we get close to loop start position"""
        max_distance = self.params['max_closure_distance']

        for i in range(min_distance_idx - start_idx,
                      len(cumulative_distance)):
            actual_idx = start_idx + i
            if actual_idx >= len(x):
                break

            current_pos = np.array([x[actual_idx], y[actual_idx]])
            distance_to_start = np.linalg.norm(
                current_pos - loop_start_pos)

            if distance_to_start <= max_distance:
                return actual_idx

        return None

    def _find_reversal_point(self, x: np.ndarray, y: np.ndarray,
                            loop_start_pos: np.ndarray,
                            vicinity_start_idx: int) -> Optional[int]:
        """Find reversal point using weighted average"""
        # Calculate distances to start from vicinity point onward
        distances_to_start = []
        for i in range(vicinity_start_idx, len(x)):
            current_pos = np.array([x[i], y[i]])
            dist = np.linalg.norm(current_pos - loop_start_pos)
            distances_to_start.append(dist)

        min_points = self.params['min_points_for_reversal']
        if len(distances_to_start) < min_points:
            return None

        # Find potential reversal points
        potential_reversals = self._find_potential_reversals(
            distances_to_start, vicinity_start_idx)

        if not potential_reversals:
            # Fallback: use minimum distance point
            min_distance = min(distances_to_start)
            min_idx = distances_to_start.index(min_distance)
            return vicinity_start_idx + min_idx

        # Score and select best reversal
        return self._select_best_reversal(
            potential_reversals, vicinity_start_idx, len(x))

    def _find_potential_reversals(self, distances_to_start: List[float],
                                  vicinity_start_idx: int
                                  ) -> List[tuple]:
        """Find points where distance starts increasing"""
        potential = []
        weights = self.params['weighted_avg_weights']
        tolerance = self.params['reversal_tolerance']
        window = self.params['window_around_current']

        for i in range(window, len(distances_to_start) - window):
            # Weighted average of current point
            current_avg = (distances_to_start[i - 1] * weights[0] +
                          distances_to_start[i] * weights[1] +
                          distances_to_start[i + 1] * weights[2])

            # Average of next few points
            next_points = distances_to_start[i + 1:i + 4]
            if len(next_points) == 3:
                next_avg = sum(next_points) / len(next_points)

                # Reversal detected
                if next_avg > current_avg * tolerance:
                    actual_idx = vicinity_start_idx + i
                    potential.append((actual_idx, current_avg, next_avg))

        return potential

    def _select_best_reversal(self, potential_reversals: List[tuple],
                             vicinity_start_idx: int,
                             total_length: int) -> int:
        """Score and select best reversal point"""
        max_distance = self.params['max_closure_distance']
        max_dist_mult = self.params['max_distance_multiplier']
        vicinity_window = min(self.params['vicinity_window'],
                             total_length - vicinity_start_idx)
        time_weight = self.params['time_factor_weight']
        dist_weight = self.params['distance_factor_weight']

        # Filter to good reversals (close to start)
        good_reversals = [r for r in potential_reversals
                         if r[1] <= max_distance * max_dist_mult]

        if not good_reversals:
            # Use first reversal
            return min(potential_reversals, key=lambda x: x[0])[0]

        # Score reversals by time and distance factors
        scored_reversals = []
        for reversal in good_reversals:
            idx, distance, next_avg = reversal
            if idx <= vicinity_start_idx + vicinity_window:
                time_factor = ((idx - vicinity_start_idx) /
                              vicinity_window)
                distance_factor = distance / max_distance
                score = time_factor * time_weight + \
                    distance_factor * dist_weight
                scored_reversals.append((score, reversal))

        if scored_reversals:
            best_score, best_reversal = min(scored_reversals)
            return best_reversal[0]
        else:
            # Use closest reversal
            return min(good_reversals, key=lambda x: x[0])[0]


class MultiLapData:
    """
    Container for multi-lap path data.

    Created using factory pattern: from_source(source, detector)
    Separates data loading from lap detection.

    Attributes:
        path_data: Full path data
        lap_boundaries: List of detected lap boundaries
        num_laps: Number of detected laps
    """

    def __init__(self, path_data: PathData,
                 lap_boundaries: List[LapBoundary]):
        """
        Create MultiLapData container.

        Args:
            path_data: Full path data
            lap_boundaries: Detected lap boundaries
        """
        self.path_data = path_data
        self.lap_boundaries = lap_boundaries
        self.num_laps = len(lap_boundaries)

    @classmethod
    def from_source(cls, source, detector: LapDetector):
        """
        Factory: Create MultiLapData from source + detector.

        This is the clean API replacing the old load_data() method.
        Dependency injection enables testing with synthetic data.

        Args:
            source: PathDataSource (CSV, Tub, or synthetic)
            detector: LapDetector strategy

        Returns:
            MultiLapData: Container with detected laps

        Example:
            source = CSVPathDataSource('path.csv')
            detector = YCrossingLapDetector()
            multilap = MultiLapData.from_source(source, detector)
        """
        path_data = source.load()
        boundaries = detector.detect_laps(path_data)
        return cls(path_data, boundaries)

    def get_lap(self, lap_index: int) -> PathData:
        """
        Extract path data for a specific lap.

        Args:
            lap_index: Index of lap (0-based)

        Returns:
            PathData for the specified lap

        Raises:
            IndexError: If lap_index out of range
        """
        if lap_index < 0 or lap_index >= self.num_laps:
            raise IndexError(
                f"Lap index {lap_index} out of range "
                f"(0-{self.num_laps-1})")

        boundary = self.lap_boundaries[lap_index]
        start = boundary.start_index
        end = boundary.end_index + 1  # +1 for inclusive slice

        return PathData(
            timestamp=self.path_data.timestamp[start:end],
            x=self.path_data.x[start:end],
            y=self.path_data.y[start:end],
            heading=self.path_data.heading[start:end],
            velocity=self.path_data.velocity[start:end]
        )

    @property
    def laps(self) -> List[PathData]:
        """Get list of PathData for all laps"""
        return [self.get_lap(i) for i in range(self.num_laps)]
