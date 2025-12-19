"""
Course Analysis and Segmentation

This module provides tools for analyzing multi-lap course data to create a mean
reference course, automatically segment it into geometric features, and provide
real-time segment detection.

Classes:
    MultiLapData: Container for raw multi-lap course data
    MeanCourse: Reconstructed mean course from multi-lap data
    CourseSegmentation: Segmented course with classified segments
    Segment: Individual course segment with geometric properties
    SegmentEstimator: Real-time segment estimation from vehicle position/heading
    SegmentEstimate: Result of segment estimation

Author: DonkeyCar Community
Date: 2025
"""

import numpy as np
import json
import logging
import math
from typing import List, Optional, Tuple, Dict, Any
from enum import Enum
from scipy.signal import savgol_filter
from scipy.spatial import KDTree
from scipy.interpolate import interp1d

logger = logging.getLogger(__name__)


def convert_numpy_types(obj):
    """
    Recursively convert NumPy types to native Python types for JSON serialization.
    
    Args:
        obj: Object to convert (can be dict, list, NumPy type, etc.)
    
    Returns:
        Converted object with native Python types
    """
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {key: convert_numpy_types(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(item) for item in obj]
    elif isinstance(obj, tuple):
        return tuple(convert_numpy_types(item) for item in obj)
    else:
        return obj


def find_loop_end_index_y_crossing(x: np.ndarray, y: np.ndarray,
                                   start_idx: int = 0,
                                   min_loop_distance: float = 1.0,
                                   y_threshold: float = 0.1) -> Optional[int]:
    """
    Find loop end index by detecting when y crosses from negative to positive.

    Args:
        x: X coordinates array (unused, kept for API compatibility)
        y: Y coordinates array
        start_idx: Index to start searching from
        min_loop_distance: Unused, kept for API compatibility
        y_threshold: Unused, kept for API compatibility

    Returns:
        Index where y crosses from negative to positive, or None if not found
    """
    # Find first crossing from negative to positive
    for i in range(start_idx, len(y) - 1):
        if y[i] < 0 and y[i + 1] >= 0:
            return i + 1

    return None


def find_loop_end_index(x: np.ndarray, y: np.ndarray,
                        start_idx: int = 0,
                        min_loop_distance: float = 5.0,
                        max_distance: float = 1.0) -> Optional[int]:
    """
    Find loop end index using weighted average reversal point detection.

    This is a simplified version of the drift correction algorithm
    from imu_visualization.py, adapted for lap detection without
    correction.

    Args:
        x: X coordinates array
        y: Y coordinates array
        start_idx: Index to start searching from
        min_loop_distance: Minimum distance to travel before considering
                          loop closure (meters)
        max_distance: Maximum distance from start position to begin
                     looking for reversal point (meters)

    Returns:
        Index of loop end point, or None if not found
    """
    if start_idx >= len(x) - 3:
        return None

    # Get loop start position
    loop_start_pos = np.array([x[start_idx], y[start_idx]])

    # Calculate cumulative distance from loop start
    dx = np.diff(x[start_idx:])
    dy = np.diff(y[start_idx:])
    distances = np.sqrt(dx**2 + dy**2)
    cumulative_distance = np.concatenate([[0], np.cumsum(distances)])

    # Phase 1: Travel minimum distance
    min_distance_idx = None
    for i in range(len(cumulative_distance)):
        if cumulative_distance[i] >= min_loop_distance:
            min_distance_idx = start_idx + i
            break

    if min_distance_idx is None:
        return None

    # Phase 2: Find when we get close to the loop start
    vicinity_start_idx = None
    for i in range(min_distance_idx - start_idx, len(cumulative_distance)):
        actual_idx = start_idx + i
        if actual_idx >= len(x):
            break

        current_pos = np.array([x[actual_idx], y[actual_idx]])
        distance_to_start = np.linalg.norm(current_pos - loop_start_pos)

        if distance_to_start <= max_distance:
            vicinity_start_idx = actual_idx
            break

    if vicinity_start_idx is None:
        return None

    # Phase 3: Find reversal point using weighted average
    distances_to_start = []
    for i in range(vicinity_start_idx, len(x)):
        current_pos = np.array([x[i], y[i]])
        dist = np.linalg.norm(current_pos - loop_start_pos)
        distances_to_start.append(dist)

    if len(distances_to_start) < 7:
        return None

    # Find potential reversal points
    potential_reversals = []
    for i in range(3, len(distances_to_start) - 3):
        current_avg = (distances_to_start[i - 1] * 0.25 +
                      distances_to_start[i] * 0.5 +
                      distances_to_start[i + 1] * 0.25)

        next_points = distances_to_start[i + 1:i + 4]
        if len(next_points) == 3:
            next_avg = sum(next_points) / len(next_points)

            if next_avg > current_avg * 1.001:
                actual_idx = vicinity_start_idx + i
                potential_reversals.append((actual_idx, current_avg, next_avg))

    # Select best reversal point
    if potential_reversals:
        good_reversals = [r for r in potential_reversals
                         if r[1] <= max_distance * 2]

        if good_reversals:
            vicinity_window = min(2000, len(x) - vicinity_start_idx)
            scored_reversals = []

            for reversal in good_reversals:
                idx, distance, next_avg = reversal
                if idx <= vicinity_start_idx + vicinity_window:
                    time_factor = (idx - vicinity_start_idx) / vicinity_window
                    distance_factor = distance / max_distance
                    score = time_factor * 0.7 + distance_factor * 0.3
                    scored_reversals.append((score, reversal))

            if scored_reversals:
                best_score, best_reversal = min(scored_reversals)
                reversal_idx, current_avg, next_avg = best_reversal
                return reversal_idx
            else:
                best_reversal = min(good_reversals, key=lambda x: x[0])
                return best_reversal[0]
        else:
            best_reversal = min(potential_reversals, key=lambda x: x[0])
            return best_reversal[0]

    # Fallback: use minimum distance point
    min_distance = min(distances_to_start)
    min_idx = distances_to_start.index(min_distance)
    return vicinity_start_idx + min_idx


class SegmentType(Enum):
    """Enumeration of course segment types"""
    STRAIGHT = "straight"
    LEFT_TURN = "left_turn"
    RIGHT_TURN = "right_turn"
    S_CURVE_LR = "s_curve_lr"  # Left-to-right
    S_CURVE_RL = "s_curve_rl"  # Right-to-left
    CHICANE = "chicane"


def normalize_angle(angle):
    """
    Normalize angle to range [-pi, pi] radians

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in range [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def angle_difference(a1, a2):
    """
    Calculate smallest difference between two angles

    Args:
        a1: First angle in radians
        a2: Second angle in radians

    Returns:
        Smallest angular difference in radians
    """
    diff = normalize_angle(a2 - a1)
    return diff


def circular_mean(angles):
    """
    Calculate mean of circular quantities (angles)

    Args:
        angles: Array of angles in radians

    Returns:
        Mean angle in radians
    """
    sin_mean = np.mean(np.sin(angles))
    cos_mean = np.mean(np.cos(angles))
    return np.arctan2(sin_mean, cos_mean)


def circular_std(angles):
    """
    Calculate standard deviation of circular quantities (angles)

    Args:
        angles: Array of angles in radians

    Returns:
        Standard deviation in radians
    """
    sin_mean = np.mean(np.sin(angles))
    cos_mean = np.mean(np.cos(angles))
    R = np.sqrt(sin_mean**2 + cos_mean**2)
    # Circular standard deviation
    std_rad = np.sqrt(-2 * np.log(R))
    return std_rad


class MultiLapData:
    """
    Container for raw multi-lap course data

    Loads data from CSV files or Tub directories
    CSV columns: timestamp, x, y, heading (radians)
    Tub fields: _timestamp_ms, car/pos, car/euler (heading derived from euler[2])
    Automatically detects and separates individual laps
    """

    def __init__(self):
        self.raw_data = None
        self.laps = []
        self.num_laps = 0

    def load_data(self, source: str,
                  lap_detection_method: str = 'y_crossing',
                  lap_detection_threshold: float = 2.0,
                  min_lap_length: int = 50,
                  min_loop_distance: float = 1.0,
                  max_closure_distance: float = 1.0,
                  y_threshold: float = 0.1) -> None:
        """
        Load multi-lap data from CSV file or Tub directory

        Args:
            source: Path to CSV file or Tub directory
            lap_detection_method: Method for detecting laps
                                 ('y_crossing', 'drift', 'distance')
            lap_detection_threshold: Distance threshold for lap (meters)
                                   Only used with 'distance' method
            min_lap_length: Minimum number of points per lap
            min_loop_distance: Minimum distance to travel before considering
                             loop closure (meters). Used with 'drift' and
                             'y_crossing' methods
            max_closure_distance: Maximum distance from start for loop
                                closure detection (meters). Only used with
                                'drift' method
            y_threshold: Threshold around zero for y-crossing detection
                        (meters). Only used with 'y_crossing' method
        """
        import os

        if not os.path.exists(source):
            raise FileNotFoundError(f"Source not found: {source}")

        is_csv = os.path.isfile(source) and source.endswith('.csv')
        is_tub = os.path.isdir(source)

        if not is_csv and not is_tub:
            raise ValueError(
                "Source must be CSV file or Tub directory")

        if is_csv:
            self._load_from_csv(source)
        else:
            self._load_from_tub(source)

        # Detect laps using specified method
        self._detect_laps(
            method=lap_detection_method,
            distance_threshold=lap_detection_threshold,
            min_length=min_lap_length,
            min_loop_distance=min_loop_distance,
            max_closure_distance=max_closure_distance,
            y_threshold=y_threshold
        )

        if self.raw_data is None:
            raise ValueError("No data loaded from source")
        logger.info(f"Loaded {len(self.raw_data)} points from {source}")
        logger.info(f"Detected {self.num_laps} laps using '{lap_detection_method}' method")

    def load_csv(self, filepath: str,
                 lap_detection_threshold: float = 2.0,
                 min_lap_length: int = 50) -> None:
        """
        Load multi-lap data from CSV file (backward compatibility)

        Args:
            filepath: Path to CSV file
            lap_detection_threshold: Distance threshold for lap (meters)
            min_lap_length: Minimum number of points per lap
        """
        self.load_data(
            filepath,
            lap_detection_method='y_crossing',
            lap_detection_threshold=lap_detection_threshold,
            min_lap_length=min_lap_length)

    def _load_from_csv(self, filepath: str) -> None:
        """Load data from CSV file"""
        try:
            self.raw_data = np.genfromtxt(
                filepath, delimiter=',',
                names=True, dtype=None, encoding='utf-8')
        except Exception as e:
            logger.error(f"Failed to load CSV file: {e}")
            raise

        # Check required columns
        required_cols = ['timestamp', 'x', 'y', 'heading']
        dtype_names = self.raw_data.dtype.names or ()
        for col in required_cols:
            if col not in dtype_names:
                raise ValueError(f"Missing required column: {col}")

    def _load_from_tub(self, tub_path: str) -> None:
        """Load data from Tub directory"""
        from donkeycar.parts.tub_v2 import Tub

        tub = Tub(tub_path, read_only=True)

        # Extract data from tub records
        data_rows = []
        for record in tub:
            # Get timestamp in seconds
            timestamp = record.get('_timestamp_ms', 0) / 1000.0

            # Get position (car/pos is [x, y, z])
            pos = record.get('car/pos')
            if pos is None or len(pos) < 2:
                continue
            x, y = pos[0], pos[1]

            # Calculate heading from euler angles (car/euler is [x, y, z] in degrees)
            euler = record.get('car/euler', [0, 0, 0])
            heading_deg = 90.0 - euler[2]
            heading = math.radians(heading_deg)

            data_rows.append((timestamp, x, y, heading))

        tub.close()

        if not data_rows:
            raise ValueError(f"No IMU path data found in Tub: {tub_path}")

        # Convert to structured array matching CSV format
        self.raw_data = np.array(
            data_rows,
            dtype=[('timestamp', 'f8'), ('x', 'f8'),
                   ('y', 'f8'), ('heading', 'f8')])

    def _detect_laps(self, method: str = 'y_crossing',
                     distance_threshold: float = 2.0,
                     min_length: int = 50,
                     min_loop_distance: float = 5.0,
                     max_closure_distance: float = 1.0,
                     y_threshold: float = 0.1) -> None:
        """
        Detect individual laps from continuous data using specified method

        Args:
            method: Detection method ('y_crossing', 'drift', or 'distance')
            distance_threshold: Distance threshold for 'distance' method
            min_length: Minimum points per lap
            min_loop_distance: Minimum distance for 'drift' and 'y_crossing'
            max_closure_distance: Max closure distance for 'drift' method
            y_threshold: Y-coordinate threshold for 'y_crossing' method
        """
        if self.raw_data is None:
            raise ValueError("No raw data available for lap detection")

        if len(self.raw_data) < min_length:
            logger.warning("Data too short for lap detection")
            return

        # Dispatch to appropriate detection method
        if method == 'y_crossing':
            self._detect_laps_y_crossing(min_loop_distance, y_threshold)
        elif method == 'drift':
            self._detect_laps_drift_correction(
                min_loop_distance, max_closure_distance)
        elif method == 'distance':
            self._detect_laps_distance(distance_threshold, min_length)
        else:
            raise ValueError(
                f"Unknown lap detection method: {method}. "
                f"Use 'y_crossing', 'drift', or 'distance'")

        # Validate detected laps
        self._validate_laps(
            min_lap_distance=min_loop_distance,
            min_points_per_lap=min_length)

    def _detect_laps_distance(self, threshold: float,
                              min_length: int) -> None:
        """
        Detect laps using simple distance threshold method (legacy)

        This is the original naive method that can produce false positives.

        Args:
            threshold: Distance threshold for lap closure (meters)
            min_length: Minimum points per lap
        """
        x = self.raw_data['x']
        y = self.raw_data['y']

        start_x, start_y = x[0], y[0]
        lap_indices = [0]

        for i in range(min_length, len(x)):
            dist = np.sqrt((x[i] - start_x)**2 + (y[i] - start_y)**2)

            if dist < threshold and (i - lap_indices[-1]) >= min_length:
                lap_indices.append(i)

        if lap_indices[-1] != len(x) - 1:
            lap_indices.append(len(x))

        self._create_laps_from_indices(lap_indices, min_length)

    def _detect_laps_y_crossing(self, min_loop_distance: float = 5.0,
                                y_threshold: float = 0.1) -> None:
        """
        Detect laps using y-coordinate zero crossing

        Assumes the start/finish line is at y=0 and detects lap completion
        when the y-coordinate crosses back through zero.

        Args:
            min_loop_distance: Minimum distance to travel before considering
                             loop closure (meters)
            y_threshold: Threshold around zero for detecting crossing (meters)
        """
        x = self.raw_data['x']
        y = self.raw_data['y']

        lap_indices = [0]
        current_start = 0
        max_loops = 100

        logger.info(f"Detecting laps with y-crossing algorithm")
        logger.info(f"  min_loop_distance={min_loop_distance}m")
        logger.info(f"  Starting y value: {y[0]:.3f}m")
        logger.info(f"  Y range: [{y.min():.3f}, {y.max():.3f}]m")

        for loop_num in range(1, max_loops + 1):
            loop_end = find_loop_end_index_y_crossing(
                x, y, current_start, min_loop_distance, y_threshold)

            if loop_end is None:
                if loop_num == 1:
                    logger.warning("No loops detected in data")
                    logger.warning(f"  Data has {len(y)} points")
                    logger.warning(f"  Y values around start: {y[0:10]}")
                else:
                    logger.info(f"Found {loop_num - 1} laps total")
                break

            logger.info(f"Lap {loop_num} end found at index {loop_end}, "
                       f"y={y[loop_end]:.3f}m")
            lap_indices.append(loop_end)
            current_start = loop_end

        # Create laps from detected indices
        self._create_laps_from_indices(lap_indices, min_length=0)

    def _detect_laps_drift_correction(self, min_loop_distance: float = 5.0,
                                       max_distance: float = 1.0) -> None:
        """
        Detect laps using drift correction algorithm

        Uses weighted average reversal point detection for accurate loop
        closure identification. Based on the algorithm from
        imu_visualization.py.

        Args:
            min_loop_distance: Minimum distance to travel before considering
                             loop closure (meters)
            max_distance: Maximum distance from origin to start looking for
                        reversal point (meters)
        """
        x = self.raw_data['x']
        y = self.raw_data['y']

        lap_indices = [0]
        current_start = 0
        max_loops = 100

        logger.info(f"Detecting laps with drift correction algorithm")
        logger.info(f"  min_loop_distance={min_loop_distance}m, "
                   f"max_distance={max_distance}m")

        for loop_num in range(1, max_loops + 1):
            loop_end = find_loop_end_index(
                x, y, current_start, min_loop_distance, max_distance)

            if loop_end is None:
                if loop_num == 1:
                    logger.warning("No loops detected in data")
                else:
                    logger.info(f"Found {loop_num - 1} laps total")
                break

            logger.debug(f"Loop {loop_num} end found at index {loop_end}")
            lap_indices.append(loop_end)
            current_start = loop_end

        # Create laps from detected indices (don't add final index,
        # find_loop_end_index already found the ends)
        self._create_laps_from_indices(lap_indices, min_length=0)

    def _create_laps_from_indices(self, lap_indices: List[int],
                                   min_length: int = 0) -> None:
        """
        Create lap data arrays from list of lap boundary indices

        Args:
            lap_indices: List of indices marking lap boundaries
            min_length: Minimum points per lap (0 to skip check)
        """
        self.laps = []

        for i in range(len(lap_indices) - 1):
            start_idx = lap_indices[i]
            end_idx = lap_indices[i + 1]

            if min_length == 0 or (end_idx - start_idx >= min_length):
                lap_data = self.raw_data[start_idx:end_idx]
                self.laps.append(lap_data)
                logger.debug(f"Lap {i+1}: indices {start_idx} to {end_idx} ({end_idx - start_idx} points)")
            else:
                logger.debug(f"Lap {i+1}: SKIPPED (too short: {end_idx - start_idx} < {min_length})")

        self.num_laps = len(self.laps)
        logger.debug(f"Created {self.num_laps} laps from indices")

    def _validate_laps(self, min_lap_distance: float = 3.0,
                       max_lap_distance: float = 100.0,
                       min_points_per_lap: int = 100) -> None:
        """
        Validate detected laps and filter out invalid ones

        Args:
            min_lap_distance: Minimum valid lap distance (meters)
            max_lap_distance: Maximum valid lap distance (meters)
            min_points_per_lap: Minimum data points per lap
        """
        if not self.laps:
            return

        valid_laps = []
        initial_count = len(self.laps)

        logger.debug(f"Validating {initial_count} laps")
        logger.debug(f"  min_lap_distance: {min_lap_distance}m, "
                    f"max_lap_distance: {max_lap_distance}m, "
                    f"min_points_per_lap: {min_points_per_lap}")

        for lap_num, lap in enumerate(self.laps, 1):
            # Check point count
            if len(lap) < min_points_per_lap:
                logger.debug(
                    f"Lap {lap_num} rejected: too few points ({len(lap)})")
                continue

            # Calculate total distance traveled
            dx = np.diff(lap['x'])
            dy = np.diff(lap['y'])
            distances = np.sqrt(dx**2 + dy**2)
            total_distance = np.sum(distances)

            # Check distance bounds
            if total_distance < min_lap_distance:
                logger.debug(
                    f"Lap {lap_num} rejected: too short "
                    f"({total_distance:.1f}m)")
                continue

            if total_distance > max_lap_distance:
                logger.debug(
                    f"Lap {lap_num} rejected: too long "
                    f"({total_distance:.1f}m)")
                continue

            # For y_crossing method, we don't check closure since laps are
            # segments between y=0 crossings, not closed loops

            logger.debug(f"Lap {lap_num} valid: {len(lap)} points, {total_distance:.1f}m")
            valid_laps.append(lap)

        logger.info(
            f"Lap validation: {len(valid_laps)}/{initial_count} laps valid")

        self.laps = valid_laps
        self.num_laps = len(valid_laps)

    def get_laps(self) -> List[np.ndarray]:
        """
        Get list of individual laps

        Returns:
            List of numpy structured arrays, one per lap
        """
        return self.laps


class MeanCourse:
    """
    Reconstructed mean course from multi-lap data

    Aligns multiple laps, removes outliers, and computes average trajectory
    """

    def __init__(self, multilap_data: Optional[MultiLapData] = None,
                 params: Optional[Dict[str, Any]] = None):
        """
        Initialize mean course calculator

        Args:
            multilap_data: MultiLapData object with lap data
            params: Dictionary of processing parameters
        """
        self.multilap_data = multilap_data

        # Default parameters
        self.params = {
            'resampling_interval': 0.1,  # meters
            'outlier_std_threshold': 2.5,
            'outlier_iterations': 2,
            'heading_smoothing_window': 5,
            'position_smoothing_window': 11,
            'position_polynomial_order': 3,
        }

        if params is not None:
            self.params.update(params)

        # Output arrays
        self.x = None
        self.y = None
        self.heading = None
        self.distance = None
        self.num_laps = 0
        self.metadata = {}

    def compute(self) -> None:
        """
        Compute mean course by resampling every lap onto a common normalized
        arc-length axis and then averaging with incremental weights so all
        laps contribute equally. This produces a smooth, interpolated course
        rather than snapping to nearest discrete points.
        """
        if self.multilap_data is None or self.multilap_data.num_laps == 0:
            raise ValueError("No lap data available")

        laps = self.multilap_data.get_laps()
        self.num_laps = len(laps)

        if self.num_laps < 1:
            raise ValueError("Need at least 1 lap for mean course")

        resampled_laps = self._resample_laps_uniform(laps)
        self._compute_weighted_mean(resampled_laps)
        self._apply_smoothing()
        self._apply_loop_closure()
        self._compute_metadata_from_resampled(resampled_laps)
        self._compute_distance()

        logger.info(f"Computed mean course from {self.num_laps} laps")
        logger.info(f"Course length: {self.distance[-1]:.2f} meters")

    def _resample_laps_uniform(self, laps: List[np.ndarray]) -> List[Dict[str, np.ndarray]]:
        """
        Resample every lap onto a shared normalized arc-length axis so that
        corresponding indices represent the same progress around the course.
        """
        interval = max(self.params.get('resampling_interval', 0.05), 1e-3)
        lap_lengths: List[float] = []
        lap_samples: List[Dict[str, np.ndarray]] = []

        for lap in laps:
            x = lap['x'].astype(np.float64)
            y = lap['y'].astype(np.float64)
            heading = lap['heading'].astype(np.float64)

            if len(x) < 2:
                lap_lengths.append(0.0)
                lap_samples.append({'x': x, 'y': y,
                                    'heading': heading,
                                    's_norm': np.zeros_like(x)})
                continue

            dx = np.diff(x)
            dy = np.diff(y)
            ds = np.sqrt(dx**2 + dy**2)
            s = np.concatenate([[0], np.cumsum(ds)])
            length = s[-1]
            lap_lengths.append(length if length > 0 else 0.0)
            s_norm = s / length if length > 0 else np.linspace(0, 1, len(x))

            lap_samples.append({'x': x, 'y': y,
                                'heading': heading,
                                's_norm': s_norm})

        reference_length = np.mean([L for L in lap_lengths if L > 0]) \
            if any(L > 0 for L in lap_lengths) else 1.0
        num_samples = max(2, int(reference_length / interval) + 1)
        s_ref = np.linspace(0.0, 1.0, num_samples)

        resampled: List[Dict[str, np.ndarray]] = []

        for sample in lap_samples:
            if len(sample['x']) == 0:
                continue
            s_norm = sample['s_norm']

            x_interp = np.interp(s_ref, s_norm, sample['x'])
            y_interp = np.interp(s_ref, s_norm, sample['y'])

            sin_h = np.sin(sample['heading'])
            cos_h = np.cos(sample['heading'])
            sin_interp = np.interp(s_ref, s_norm, sin_h)
            cos_interp = np.interp(s_ref, s_norm, cos_h)
            heading_interp = np.arctan2(sin_interp, cos_interp)

            resampled.append({
                'x': x_interp,
                'y': y_interp,
                'heading': heading_interp
            })

        if not resampled:
            raise ValueError("Failed to resample laps; check data quality")

        return resampled

    def _compute_weighted_mean(self, laps: List[Dict[str, np.ndarray]]) -> None:
        """Incrementally average resampled laps with equal weights."""
        self.x = laps[0]['x'].copy()
        self.y = laps[0]['y'].copy()
        sin_mean = np.sin(laps[0]['heading'])
        cos_mean = np.cos(laps[0]['heading'])

        for idx, lap in enumerate(laps[1:], start=2):
            new_weight = 1.0 / idx
            prev_weight = 1.0 - new_weight
            self.x = prev_weight * self.x + new_weight * lap['x']
            self.y = prev_weight * self.y + new_weight * lap['y']
            sin_mean = prev_weight * sin_mean + new_weight * np.sin(lap['heading'])
            cos_mean = prev_weight * cos_mean + new_weight * np.cos(lap['heading'])

        self.heading = np.arctan2(sin_mean, cos_mean)

    def _compute_metadata_from_resampled(self, laps: List[Dict[str, np.ndarray]]) -> None:
        """Compute per-point standard deviation across resampled laps."""
        if not laps:
            self.metadata = {}
            return

        stack_x = np.array([lap['x'] for lap in laps], dtype=np.float64)
        stack_y = np.array([lap['y'] for lap in laps], dtype=np.float64)
        stack_sin = np.sin(np.array([lap['heading'] for lap in laps]))
        stack_cos = np.cos(np.array([lap['heading'] for lap in laps]))

        self.metadata['x_std'] = np.std(stack_x, axis=0)
        self.metadata['y_std'] = np.std(stack_y, axis=0)
        mean_heading = np.arctan2(stack_sin, stack_cos)
        heading_diff = np.unwrap(mean_heading, axis=0)
        self.metadata['heading_std'] = np.std(heading_diff, axis=0)

    def _apply_smoothing(self) -> None:
        """
        Apply smoothing to reduce high-frequency noise
        """
        pos_window = self.params['position_smoothing_window']
        pos_order = self.params['position_polynomial_order']
        head_window = self.params['heading_smoothing_window']

        # Ensure window size is valid for position smoothing
        data_len = len(self.x)

        # Adjust polynomial order if data is too short
        if data_len < pos_order + 2:
            pos_order = max(1, data_len - 2)

        # Adjust window size
        if pos_window > data_len:
            pos_window = data_len if data_len % 2 == 1 else data_len - 1
        if pos_window % 2 == 0:
            pos_window -= 1

        # Ensure window is large enough for polynomial order
        min_window = pos_order + 2
        if pos_window < min_window:
            pos_window = min_window
            if pos_window % 2 == 0:
                pos_window += 1

        # Final check: window must not exceed data length
        if pos_window > data_len:
            pos_window = data_len if data_len % 2 == 1 else data_len - 1

        if head_window > len(self.heading):
            head_window = len(self.heading)
        if head_window % 2 == 0:
            head_window -= 1

        # Savitzky-Golay filter for positions
        # Only apply if we have enough data points
        if data_len >= pos_order + 2 and pos_window >= pos_order + 2 and pos_window <= data_len:
            self.x = savgol_filter(self.x, pos_window, pos_order)
            self.y = savgol_filter(self.y, pos_window, pos_order)

        # Moving average for heading (circular, already in radians)
        if head_window >= 3:
            sin_h = np.sin(self.heading)
            cos_h = np.cos(self.heading)

            # Convolve with uniform kernel
            kernel = np.ones(head_window) / head_window
            sin_h_smooth = np.convolve(sin_h, kernel, mode='same')
            cos_h_smooth = np.convolve(cos_h, kernel, mode='same')

            self.heading = np.arctan2(sin_h_smooth, cos_h_smooth)

    def _apply_loop_closure(self) -> None:
        """
        Apply loop closure to glue start and end of course together.

        For closed-loop courses, this ensures the start and end connect
        smoothly at y=0 by applying a gradual correction to the first
        and last 2.5% of points.
        """
        if self.x is None or self.y is None or len(self.x) < 10:
            return

        # Calculate midpoint x between start and end
        x_mid = (self.x[0] + self.x[-1]) / 2.0

        # Calculate how far start and end are from y=0
        y_start_error = self.y[0]
        y_end_error = self.y[-1]

        # Calculate x errors from midpoint
        x_start_error = self.x[0] - x_mid
        x_end_error = self.x[-1] - x_mid

        # Determine number of points to correct (2.5% at each end)
        num_points = len(self.x)
        correction_length = max(1, int(num_points * 0.025))

        # Apply gradual correction to first 2.5% of points
        for i in range(correction_length):
            factor = 1.0 - (i / correction_length)
            self.x[i] -= x_start_error * factor
            self.y[i] -= y_start_error * factor

        # Apply gradual correction to last 2.5% of points
        for i in range(correction_length):
            idx = num_points - 1 - i
            factor = 1.0 - (i / correction_length)
            self.x[idx] -= x_end_error * factor
            self.y[idx] -= y_end_error * factor

        logger.debug(
            f"Applied loop closure: x_mid={x_mid:.3f}, "
            f"y_errors=[{y_start_error:.3f}, {y_end_error:.3f}]")

    @staticmethod
    def _blend_heading(current: float, new_value: float,
                       new_weight: float) -> float:
        """Blend two headings using circular interpolation."""
        if new_weight <= 0.0:
            return current
        prev_weight = 1.0 - new_weight
        sin_val = prev_weight * np.sin(current) + new_weight * np.sin(new_value)
        cos_val = prev_weight * np.cos(current) + new_weight * np.cos(new_value)
        if sin_val == 0 and cos_val == 0:
            return current
        return np.arctan2(sin_val, cos_val)

    def _compute_metadata_from_laps(self, laps: List[np.ndarray]) -> None:
        """Compute spread statistics for diagnostics."""
        if not laps:
            self.metadata = {}
            return

        mean_points = np.column_stack((self.x, self.y))
        tree = KDTree(mean_points)
        num_points = len(self.x)

        x_var = np.zeros(num_points, dtype=np.float64)
        y_var = np.zeros(num_points, dtype=np.float64)
        heading_var = np.zeros(num_points, dtype=np.float64)
        counts = np.zeros(num_points, dtype=np.int32)

        for lap in laps:
            for x_val, y_val, heading_val in zip(
                    lap['x'], lap['y'], lap['heading']):
                _, idx = tree.query([x_val, y_val])
                dx = x_val - self.x[idx]
                dy = y_val - self.y[idx]
                heading_diff = angle_difference(self.heading[idx], heading_val)
                x_var[idx] += dx * dx
                y_var[idx] += dy * dy
                heading_var[idx] += heading_diff * heading_diff
                counts[idx] += 1

        x_std = np.zeros(num_points, dtype=np.float64)
        y_std = np.zeros(num_points, dtype=np.float64)
        heading_std = np.zeros(num_points, dtype=np.float64)

        nonzero = counts > 0
        x_std[nonzero] = np.sqrt(x_var[nonzero] / counts[nonzero])
        y_std[nonzero] = np.sqrt(y_var[nonzero] / counts[nonzero])
        heading_std[nonzero] = np.sqrt(heading_var[nonzero] / counts[nonzero])

        self.metadata['x_std'] = x_std
        self.metadata['y_std'] = y_std
        self.metadata['heading_std'] = heading_std

    def _compute_distance(self) -> None:
        """
        Compute cumulative arc-length distance along course
        """
        dx = np.diff(self.x)
        dy = np.diff(self.y)
        ds = np.sqrt(dx**2 + dy**2)
        self.distance = np.concatenate([[0], np.cumsum(ds)])

    def save(self, filepath: str) -> None:
        """
        Save mean course to file

        Args:
            filepath: Output file path (CSV or JSON)
        """
        if (self.x is None or self.y is None or
                self.heading is None or self.distance is None):
            raise ValueError("Mean course data not computed; call compute() "
                             "before saving.")

        x = self.x
        y = self.y
        heading = self.heading
        distance = self.distance

        if filepath.endswith('.json'):
            # Save as JSON
            data = {
                'x': x.tolist(),
                'y': y.tolist(),
                'heading': heading.tolist(),
                'distance': distance.tolist(),
                'num_laps': self.num_laps,
                'params': self.params,
                'metadata': {
                    'x_std': self.metadata.get('x_std', []).tolist() if 'x_std' in self.metadata else [],
                    'y_std': self.metadata.get('y_std', []).tolist() if 'y_std' in self.metadata else [],
                    'heading_std': self.metadata.get('heading_std', []).tolist() if 'heading_std' in self.metadata else [],
                }
            }

            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
        else:
            # Save as CSV
            header = "x,y,heading,distance"
            data = np.column_stack([self.x, self.y, self.heading, self.distance])
            np.savetxt(filepath, data, delimiter=',', header=header, comments='')

        logger.info(f"Saved mean course to {filepath}")

    def load(self, filepath: str) -> None:
        """
        Load mean course from file

        Args:
            filepath: Input file path (CSV or JSON)
        """
        if filepath.endswith('.json'):
            # Load from JSON
            with open(filepath, 'r') as f:
                data = json.load(f)

            self.x = np.array(data['x'])
            self.y = np.array(data['y'])
            self.heading = np.array(data['heading'])
            self.distance = np.array(data['distance'])
            self.num_laps = data.get('num_laps', 0)
            self.params = data.get('params', {})

            metadata = data.get('metadata', {})
            self.metadata = {
                'x_std': np.array(metadata.get('x_std', [])),
                'y_std': np.array(metadata.get('y_std', [])),
                'heading_std': np.array(metadata.get('heading_std', []))
            }
        else:
            # Load from CSV
            data = np.genfromtxt(filepath, delimiter=',', names=True)
            self.x = data['x']
            self.y = data['y']
            self.heading = data['heading']
            self.distance = data['distance']
            self.num_laps = 0
            self.metadata = {}

        logger.info(f"Loaded mean course from {filepath}")


class Segment:
    """
    Individual course segment with geometric properties
    """

    def __init__(self, segment_id: int, segment_type: SegmentType,
                 start_index: int, end_index: int,
                 x: np.ndarray, y: np.ndarray, heading: np.ndarray,
                 distance: np.ndarray, curvature: np.ndarray):
        """
        Initialize segment

        Args:
            segment_id: Integer segment identifier
            segment_type: SegmentType enum
            start_index: Start index in mean course arrays
            end_index: End index in mean course arrays
            x, y: Position arrays for this segment
            heading: Heading array for this segment
            distance: Distance array for this segment
            curvature: Curvature array for this segment
        """
        self.segment_id = segment_id
        self.segment_type = segment_type
        self.start_index = start_index
        self.end_index = end_index

        # Segment data
        self.x = x
        self.y = y
        self.heading = heading
        self.distance = distance
        self.curvature = curvature

        # Compute metrics
        self.length = distance[-1] - distance[0]
        self.mean_curvature = np.mean(curvature)
        self.max_curvature = np.max(np.abs(curvature))
        self.total_heading_change = np.abs(heading[-1] - heading[0])
        self.entry_heading = heading[0]
        self.exit_heading = heading[-1]

    def __repr__(self):
        return (f"Segment(id={self.segment_id}, type={self.segment_type.value}, "
                f"length={self.length:.2f}m, curvature={self.mean_curvature:.3f})")


class CourseSegmentation:
    """
    Segmented course with classified segments

    Analyzes mean course to identify and classify geometric segments
    """

    def __init__(self, mean_course: Optional[MeanCourse] = None,
                 params: Optional[Dict[str, Any]] = None):
        """
        Initialize course segmentation

        Args:
            mean_course: MeanCourse object
            params: Dictionary of segmentation parameters
        """
        self.mean_course = mean_course

        # Default parameters
        self.params = {
            'curvature_window': 5,
            'curvature_smoothing_window': 21,
            'straight_curvature_threshold': 0.08,  # rad/m
            'min_segment_length': 0.8,  # meters
            'inflection_threshold': 0.05,  # rad/m
            'classification_window': 5,  # points for type classification
            'boundary_method': 'hybrid',  # 'threshold', 'extrema', 'gradient', 'hybrid'
            'gradient_prominence': 0.1,  # Minimum prominence for gradient peaks
        }

        if params is not None:
            self.params.update(params)

        self.segments: List[Segment] = []
        self.total_segments = 0
        self.segment_counts: Dict[SegmentType, int] = {}

    def compute(self, use_adaptive_threshold: bool = True) -> None:
        """
        Compute course segmentation

        Process:
        1. Calculate curvature
        2. (Optional) Compute adaptive threshold
        3. Detect segment boundaries
        4. Classify segments
        5. Create Segment objects

        Args:
            use_adaptive_threshold: If True, automatically adjust the
                                   straight_curvature_threshold based on
                                   the curvature distribution (default: True)
        """
        if self.mean_course is None:
            raise ValueError("No mean course available")

        # Step 1: Calculate curvature
        curvature = self._calculate_curvature()

        # Step 2: Optionally compute adaptive threshold
        if use_adaptive_threshold:
            self._compute_adaptive_threshold(curvature)

        # Step 3: Detect segment boundaries
        boundaries = self._detect_boundaries(curvature)

        # Step 4: Classify and create segments
        self._create_segments(boundaries, curvature)

        # Step 5: Merge adjacent segments of same type
        self._merge_adjacent_segments()

        # Step 6: Count segment types
        self._count_segments()

        logger.info(f"Segmented course into {self.total_segments} segments")
        logger.info(f"Segment counts: {self.segment_counts}")

    def _calculate_curvature(self) -> np.ndarray:
        """
        Calculate path curvature at each point

        Returns:
            Array of curvature values (rad/m)
        """
        x = self.mean_course.x
        y = self.mean_course.y
        heading = self.mean_course.heading  # Already in radians
        distance = self.mean_course.distance

        # Calculate curvature using finite differences
        # κ = dθ/ds (radians/meter)
        window = self.params['curvature_window']

        curvature = np.zeros(len(heading))

        for i in range(window, len(heading) - window):
            # Central difference (in radians)
            dtheta = normalize_angle(
                heading[i + window] - heading[i - window])
            ds = distance[i + window] - distance[i - window]

            if ds > 0:
                curvature[i] = dtheta / ds

        # Handle edges
        curvature[:window] = curvature[window]
        curvature[-window:] = curvature[-window - 1]

        # Apply smoothing
        smooth_window = self.params['curvature_smoothing_window']
        if smooth_window > len(curvature):
            smooth_window = len(curvature)
        if smooth_window % 2 == 0:
            smooth_window -= 1
        if smooth_window >= 3:
            kernel = np.ones(smooth_window) / smooth_window
            curvature = np.convolve(curvature, kernel, mode='same')

        return curvature

    def _compute_adaptive_threshold(self, curvature: np.ndarray) -> None:
        """
        Compute adaptive threshold based on curvature distribution.

        Uses a gap-based approach to find natural separation between
        straight and curved sections. Looks for the largest gap in the
        curvature distribution and sets threshold there.

        Args:
            curvature: Array of curvature values
        """
        abs_curvature = np.abs(curvature)

        # Sort curvature values to find gaps
        sorted_curv = np.sort(abs_curvature)

        # Find the largest gap in the lower 50% of values
        # This helps distinguish straights from curves even in highly
        # curved tracks
        midpoint = len(sorted_curv) // 2
        lower_half = sorted_curv[:midpoint]

        if len(lower_half) > 1:
            # Calculate gaps between consecutive values
            gaps = np.diff(lower_half)

            # Find the largest gap
            max_gap_idx = np.argmax(gaps)

            # Set threshold at midpoint of largest gap
            threshold = (lower_half[max_gap_idx] +
                        lower_half[max_gap_idx + 1]) / 2.0
        else:
            # Fallback: use 20th percentile
            threshold = np.percentile(abs_curvature, 20)

        # Ensure threshold is reasonable (between 0.05 and 2.0 rad/m)
        threshold = np.clip(threshold, 0.05, 2.0)

        # Additional check: if threshold would classify >90% as curves,
        # use a more aggressive threshold
        points_below = np.sum(abs_curvature < threshold)
        pct_below = points_below / len(abs_curvature)

        if pct_below < 0.05:
            # Less than 5% would be "straight" - too aggressive
            # Use 10th percentile instead
            threshold = np.percentile(abs_curvature, 10)
            threshold = np.clip(threshold, 0.05, 2.0)
            logger.info(f"Adjusted threshold to 10th percentile "
                       f"({threshold:.3f}) to avoid over-classification")

        # Update parameter
        old_threshold = self.params['straight_curvature_threshold']
        self.params['straight_curvature_threshold'] = threshold

        logger.info(f"Adaptive threshold: {threshold:.3f} rad/m "
                   f"(was {old_threshold:.3f} rad/m)")
        logger.info(f"Curvature stats: min={abs_curvature.min():.3f}, "
                   f"max={abs_curvature.max():.3f}, "
                   f"mean={abs_curvature.mean():.3f}, "
                   f"median={np.median(abs_curvature):.3f}, "
                   f"{pct_below*100:.1f}% below threshold")

    def _detect_boundaries(self, curvature: np.ndarray) -> List[int]:
        """
        Detect segment boundaries using configurable method.

        Methods:
        - 'threshold': Detects transitions between straight/left/right
        - 'extrema': Detects local peaks and valleys in curvature
        - 'gradient': Detects where curvature changes most rapidly
        - 'hybrid': Combines threshold + extrema methods

        Args:
            curvature: Array of curvature values

        Returns:
            List of boundary indices
        """
        method = self.params.get('boundary_method', 'hybrid')
        threshold = self.params['straight_curvature_threshold']
        min_length = self.params['min_segment_length']
        distance = self.mean_course.distance

        # Get boundaries based on method
        if method == 'threshold':
            point_types = self._classify_point_types(curvature, threshold)
            all_boundaries = self._find_all_type_changes(point_types)

        elif method == 'extrema':
            point_types = self._classify_point_types(curvature, threshold)
            all_boundaries = self._find_curvature_extrema(curvature)

        elif method == 'gradient':
            point_types = self._classify_point_types(curvature, threshold)
            all_boundaries = self._find_curvature_gradients(curvature)

        else:  # 'hybrid' or unknown -> use hybrid
            point_types = self._classify_point_types(curvature, threshold)
            threshold_boundaries = self._find_all_type_changes(point_types)
            extrema_boundaries = self._find_curvature_extrema(curvature)
            all_boundaries = sorted(set(threshold_boundaries + extrema_boundaries))

        if not all_boundaries:
            return [0]

        # Filter by minimum length
        all_boundaries = self._filter_by_min_length(
            all_boundaries, distance, min_length)

        if not all_boundaries:
            return [0]

        # Handle closed loop wrap-around
        all_boundaries = self._handle_closed_loop(
            all_boundaries, point_types, distance, min_length)

        return all_boundaries

    def _find_curvature_extrema(self, curvature: np.ndarray) -> List[int]:
        """
        Find local extrema (peaks and valleys) in absolute curvature.

        These represent transitions between different curvature levels,
        such as from tight curves to gentle curves or straights.

        Args:
            curvature: Array of curvature values

        Returns:
            List of indices where extrema occur
        """
        from scipy.signal import find_peaks

        abs_curvature = np.abs(curvature)

        # Find peaks (local maxima)
        # Use prominence to filter out minor fluctuations
        mean_curv = np.mean(abs_curvature)
        prominence = max(0.1, mean_curv * 0.2)  # 20% of mean curvature

        peaks, _ = find_peaks(abs_curvature, prominence=prominence)

        # Find valleys (local minima) by inverting
        valleys, _ = find_peaks(-abs_curvature, prominence=prominence)

        # Combine and sort
        extrema = sorted(list(peaks) + list(valleys))

        logger.debug(f"Found {len(peaks)} curvature peaks and "
                    f"{len(valleys)} valleys")

        return extrema

    def _find_curvature_gradients(self, curvature: np.ndarray) -> List[int]:
        """
        Find points where curvature changes most rapidly (gradient peaks).

        This detects entry/exit points of curves rather than apex/valley
        points. Uses the rate of change of curvature: d(kappa)/ds.

        Args:
            curvature: Array of curvature values

        Returns:
            List of indices where curvature gradient is highest
        """
        from scipy.signal import find_peaks

        distance = self.mean_course.distance

        # Calculate curvature gradient (rate of change)
        # d(kappa)/ds where s is arc length
        gradient = np.zeros_like(curvature)

        for i in range(1, len(curvature) - 1):
            dk = curvature[i + 1] - curvature[i - 1]
            ds = distance[i + 1] - distance[i - 1]
            if ds > 0:
                gradient[i] = dk / ds

        # Use absolute gradient to find rapid changes in either direction
        abs_gradient = np.abs(gradient)

        # Find peaks in gradient (points of rapid curvature change)
        mean_grad = np.mean(abs_gradient)
        prominence_param = self.params.get('gradient_prominence', 0.1)
        prominence = max(prominence_param, mean_grad * 0.3)

        peaks, _ = find_peaks(abs_gradient, prominence=prominence)

        logger.debug(f"Found {len(peaks)} curvature gradient peaks "
                    f"(mean gradient: {mean_grad:.4f})")

        return sorted(list(peaks))


    def _classify_point_types(self, curvature: np.ndarray,
                               threshold: float) -> np.ndarray:
        """
        Classify each point as left turn, right turn, or straight.

        Args:
            curvature: Array of curvature values
            threshold: Curvature threshold for straight sections

        Returns:
            Array of point types (-1: right, 0: straight, 1: left)
        """
        window_size = self.params.get('classification_window', 5)
        point_types = np.zeros(len(curvature), dtype=int)

        for i in range(len(curvature)):
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(curvature), i + window_size // 2 + 1)
            window_curv = curvature[start_idx:end_idx]
            avg_curv = np.mean(window_curv)
            point_types[i] = self._classify_curvature(avg_curv, threshold)

        return point_types

    def _classify_curvature(self, avg_curv: float,
                             threshold: float) -> int:
        """
        Classify curvature value as turn type.

        Args:
            avg_curv: Average curvature value
            threshold: Threshold for straight sections

        Returns:
            Turn type (-1: right, 0: straight, 1: left)
        """
        if abs(avg_curv) < threshold:
            return 0
        if avg_curv > 0:
            return 1
        return -1

    def _find_all_type_changes(self, point_types: np.ndarray) -> List[int]:
        """
        Find all indices where point type changes.

        Args:
            point_types: Array of classified point types

        Returns:
            List of indices where type changes
        """
        boundaries = []
        for i in range(1, len(point_types)):
            if point_types[i] != point_types[i - 1]:
                boundaries.append(i)
        return boundaries

    def _filter_by_min_length(self, boundaries: List[int],
                                distance: np.ndarray,
                                min_length: float) -> List[int]:
        """
        Filter out segments shorter than minimum length.

        Args:
            boundaries: List of boundary indices
            distance: Distance array
            min_length: Minimum segment length

        Returns:
            Filtered list of boundaries
        """
        if len(boundaries) < 2:
            return boundaries

        filtered = [boundaries[0]]

        for i in range(1, len(boundaries)):
            seg_start = filtered[-1]
            seg_end = boundaries[i]
            seg_length = distance[seg_end] - distance[seg_start]

            if seg_length >= min_length:
                filtered.append(boundaries[i])

        if len(filtered) < 2:
            return boundaries

        return filtered

    def _handle_closed_loop(self, boundaries: List[int],
                             point_types: np.ndarray,
                             distance: np.ndarray,
                             min_length: float) -> List[int]:
        """
        Handle closed loop wrap-around.

        For closed loops, ensure we don't create duplicate boundaries at
        start/end point (which are the same physical location).

        Args:
            boundaries: List of boundary indices
            point_types: Array of point types
            distance: Distance array
            min_length: Minimum segment length

        Returns:
            Adjusted boundaries for closed loop
        """
        if not boundaries:
            # No boundaries found - treat as single segment
            return [0]

        # For closed loops, we want boundaries but NOT both 0 and N-1
        # since they represent the same physical location.
        # Only add 0 if it's not already there
        result = boundaries[:]
        if 0 not in result:
            result.insert(0, 0)

        # Never add len-1 for closed loops - it duplicates the 0 marker
        # The segments will wrap around naturally

        return result

    def _merge_short_segments(self, boundaries: List[int], min_length: float) -> List[int]:
        """
        Merge segments that are shorter than minimum length

        Args:
            boundaries: List of boundary indices
            min_length: Minimum segment length in meters

        Returns:
            Filtered list of boundaries
        """
        distance = self.mean_course.distance
        merged = [boundaries[0]]

        for i in range(1, len(boundaries) - 1):
            seg_length = distance[boundaries[i + 1]] - distance[merged[-1]]

            # Keep boundary if segment is long enough
            if seg_length >= min_length:
                merged.append(boundaries[i])

        merged.append(boundaries[-1])

        return merged

    def _create_segments(self, boundaries: List[int], curvature: np.ndarray) -> None:
        """
        Create Segment objects with classification

        Args:
            boundaries: List of boundary indices
            curvature: Array of curvature values
        """
        self.segments = []

        # Create segments between consecutive boundaries
        for i in range(len(boundaries) - 1):
            start_idx = boundaries[i]
            end_idx = boundaries[i + 1]

            # Extract segment data
            seg_x = self.mean_course.x[start_idx:end_idx + 1]
            seg_y = self.mean_course.y[start_idx:end_idx + 1]
            seg_heading = self.mean_course.heading[start_idx:end_idx + 1]
            seg_distance = self.mean_course.distance[start_idx:end_idx + 1]
            seg_curvature = curvature[start_idx:end_idx + 1]

            # Classify segment
            seg_type = self._classify_segment(seg_curvature)

            # Create Segment object
            segment = Segment(
                segment_id=i,
                segment_type=seg_type,
                start_index=start_idx,
                end_index=end_idx,
                x=seg_x,
                y=seg_y,
                heading=seg_heading,
                distance=seg_distance,
                curvature=seg_curvature
            )

            self.segments.append(segment)

        # Add wrap-around segment for closed loops (from last boundary to first)
        if len(boundaries) > 0:
            start_idx = boundaries[-1]
            end_idx = len(curvature) - 1

            # Only create wrap segment if there's actual distance to cover
            if end_idx > start_idx:
                # Extract data from last boundary to end of course
                seg_x = self.mean_course.x[start_idx:]
                seg_y = self.mean_course.y[start_idx:]
                seg_heading = self.mean_course.heading[start_idx:]
                seg_distance = self.mean_course.distance[start_idx:]
                seg_curvature = curvature[start_idx:]

                # Classify segment
                seg_type = self._classify_segment(seg_curvature)

                # Create wrap-around Segment object
                segment = Segment(
                    segment_id=len(self.segments),
                    segment_type=seg_type,
                    start_index=start_idx,
                    end_index=end_idx,
                    x=seg_x,
                    y=seg_y,
                    heading=seg_heading,
                    distance=seg_distance,
                    curvature=seg_curvature
                )

                self.segments.append(segment)

        self.total_segments = len(self.segments)

    def _classify_segment(self, curvature: np.ndarray) -> SegmentType:
        """
        Classify segment based on curvature characteristics

        Args:
            curvature: Curvature array for segment

        Returns:
            SegmentType classification
        """
        threshold = self.params['straight_curvature_threshold']
        inflection_threshold = self.params['inflection_threshold']

        mean_curv = np.mean(curvature)

        # Count inflection points
        inflections = 0
        for i in range(1, len(curvature)):
            if curvature[i - 1] * curvature[i] < 0:
                if abs(curvature[i - 1]) > inflection_threshold or abs(curvature[i]) > inflection_threshold:
                    inflections += 1

        # Classify based on curvature and inflections
        if inflections >= 3:
            return SegmentType.CHICANE
        elif inflections == 1:
            # S-curve
            if curvature[0] > 0 and curvature[-1] < 0:
                return SegmentType.S_CURVE_LR
            else:
                return SegmentType.S_CURVE_RL
        elif inflections == 2:
            # Could be S-curve or chicane
            if len(curvature) > 10:
                return SegmentType.CHICANE
            else:
                if abs(mean_curv) < threshold / 2:
                    return SegmentType.S_CURVE_LR  # Default
                elif mean_curv > 0:
                    return SegmentType.S_CURVE_LR
                else:
                    return SegmentType.S_CURVE_RL
        else:
            # No inflections - straight or turn
            if abs(mean_curv) < threshold:
                return SegmentType.STRAIGHT
            elif mean_curv > 0:
                return SegmentType.LEFT_TURN
            else:
                return SegmentType.RIGHT_TURN

    def _merge_adjacent_segments(self) -> None:
        """
        Merge adjacent segments of the same type.

        This reduces over-segmentation by combining consecutive segments
        that have the same classification.
        """
        if len(self.segments) <= 1:
            return

        merged_segments = []
        current_segment = self.segments[0]

        for next_segment in self.segments[1:]:
            # Check if segments are same type and should be merged
            if (current_segment.segment_type == next_segment.segment_type and
                self._should_merge_segments(current_segment, next_segment)):
                # Merge next_segment into current_segment
                current_segment = self._merge_two_segments(
                    current_segment, next_segment)
            else:
                # Different type or shouldn't merge - save current and
                # start new one
                merged_segments.append(current_segment)
                current_segment = next_segment

        # Don't forget the last segment
        merged_segments.append(current_segment)

        # Update segment IDs
        for i, seg in enumerate(merged_segments):
            seg.segment_id = i

        # Update segment list
        old_count = len(self.segments)
        self.segments = merged_segments
        self.total_segments = len(self.segments)

        logger.info(f"Merged {old_count} segments into {self.total_segments}")

    def _should_merge_segments(self, seg1: Segment, seg2: Segment) -> bool:
        """
        Determine if two adjacent segments should be merged.

        Conservative policy: Only merge straight sections to avoid losing
        information about curves. Extrema-based boundaries between curves
        represent meaningful geometry changes (peaks/valleys) and should be
        preserved.

        Args:
            seg1: First segment
            seg2: Second segment (adjacent to seg1)

        Returns:
            True if segments should be merged
        """
        # Don't merge if types are different
        if seg1.segment_type != seg2.segment_type:
            return False

        # Only merge straights - preserve all curve boundaries from extrema
        # detection
        if seg1.segment_type == SegmentType.STRAIGHT:
            return True

        # Don't merge curves - extrema boundaries represent meaningful
        # geometric features
        return False

    def _merge_two_segments(self, seg1: Segment, seg2: Segment) -> Segment:
        """
        Merge two adjacent segments into one.

        Args:
            seg1: First segment
            seg2: Second segment (must be adjacent)

        Returns:
            New merged segment
        """
        # Combine arrays
        merged_x = np.concatenate([seg1.x, seg2.x[1:]])  # Skip duplicate point
        merged_y = np.concatenate([seg1.y, seg2.y[1:]])
        merged_heading = np.concatenate([seg1.heading, seg2.heading[1:]])
        merged_distance = np.concatenate([seg1.distance, seg2.distance[1:]])
        merged_curvature = np.concatenate([seg1.curvature, seg2.curvature[1:]])

        # Create new merged segment
        merged = Segment(
            segment_id=seg1.segment_id,
            segment_type=seg1.segment_type,
            start_index=seg1.start_index,
            end_index=seg2.end_index,
            x=merged_x,
            y=merged_y,
            heading=merged_heading,
            distance=merged_distance,
            curvature=merged_curvature
        )

        return merged

    def _count_segments(self) -> None:
        """
        Count segments by type
        """
        self.segment_counts = {}

        for segment in self.segments:
            seg_type = segment.segment_type
            self.segment_counts[seg_type] = self.segment_counts.get(seg_type, 0) + 1

    def get_segment(self, segment_id: int) -> Optional[Segment]:
        """
        Get segment by ID

        Args:
            segment_id: Segment identifier

        Returns:
            Segment object or None if not found
        """
        if 0 <= segment_id < len(self.segments):
            return self.segments[segment_id]
        return None

    def save(self, filepath: str) -> None:
        """
        Save segmentation to JSON file

        Args:
            filepath: Output file path
        """
        data = {
            'total_segments': self.total_segments,
            'segment_counts': {k.value: v for k, v in self.segment_counts.items()},
            'segments': [
                {
                    'segment_id': seg.segment_id,
                    'segment_type': seg.segment_type.value,
                    'start_index': seg.start_index,
                    'end_index': seg.end_index,
                    'length': seg.length,
                    'mean_curvature': seg.mean_curvature,
                    'max_curvature': seg.max_curvature,
                    'total_heading_change': seg.total_heading_change,
                    'entry_heading': seg.entry_heading,
                    'exit_heading': seg.exit_heading,
                }
                for seg in self.segments
            ]
        }

        with open(filepath, 'w') as f:
            json.dump(convert_numpy_types(data), f, indent=2)

        logger.info(f"Saved segmentation to {filepath}")

    def load(self, filepath: str, mean_course: MeanCourse) -> None:
        """
        Load segmentation from JSON file

        Args:
            filepath: Input file path
            mean_course: Associated MeanCourse object
        """
        self.mean_course = mean_course

        with open(filepath, 'r') as f:
            data = json.load(f)

        self.total_segments = data['total_segments']
        self.segment_counts = {SegmentType(k): v for k, v in data['segment_counts'].items()}

        # Reconstruct segments
        self.segments = []
        curvature = self._calculate_curvature()  # Recompute curvature

        for seg_data in data['segments']:
            start_idx = seg_data['start_index']
            end_idx = seg_data['end_index']

            segment = Segment(
                segment_id=seg_data['segment_id'],
                segment_type=SegmentType(seg_data['segment_type']),
                start_index=start_idx,
                end_index=end_idx,
                x=mean_course.x[start_idx:end_idx + 1],
                y=mean_course.y[start_idx:end_idx + 1],
                heading=mean_course.heading[start_idx:end_idx + 1],
                distance=mean_course.distance[start_idx:end_idx + 1],
                curvature=curvature[start_idx:end_idx + 1]
            )

            self.segments.append(segment)

        logger.info(f"Loaded segmentation from {filepath}")


class SegmentEstimate:
    """
    Result of segment estimation
    """

    def __init__(self, segment_id: Optional[int], confidence: float,
                 distance_to_course: float, heading_error: float,
                 course_position: Tuple[float, float],
                 course_heading: float):
        """
        Initialize segment estimate

        Args:
            segment_id: Estimated segment ID (None if off-track)
            confidence: Confidence score [0, 1]
            distance_to_course: Distance from vehicle to course (meters)
            heading_error: Difference between vehicle and course heading (degrees)
            course_position: Matched (x, y) position on course
            course_heading: Expected heading at matched position
        """
        self.segment_id = segment_id
        self.confidence = confidence
        self.distance_to_course = distance_to_course
        self.heading_error = heading_error
        self.course_position = course_position
        self.course_heading = course_heading

    def __repr__(self):
        return (f"SegmentEstimate(segment={self.segment_id}, "
                f"confidence={self.confidence:.2f}, "
                f"distance={self.distance_to_course:.2f}m)")


class SegmentEstimator:
    """
    Real-time segment estimation from vehicle position/heading

    Uses spatial indexing for efficient nearest-neighbor search
    """

    def __init__(self, course_segmentation: CourseSegmentation,
                 params: Optional[Dict[str, Any]] = None):
        """
        Initialize segment estimator

        Args:
            course_segmentation: CourseSegmentation object
            params: Dictionary of estimator parameters
        """
        self.segmentation = course_segmentation
        self.mean_course = course_segmentation.mean_course

        # Default parameters
        self.params = {
            'search_radius': 5.0,  # meters
            'position_tolerance': 2.0,  # meters
            'heading_tolerance': 45.0,  # degrees
            'heading_weight': 0.3,  # weight for heading in distance metric
        }

        if params is not None:
            self.params.update(params)

        # Build KD-tree for fast spatial search
        points = np.column_stack([self.mean_course.x, self.mean_course.y])
        self.kdtree = KDTree(points)

        # Map indices to segments
        self._build_index_to_segment_map()

    def _build_index_to_segment_map(self) -> None:
        """
        Build mapping from course indices to segment IDs
        """
        self.index_to_segment = np.zeros(len(self.mean_course.x), dtype=int)

        for segment in self.segmentation.segments:
            self.index_to_segment[segment.start_index:segment.end_index + 1] = segment.segment_id

    def estimate(self, x: float, y: float, heading: float,
                 last_position: Optional[Tuple[float, float]] = None) -> SegmentEstimate:
        """
        Estimate current segment from vehicle state

        Args:
            x: Vehicle X position (meters)
            y: Vehicle Y position (meters)
            heading: Vehicle heading (degrees)
            last_position: Optional last known position for incremental search

        Returns:
            SegmentEstimate object
        """
        # Query KD-tree for nearest neighbors
        query_point = np.array([[x, y]])

        # Get multiple nearest neighbors for heading disambiguation
        k = min(10, len(self.mean_course.x))
        distances, indices = self.kdtree.query(query_point, k=k)

        distances = distances[0]
        indices = indices[0]

        # Check if vehicle is on course
        min_distance = distances[0]
        position_tolerance = self.params['position_tolerance']

        if min_distance > position_tolerance * 2:
            # Vehicle is far off course
            return SegmentEstimate(
                segment_id=None,
                confidence=0.0,
                distance_to_course=min_distance,
                heading_error=0.0,
                course_position=(self.mean_course.x[indices[0]],
                               self.mean_course.y[indices[0]]),
                course_heading=self.mean_course.heading[indices[0]]
            )

        # Find best match using combined position and heading
        best_idx = None
        best_score = float('inf')
        heading_weight = self.params['heading_weight']
        heading_tolerance = self.params['heading_tolerance']

        for i, idx in enumerate(indices):
            dist = distances[i]
            course_heading = self.mean_course.heading[idx]
            heading_diff = abs(angle_difference(heading, course_heading))

            # Combined score
            score = dist + heading_weight * (heading_diff / 180.0) * dist

            if score < best_score:
                best_score = score
                best_idx = idx

        # Get segment ID
        segment_id = int(self.index_to_segment[best_idx])

        # Calculate confidence
        course_heading = self.mean_course.heading[best_idx]
        heading_error = angle_difference(heading, course_heading)

        # Confidence based on distance and heading alignment
        distance_factor = np.exp(-min_distance / position_tolerance)
        heading_factor = 1.0 - (abs(heading_error) / heading_tolerance)
        heading_factor = max(0.0, heading_factor)

        confidence = distance_factor * (0.7 + 0.3 * heading_factor)
        confidence = min(1.0, max(0.0, confidence))

        return SegmentEstimate(
            segment_id=segment_id,
            confidence=confidence,
            distance_to_course=min_distance,
            heading_error=heading_error,
            course_position=(self.mean_course.x[best_idx],
                           self.mean_course.y[best_idx]),
            course_heading=course_heading
        )

    def estimate_batch(self, positions: np.ndarray) -> List[SegmentEstimate]:
        """
        Estimate segments for batch of positions

        Args:
            positions: Nx3 array of (x, y, heading)

        Returns:
            List of SegmentEstimate objects
        """
        results = []

        for i in range(len(positions)):
            x, y, heading = positions[i]
            estimate = self.estimate(x, y, heading)
            results.append(estimate)

        return results
