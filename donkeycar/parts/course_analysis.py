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
from typing import List, Optional, Tuple, Dict, Any
from enum import Enum
from scipy.signal import savgol_filter
from scipy.spatial import KDTree
from scipy.interpolate import interp1d

logger = logging.getLogger(__name__)


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
    Tub fields: _timestamp_ms, car/pos, car/heading (radians)
    Automatically detects and separates individual laps
    """

    def __init__(self):
        self.raw_data = None
        self.laps = []
        self.num_laps = 0

    def load_data(self, source: str,
                  lap_detection_threshold: float = 2.0,
                  min_lap_length: int = 50) -> None:
        """
        Load multi-lap data from CSV file or Tub directory

        Args:
            source: Path to CSV file or Tub directory
            lap_detection_threshold: Distance threshold for lap (meters)
            min_lap_length: Minimum number of points per lap
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

        # Detect laps
        self._detect_laps(lap_detection_threshold, min_lap_length)

        if self.raw_data is None:
            raise ValueError("No data loaded from source")
        logger.info(f"Loaded {len(self.raw_data)} points from {source}")
        logger.info(f"Detected {self.num_laps} laps")

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
        self.load_data(filepath, lap_detection_threshold, min_lap_length)

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

            # Get heading in radians
            heading = record.get('car/heading', 0.0)

            data_rows.append((timestamp, x, y, heading))

        tub.close()

        if not data_rows:
            raise ValueError(f"No IMU path data found in Tub: {tub_path}")

        # Convert to structured array matching CSV format
        self.raw_data = np.array(
            data_rows,
            dtype=[('timestamp', 'f8'), ('x', 'f8'),
                   ('y', 'f8'), ('heading', 'f8')])

    def _detect_laps(self, threshold: float, min_length: int) -> None:
        """
        Detect individual laps from continuous data

        Detects when vehicle returns near starting position

        Args:
            threshold: Distance threshold for lap closure (meters)
            min_length: Minimum points per lap
        """
        if self.raw_data is None:
            raise ValueError("No raw data available for lap detection")

        if len(self.raw_data) < min_length:
            logger.warning("Data too short for lap detection")
            return

        # Extract positions
        x = self.raw_data['x']
        y = self.raw_data['y']

        # Start position
        start_x, start_y = x[0], y[0]

        # Find indices where vehicle returns to start
        lap_indices = [0]

        for i in range(min_length, len(x)):
            dist = np.sqrt((x[i] - start_x)**2 + (y[i] - start_y)**2)

            # Check if we've returned to start and have enough points
            if dist < threshold and (i - lap_indices[-1]) >= min_length:
                lap_indices.append(i)

        # Add final index
        if lap_indices[-1] != len(x) - 1:
            lap_indices.append(len(x))

        # Extract laps
        self.laps = []
        for i in range(len(lap_indices) - 1):
            start_idx = lap_indices[i]
            end_idx = lap_indices[i + 1]

            if end_idx - start_idx >= min_length:
                lap_data = self.raw_data[start_idx:end_idx]
                self.laps.append(lap_data)

        self.num_laps = len(self.laps)

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
        Compute mean course from multi-lap data

        Process:
        1. Align all laps to common reference frame
        2. Resample to common arc-length parameterization
        3. Remove outliers
        4. Compute point-wise mean
        5. Apply smoothing
        """
        if self.multilap_data is None or self.multilap_data.num_laps == 0:
            raise ValueError("No lap data available")

        laps = self.multilap_data.get_laps()
        self.num_laps = len(laps)

        if self.num_laps < 1:
            raise ValueError("Need at least 1 lap for mean course")

        # Step 1: Align laps
        aligned_laps = self._align_laps(laps)

        # Step 2: Resample to common parameterization
        resampled_laps = self._resample_laps(aligned_laps)

        # Step 3: Remove outliers
        cleaned_laps = self._remove_outliers(resampled_laps)

        # Step 4: Compute mean
        self._compute_mean(cleaned_laps)

        # Step 5: Apply smoothing
        self._apply_smoothing()

        # Compute cumulative distance
        self._compute_distance()

        logger.info(f"Computed mean course from {self.num_laps} laps")
        logger.info(f"Course length: {self.distance[-1]:.2f} meters")

    def _align_laps(self, laps: List[np.ndarray]) -> List[Dict[str, np.ndarray]]:
        """
        Align all laps to common reference frame

        Translates to common start position and rotates to common heading

        Args:
            laps: List of raw lap data

        Returns:
            List of aligned lap dictionaries with 'x', 'y', 'heading' arrays
        """
        aligned = []

        # Use first lap as reference
        ref_x0 = laps[0]['x'][0]
        ref_y0 = laps[0]['y'][0]
        ref_heading0 = laps[0]['heading'][0]

        for lap in laps:
            x = lap['x'].copy()
            y = lap['y'].copy()
            heading = lap['heading'].copy()

            # Translate to reference start position
            x = x - lap['x'][0] + ref_x0
            y = y - lap['y'][0] + ref_y0

            # Rotate to reference heading
            heading_offset = ref_heading0 - lap['heading'][0]
            heading = heading + heading_offset

            # Normalize headings
            heading = np.array([normalize_angle(h) for h in heading])

            aligned.append({'x': x, 'y': y, 'heading': heading})

        return aligned

    def _resample_laps(self, laps: List[Dict[str, np.ndarray]]) -> List[Dict[str, np.ndarray]]:
        """
        Resample laps to common arc-length parameterization

        Args:
            laps: List of aligned lap data

        Returns:
            List of resampled lap data with uniform spacing
        """
        interval = self.params['resampling_interval']
        resampled = []

        for lap in laps:
            x = lap['x']
            y = lap['y']
            heading = lap['heading']

            # Compute arc length
            dx = np.diff(x)
            dy = np.diff(y)
            ds = np.sqrt(dx**2 + dy**2)
            s = np.concatenate([[0], np.cumsum(ds)])

            # New arc length parameterization
            total_length = s[-1]
            num_points = int(total_length / interval) + 1
            s_new = np.linspace(0, total_length, num_points)

            # Interpolate x, y
            x_new = np.interp(s_new, s, x)
            y_new = np.interp(s_new, s, y)

            # Interpolate heading (circular, already in radians)
            sin_h = np.sin(heading)
            cos_h = np.cos(heading)
            sin_h_new = np.interp(s_new, s, sin_h)
            cos_h_new = np.interp(s_new, s, cos_h)
            heading_new = np.arctan2(sin_h_new, cos_h_new)

            resampled.append({'x': x_new, 'y': y_new, 'heading': heading_new})

        return resampled

    def _remove_outliers(self, laps: List[Dict[str, np.ndarray]]) -> List[Dict[str, np.ndarray]]:
        """
        Remove outlier points that deviate from mean trajectory

        Args:
            laps: List of resampled lap data

        Returns:
            List of cleaned lap data
        """
        threshold = self.params['outlier_std_threshold']
        iterations = self.params['outlier_iterations']

        # Find minimum length
        min_len = min(len(lap['x']) for lap in laps)

        # Truncate all laps to same length
        for lap in laps:
            lap['x'] = lap['x'][:min_len]
            lap['y'] = lap['y'][:min_len]
            lap['heading'] = lap['heading'][:min_len]

        # Iterative outlier removal
        for iteration in range(iterations):
            # Stack data
            x_stack = np.array([lap['x'] for lap in laps])
            y_stack = np.array([lap['y'] for lap in laps])

            # Compute mean and std
            x_mean = np.mean(x_stack, axis=0)
            y_mean = np.mean(y_stack, axis=0)
            x_std = np.std(x_stack, axis=0)
            y_std = np.std(y_stack, axis=0)

            # Find outliers
            for i, lap in enumerate(laps):
                x_dev = np.abs(lap['x'] - x_mean) / (x_std + 1e-6)
                y_dev = np.abs(lap['y'] - y_mean) / (y_std + 1e-6)

                # Mark outliers
                outliers = (x_dev > threshold) | (y_dev > threshold)

                # Replace outliers with mean
                lap['x'][outliers] = x_mean[outliers]
                lap['y'][outliers] = y_mean[outliers]

        return laps

    def _compute_mean(self, laps: List[Dict[str, np.ndarray]]) -> None:
        """
        Compute point-wise mean of aligned and cleaned laps

        Args:
            laps: List of cleaned lap data
        """
        # Stack data
        x_stack = np.array([lap['x'] for lap in laps])
        y_stack = np.array([lap['y'] for lap in laps])
        heading_stack = np.array([lap['heading'] for lap in laps])

        # Compute means
        self.x = np.mean(x_stack, axis=0)
        self.y = np.mean(y_stack, axis=0)

        # Circular mean for heading
        self.heading = np.array([
            circular_mean(heading_stack[:, i])
            for i in range(heading_stack.shape[1])
        ])

        # Store statistics
        self.metadata['x_std'] = np.std(x_stack, axis=0)
        self.metadata['y_std'] = np.std(y_stack, axis=0)
        self.metadata['heading_std'] = np.array([
            circular_std(heading_stack[:, i])
            for i in range(heading_stack.shape[1])
        ])

    def _apply_smoothing(self) -> None:
        """
        Apply smoothing to reduce high-frequency noise
        """
        pos_window = self.params['position_smoothing_window']
        pos_order = self.params['position_polynomial_order']
        head_window = self.params['heading_smoothing_window']

        # Ensure window size is valid
        if pos_window > len(self.x):
            pos_window = len(self.x) if len(self.x) % 2 == 1 else len(self.x) - 1
        if pos_window % 2 == 0:
            pos_window -= 1
        if pos_window < pos_order + 2:
            pos_window = pos_order + 2
            if pos_window % 2 == 0:
                pos_window += 1

        if head_window > len(self.heading):
            head_window = len(self.heading)
        if head_window % 2 == 0:
            head_window -= 1

        # Savitzky-Golay filter for positions
        if pos_window >= pos_order + 2:
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
            'curvature_smoothing_window': 11,
            'straight_curvature_threshold': 0.05,  # rad/m
            'min_segment_length': 0.5,  # meters
            'inflection_threshold': 0.02,  # rad/m
        }

        if params is not None:
            self.params.update(params)

        self.segments: List[Segment] = []
        self.total_segments = 0
        self.segment_counts: Dict[SegmentType, int] = {}

    def compute(self) -> None:
        """
        Compute course segmentation

        Process:
        1. Calculate curvature
        2. Detect segment boundaries
        3. Classify segments
        4. Create Segment objects
        """
        if self.mean_course is None:
            raise ValueError("No mean course available")

        # Step 1: Calculate curvature
        curvature = self._calculate_curvature()

        # Step 2: Detect segment boundaries
        boundaries = self._detect_boundaries(curvature)

        # Step 3: Classify and create segments
        self._create_segments(boundaries, curvature)

        # Count segment types
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

    def _detect_boundaries(self, curvature: np.ndarray) -> List[int]:
        """
        Detect segment boundaries based on curvature

        Args:
            curvature: Array of curvature values

        Returns:
            List of boundary indices
        """
        threshold = self.params['straight_curvature_threshold']
        inflection_threshold = self.params['inflection_threshold']
        min_length = self.params['min_segment_length']

        boundaries = [0]

        # Detect sign changes and threshold crossings
        for i in range(1, len(curvature) - 1):
            # Check for inflection point (sign change)
            if curvature[i - 1] * curvature[i] < 0:
                # Only if curvature is significant
                if abs(curvature[i - 1]) > inflection_threshold or abs(curvature[i]) > inflection_threshold:
                    boundaries.append(i)

            # Check for transition from straight to curve
            elif abs(curvature[i - 1]) < threshold and abs(curvature[i]) >= threshold:
                boundaries.append(i)

            # Check for transition from curve to straight
            elif abs(curvature[i - 1]) >= threshold and abs(curvature[i]) < threshold:
                boundaries.append(i)

        boundaries.append(len(curvature) - 1)

        # Merge short segments
        boundaries = self._merge_short_segments(boundaries, min_length)

        return boundaries

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
            json.dump(data, f, indent=2)

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
