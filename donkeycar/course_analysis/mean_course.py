"""
Mean course reconstruction from multi-lap data.

Provides builder pattern for constructing mean courses from multiple laps.
All processing is done via pure functions (no state mutation).

Design principles:
- Builder pattern: Separate construction from representation
- Immutable data: MeanCourse is read-only
- No magic numbers: All parameters configurable
- Pure functions: build() returns new MeanCourse

Phase 3 of IMU path refactoring.
"""

from typing import List, Dict, Any, Optional
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

from .lap_detection import MultiLapData


class MeanCourse:
    """
    Immutable container for reconstructed mean course.

    Stores the averaged trajectory computed from multiple laps.
    All arrays are read-only to prevent accidental modification.

    Attributes:
        x: X positions (meters, read-only)
        y: Y positions (meters, read-only)
        heading: Heading angles (radians, read-only)
        distance: Cumulative distance (meters, read-only)
        metadata: Additional information (dict)
    """

    def __init__(self, x: np.ndarray, y: np.ndarray,
                 heading: np.ndarray, distance: np.ndarray,
                 metadata: Dict[str, Any]):
        """
        Create immutable mean course.

        Args:
            x: X positions
            y: Y positions
            heading: Heading angles
            distance: Cumulative distance
            metadata: Additional course information
        """
        # Store as read-only arrays
        self._x = np.array(x, dtype=np.float64)
        self._y = np.array(y, dtype=np.float64)
        self._heading = np.array(heading, dtype=np.float64)
        self._distance = np.array(distance, dtype=np.float64)
        self._metadata = dict(metadata)

        # Make arrays read-only
        self._x.flags.writeable = False
        self._y.flags.writeable = False
        self._heading.flags.writeable = False
        self._distance.flags.writeable = False

    @property
    def x(self) -> np.ndarray:
        """X positions (read-only)"""
        return self._x

    @property
    def y(self) -> np.ndarray:
        """Y positions (read-only)"""
        return self._y

    @property
    def heading(self) -> np.ndarray:
        """Heading angles (read-only)"""
        return self._heading

    @property
    def distance(self) -> np.ndarray:
        """Cumulative distance (read-only)"""
        return self._distance

    @property
    def metadata(self) -> Dict[str, Any]:
        """Course metadata (read-only copy)"""
        return self._metadata.copy()

    def __len__(self) -> int:
        """Number of points in course"""
        return len(self._x)

    @property
    def length(self) -> float:
        """Total course length in meters"""
        return float(self._distance[-1])


class MeanCourseBuilder:
    """
    Builder for constructing mean course from multiple laps.

    Uses pure function approach: build() returns new MeanCourse,
    doesn't mutate builder state.

    Algorithm:
    1. Resample all laps onto common normalized arc-length axis
    2. Compute weighted mean with equal lap contribution
    3. Apply smoothing (Savitzky-Golay for position, moving avg for heading)
    4. Apply loop closure correction (ensure start/end continuity)
    5. Compute cumulative distance
    """

    DEFAULT_PARAMS = {
        'resampling_interval': 0.1,         # Distance between resampled pts
        'min_resampling_interval': 0.001,   # Minimum allowed interval
        'fallback_interval': 0.05,          # Fallback if param invalid
        'position_smoothing_window': 11,    # Savgol window for position
        'position_polynomial_order': 3,     # Savgol polynomial order
        'heading_smoothing_window': 5,      # Moving average window
        'loop_closure_pct': 0.025,          # Percent of course for closure
        'outlier_std_threshold': 2.5,       # Std devs for outlier detection
        'outlier_iterations': 2,            # Outlier removal iterations
    }

    def __init__(self, cfg=None, params: Optional[Dict[str, Any]] = None):
        """
        Create mean course builder.

        Args:
            cfg: Config object (optional)
            params: Explicit parameter overrides (optional)
        """
        # Start with defaults
        self.params = self.DEFAULT_PARAMS.copy()

        # Override from config
        if cfg is not None:
            mean_params = getattr(cfg, 'MEAN_COURSE_PARAMS', {})
            for key in self.params.keys():
                if key in mean_params:
                    self.params[key] = mean_params[key]

        # Override from explicit params
        if params is not None:
            self.params.update(params)

    def build(self, multilap_data: MultiLapData) -> MeanCourse:
        """
        Build mean course from multi-lap data.

        Pure function - returns new MeanCourse, doesn't modify input.

        Args:
            multilap_data: Multi-lap data to process

        Returns:
            MeanCourse: Immutable mean course

        Raises:
            ValueError: If insufficient laps or invalid data
        """
        if multilap_data.num_laps < 1:
            raise ValueError("Need at least 1 lap for mean course")

        # Process laps
        resampled_laps = self._resample_laps(multilap_data)
        x, y, heading = self._compute_weighted_mean(resampled_laps)
        x, y = self._apply_smoothing(x, y)
        x, y = self._apply_loop_closure(x, y)
        heading = self._smooth_heading(heading)
        distance = self._compute_distance(x, y)
        metadata = self._compute_metadata(resampled_laps)

        return MeanCourse(x, y, heading, distance, metadata)

    def _resample_laps(self, multilap_data: MultiLapData
                      ) -> List[Dict[str, np.ndarray]]:
        """
        Resample all laps onto common normalized arc-length axis.

        Args:
            multilap_data: Multi-lap data

        Returns:
            List of resampled lap dictionaries
        """
        interval = max(self.params['resampling_interval'],
                      self.params['min_resampling_interval'])

        lap_lengths = []
        lap_samples = []

        for i in range(multilap_data.num_laps):
            lap_data = multilap_data.get_lap(i)
            x = lap_data.x.astype(np.float64)
            y = lap_data.y.astype(np.float64)
            heading = lap_data.heading.astype(np.float64)

            if len(x) < 2:
                lap_lengths.append(0.0)
                continue

            # Calculate arc length
            dx = np.diff(x)
            dy = np.diff(y)
            ds = np.sqrt(dx**2 + dy**2)
            s = np.concatenate([[0], np.cumsum(ds)])
            length = s[-1]

            if length > 0:
                lap_lengths.append(length)
                s_norm = s / length
                lap_samples.append({
                    'x': x, 'y': y, 'heading': heading, 's_norm': s_norm
                })

        if not lap_samples:
            raise ValueError("No valid laps to resample")

        # Determine reference length and sample count
        ref_length = np.mean(lap_lengths)
        num_samples = max(2, int(ref_length / interval) + 1)
        s_ref = np.linspace(0.0, 1.0, num_samples)

        # Resample each lap onto reference axis
        resampled = []
        for sample in lap_samples:
            s_norm = sample['s_norm']

            # Interpolate position
            x_interp = np.interp(s_ref, s_norm, sample['x'])
            y_interp = np.interp(s_ref, s_norm, sample['y'])

            # Interpolate heading (circular)
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

        return resampled

    def _compute_weighted_mean(self, laps: List[Dict[str, np.ndarray]]
                               ) -> tuple:
        """
        Compute weighted mean with equal lap contribution.

        Args:
            laps: Resampled laps

        Returns:
            (x, y, heading) arrays
        """
        x_mean = laps[0]['x'].copy()
        y_mean = laps[0]['y'].copy()
        sin_mean = np.sin(laps[0]['heading'])
        cos_mean = np.cos(laps[0]['heading'])

        # Incremental averaging
        for idx, lap in enumerate(laps[1:], start=2):
            new_weight = 1.0 / idx
            prev_weight = 1.0 - new_weight

            x_mean = prev_weight * x_mean + new_weight * lap['x']
            y_mean = prev_weight * y_mean + new_weight * lap['y']
            sin_mean = prev_weight * sin_mean + \
                new_weight * np.sin(lap['heading'])
            cos_mean = prev_weight * cos_mean + \
                new_weight * np.cos(lap['heading'])

        heading_mean = np.arctan2(sin_mean, cos_mean)
        return x_mean, y_mean, heading_mean

    def _apply_smoothing(self, x: np.ndarray, y: np.ndarray) -> tuple:
        """
        Apply Savitzky-Golay smoothing to position.

        Args:
            x: X positions
            y: Y positions

        Returns:
            (x_smooth, y_smooth)
        """
        window = self.params['position_smoothing_window']
        poly_order = self.params['position_polynomial_order']

        # Ensure window is valid
        if window > len(x):
            window = len(x) if len(x) % 2 == 1 else len(x) - 1
        if window < poly_order + 2:
            return x, y  # Skip smoothing if window too small

        x_smooth = savgol_filter(x, window, poly_order)
        y_smooth = savgol_filter(y, window, poly_order)

        return x_smooth, y_smooth

    def _smooth_heading(self, heading: np.ndarray) -> np.ndarray:
        """
        Apply moving average to heading.

        Args:
            heading: Heading angles

        Returns:
            Smoothed heading
        """
        window = self.params['heading_smoothing_window']
        if window < 1 or window > len(heading):
            return heading

        # Circular smoothing
        sin_h = np.sin(heading)
        cos_h = np.cos(heading)

        kernel = np.ones(window) / window
        sin_smooth = np.convolve(sin_h, kernel, mode='same')
        cos_smooth = np.convolve(cos_h, kernel, mode='same')

        return np.arctan2(sin_smooth, cos_smooth)

    def _apply_loop_closure(self, x: np.ndarray, y: np.ndarray) -> tuple:
        """
        Apply loop closure correction to ensure start/end continuity.

        Args:
            x: X positions
            y: Y positions

        Returns:
            (x_corrected, y_corrected)
        """
        closure_pct = self.params['loop_closure_pct']
        n = len(x)
        closure_points = max(2, int(n * closure_pct))

        # Calculate closure error
        dx_closure = x[-1] - x[0]
        dy_closure = y[-1] - y[0]

        # Apply linear correction over closure region
        correction_x = np.linspace(0, dx_closure, closure_points)
        correction_y = np.linspace(0, dy_closure, closure_points)

        x_corrected = x.copy()
        y_corrected = y.copy()
        x_corrected[-closure_points:] -= correction_x
        y_corrected[-closure_points:] -= correction_y

        return x_corrected, y_corrected

    def _compute_distance(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Compute cumulative distance along course.

        Args:
            x: X positions
            y: Y positions

        Returns:
            Cumulative distance array
        """
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.sqrt(dx**2 + dy**2)
        return np.concatenate([[0], np.cumsum(ds)])

    def _compute_metadata(self, laps: List[Dict[str, np.ndarray]]
                         ) -> Dict[str, Any]:
        """
        Compute metadata about mean course.

        Args:
            laps: Resampled laps

        Returns:
            Metadata dictionary
        """
        return {
            'num_laps_used': len(laps),
            'num_points': len(laps[0]['x']) if laps else 0,
        }
