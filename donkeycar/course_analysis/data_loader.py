"""
Data loading infrastructure for course analysis.

Provides immutable data containers and pluggable data sources for loading
path data from various formats (CSV, Tub).

Design principles:
- Immutable data containers (PathData)
- Strategy pattern for data sources (PathDataSource)
- No file I/O in tests (inject synthetic data)
- All configuration via constructor parameters

Phase 1 of IMU path refactoring.
"""

from abc import ABC, abstractmethod
from typing import Optional, Tuple
import os
import numpy as np
import pandas as pd


class PathData:
    """
    Immutable container for vehicle path data.

    Stores timestamp, position (x, y), heading, velocity, and cumulative
    distance arrays. Arrays are made read-only to prevent accidental
    modification.

    Attributes:
        timestamp: Time in seconds (np.ndarray, read-only)
        x: X position in meters, right direction (np.ndarray, read-only)
        y: Y position in meters, forward direction (np.ndarray, read-only)
        heading: Heading angle in radians (np.ndarray, read-only)
        velocity: Speed in meters/second (np.ndarray, read-only)
        distance: Cumulative distance in meters (np.ndarray, read-only)
    """

    def __init__(
        self,
        timestamp: np.ndarray,
        x: np.ndarray,
        y: np.ndarray,
        heading: np.ndarray,
        velocity: np.ndarray,
        distance: np.ndarray = None
    ):
        """
        Create immutable PathData container.

        Args:
            timestamp: Time values (seconds)
            x: X positions (meters)
            y: Y positions (meters)
            heading: Heading angles (radians)
            velocity: Speed values (m/s)
            distance: Cumulative distance values (meters). If None, computed
                     from x, y using Euclidean distance.

        Raises:
            ValueError: If arrays have different lengths
        """
        # Validate lengths
        lengths = [len(timestamp), len(x), len(y), len(heading),
                   len(velocity)]
        if len(set(lengths)) != 1:
            raise ValueError(
                f"All arrays must have same length, got {lengths}")

        # Store as read-only arrays
        self._timestamp = np.array(timestamp, dtype=np.float64)
        self._x = np.array(x, dtype=np.float64)
        self._y = np.array(y, dtype=np.float64)
        self._heading = np.array(heading, dtype=np.float64)
        self._velocity = np.array(velocity, dtype=np.float64)

        # Compute or use provided distance
        if distance is None:
            self._distance = self._compute_cumulative_distance(
                self._x, self._y)
        else:
            if len(distance) != len(timestamp):
                raise ValueError(
                    f"Distance array length {len(distance)} must match "
                    f"timestamp length {len(timestamp)}")
            self._distance = np.array(distance, dtype=np.float64)

        # Make arrays read-only
        self._timestamp.flags.writeable = False
        self._x.flags.writeable = False
        self._y.flags.writeable = False
        self._heading.flags.writeable = False
        self._velocity.flags.writeable = False
        self._distance.flags.writeable = False

    @staticmethod
    def _compute_cumulative_distance(x: np.ndarray, y: np.ndarray) \
        -> np.ndarray:
        """
        Compute cumulative distance from x, y coordinates.

        Args:
            x: X positions
            y: Y positions

        Returns:
            Cumulative distance array (starts at 0)
        """
        if len(x) == 0:
            return np.array([])

        # Compute segment distances
        dx = np.diff(x)
        dy = np.diff(y)
        segment_distances = np.sqrt(dx**2 + dy**2)

        # Cumulative sum (prepend 0 for first point)
        cumulative = np.concatenate([[0.0], np.cumsum(segment_distances)])
        return cumulative

    @property
    def timestamp(self) -> np.ndarray:
        """Time values (seconds, read-only)"""
        return self._timestamp

    @property
    def x(self) -> np.ndarray:
        """X positions (meters, read-only)"""
        return self._x

    @property
    def y(self) -> np.ndarray:
        """Y positions (meters, read-only)"""
        return self._y

    @property
    def heading(self) -> np.ndarray:
        """Heading angles (radians, read-only)"""
        return self._heading

    @property
    def velocity(self) -> np.ndarray:
        """Speed values (m/s, read-only)"""
        return self._velocity

    @property
    def distance(self) -> np.ndarray:
        """Cumulative distance values (meters, read-only)"""
        return self._distance

    def __len__(self) -> int:
        """Return number of data points"""
        return len(self._timestamp)

    @property
    def duration(self) -> float:
        """Total duration in seconds"""
        return float(self._timestamp[-1] - self._timestamp[0])

    @property
    def mean_velocity(self) -> float:
        """Mean velocity in m/s"""
        return float(np.mean(self._velocity))

    @property
    def total_distance(self) -> float:
        """
        Total distance traveled in meters.
        Computed as sum of distances between consecutive points.
        """
        dx = np.diff(self._x)
        dy = np.diff(self._y)
        distances = np.sqrt(dx**2 + dy**2)
        return float(np.sum(distances))


class PathDataSource(ABC):
    """
    Abstract base class for path data sources.

    Concrete implementations load data from different formats (CSV, Tub).
    This enables dependency injection and testing with synthetic data.
    """

    @abstractmethod
    def load(self) -> PathData:
        """
        Load and return path data.

        Returns:
            PathData: Immutable path data container

        Raises:
            FileNotFoundError: If source file/directory doesn't exist
            ValueError: If data format is invalid
        """
        pass


class CSVPathDataSource(PathDataSource):
    """
    Load path data from CSV file.

    Expected CSV format:
        t,x,y,h,v
        0.0,0.0,0.0,0.0,1.0
        0.1,0.1,0.1,0.1,1.0
        ...

    Columns:
        t: timestamp (seconds)
        x: X position (meters)
        y: Y position (meters)
        h: heading (radians)
        v: velocity (m/s)
    """

    def __init__(self, filepath: str):
        """
        Create CSV data source.

        Args:
            filepath: Path to CSV file
        """
        self.filepath = filepath

    def load(self) -> PathData:
        """
        Load path data from CSV file.

        Returns:
            PathData: Loaded path data

        Raises:
            FileNotFoundError: If CSV file doesn't exist
            ValueError: If required columns are missing
        """
        try:
            df = pd.read_csv(self.filepath)
        except FileNotFoundError:
            raise FileNotFoundError(
                f"CSV file not found: {self.filepath}")

        # Validate required columns
        required = ['t', 'x', 'y', 'h', 'v']
        missing = [col for col in required if col not in df.columns]
        if missing:
            raise ValueError(
                f"Missing required columns: {missing}. "
                f"Expected: {required}")

        return PathData(
            timestamp=df['t'].values,
            x=df['x'].values,
            y=df['y'].values,
            heading=df['h'].values,
            velocity=df['v'].values
        )


class TubPathDataSource(PathDataSource):
    """
    Load path data from Tub directory.

    Tub format stores data in manifest.json + catalog_*.json + data files.
    Extracts: _timestamp_ms, car/pos, car/euler, car/speed, car/distance
    """

    def __init__(self, tub_path: str):
        """
        Create Tub data source.

        Args:
            tub_path: Path to Tub directory
        """
        self.tub_path = tub_path

    def load(self) -> PathData:
        """
        Load path data from Tub directory.

        Returns:
            PathData: Loaded path data

        Raises:
            FileNotFoundError: If Tub directory doesn't exist
            ValueError: If required fields are missing or no valid data found
        """
        import math
        from donkeycar.parts.tub_v2 import Tub

        if not os.path.exists(self.tub_path):
            raise FileNotFoundError(f"Tub directory not found: {self.tub_path}")

        # Open tub
        tub = Tub(self.tub_path, read_only=True)

        # Extract data from tub records
        timestamps = []
        x_positions = []
        y_positions = []
        headings = []
        velocities = []
        distances = []

        for record in tub:
            # Get timestamp in seconds (tub stores milliseconds)
            t = record.get('_timestamp_ms', 0) / 1000.0

            # Get position (car/pos is a vector [x, y, z])
            pos = record.get('car/pos')
            if pos is None or len(pos) < 2:
                continue

            x, y = pos[0], pos[1]

            # Get velocity (car/speed)
            v = record.get('car/speed', 0.0)

            # Get cumulative distance (car/distance)
            d = record.get('car/distance', None)

            # Calculate heading from euler angles (car/euler is [x, y, z] in
            # degrees)
            euler = record.get('car/euler', [0, 0, 0])
            # Convert to radians: heading = 90 - euler[2] (in degrees), then
            # to radians
            h_deg = 90.0 - euler[2]
            h = math.radians(h_deg)

            # Append to lists
            timestamps.append(t)
            x_positions.append(x)
            y_positions.append(y)
            headings.append(h)
            velocities.append(v)
            distances.append(d)

        tub.close()

        if len(timestamps) == 0:
            raise ValueError(f"No valid IMU path data found in Tub: "
                           f"{self.tub_path}")

        # Check if car/distance was available in tub
        # If all values are None, let PathData compute it from x, y
        distance_array = None
        if any(d is not None for d in distances):
            # At least some distance values available, use them
            # Fill None values with 0 (or could interpolate)
            distance_array = np.array([d if d is not None else 0.0
                                      for d in distances])

        # Convert to numpy arrays and create PathData
        return PathData(
            timestamp=np.array(timestamps),
            x=np.array(x_positions),
            y=np.array(y_positions),
            heading=np.array(headings),
            velocity=np.array(velocities),
            distance=distance_array
        )
