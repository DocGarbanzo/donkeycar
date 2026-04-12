"""
Synthetic course generators and driving simulator for testing.

Provides geometrically-defined courses (L-shape, U-shape) and an
Ornstein-Uhlenbeck driving simulator that produces realistic multi-lap
data for testing lap detection, mean course recovery, segmentation,
segment assignment, and segment statistics.
"""
import numpy as np
from donkeycar.course_analysis import PathData, MeanCourse


def _ccw_heading(theta):
    """Heading for counter-clockwise arc at angle theta."""
    return np.arctan2(np.cos(theta), -np.sin(theta))


def _cw_heading(theta):
    """Heading for clockwise arc at angle theta."""
    return np.arctan2(-np.cos(theta), np.sin(theta))


def _arc_points(cx, cy, r, theta_start, theta_end, n, clockwise=False):
    """Generate points along a circular arc.

    Args:
        cx, cy: Arc center coordinates
        r: Arc radius
        theta_start: Starting angle (radians)
        theta_end: Ending angle (radians)
        n: Number of points
        clockwise: If True, traverse clockwise (theta decreasing)

    Returns:
        x, y, heading arrays
    """
    theta = np.linspace(theta_start, theta_end, n)
    x = cx + r * np.cos(theta)
    y = cy + r * np.sin(theta)
    if clockwise:
        h = _cw_heading(theta)
    else:
        h = _ccw_heading(theta)
    return x, y, h


def _straight_points(x0, y0, x1, y1, heading, n):
    """Generate points along a straight line.

    Args:
        x0, y0: Start coordinates
        x1, y1: End coordinates
        heading: Constant heading value (radians)
        n: Number of points

    Returns:
        x, y, heading arrays
    """
    x = np.linspace(x0, x1, n)
    y = np.linspace(y0, y1, n)
    h = np.full(n, heading)
    return x, y, h


def _num_points(length, ppm):
    """Compute number of points for a segment."""
    return max(3, int(round(length * ppm)))


def _concat_segments(segments):
    """Concatenate segments, removing duplicate junction points.

    Args:
        segments: List of (x, y, heading, seg_type, seg_id) tuples

    Returns:
        x, y, heading arrays and segment metadata list
    """
    all_x, all_y, all_h = [], [], []
    meta = []
    idx = 0
    for i, (sx, sy, sh, stype, sid) in enumerate(segments):
        if i > 0:
            sx, sy, sh = sx[1:], sy[1:], sh[1:]
        start_idx = idx
        n_pts = len(sx)
        all_x.append(sx)
        all_y.append(sy)
        all_h.append(sh)
        idx += n_pts
        end_idx = idx - 1
        meta.append({
            'segment_id': sid,
            'type': stype,
            'start_index': start_idx,
            'end_index': end_idx,
        })

    x = np.concatenate(all_x)
    y = np.concatenate(all_y)
    h = np.concatenate(all_h)

    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx**2 + dy**2)
    distance = np.concatenate([[0.0], np.cumsum(ds)])

    for m in meta:
        m['start_distance'] = float(distance[m['start_index']])
        m['end_distance'] = float(distance[m['end_index']])

    return x, y, h, distance, meta


class LShapeCourse:
    """L-shaped course geometry.

    The course traces the outside of the letter 'L' at offset distance
    d (also used as turn radius). Counter-clockwise traversal with
    8 geometric segments:

        S1: Straight east    (bottom of horizontal bar)
        A1: Semicircle 180°  (right end, CCW)
        S2: Straight west    (top of horizontal bar)
        A2: Quarter 90°      (inner corner, CW)
        S3: Straight north   (right of vertical bar)
        A3: Semicircle 180°  (top end, CCW)
        S4: Straight south   (left of vertical bar)
        A4: Quarter 90°      (bottom-left corner, CCW)

    Y-crossing (neg→pos) occurs in A1 at (h_len+d, 0).
    """

    def __init__(self, h_len=10.0, v_len=15.0, d=1.0,
                 points_per_meter=10):
        if h_len <= 2 * d:
            raise ValueError(f"h_len ({h_len}) must be > 2*d ({2*d})")
        if v_len <= 2 * d:
            raise ValueError(f"v_len ({v_len}) must be > 2*d ({2*d})")
        self.h_len = h_len
        self.v_len = v_len
        self.d = d
        self.ppm = points_per_meter

    def generate(self):
        """Generate course geometry.

        Returns:
            (x, y, heading, distance, segment_metadata) tuple
        """
        h, v, d, ppm = self.h_len, self.v_len, self.d, self.ppm
        segments = []

        # S1: Straight east, (0,-d) → (h,-d)
        n = _num_points(h, ppm)
        segments.append((*_straight_points(0, -d, h, -d, 0.0, n),
                         'straight', 0))

        # A1: Semicircle CCW at right end, center (h,0), r=d
        # θ: -π/2 → π/2
        n = _num_points(np.pi * d, ppm)
        segments.append((*_arc_points(h, 0, d, -np.pi/2, np.pi/2, n),
                         'semicircle_left', 1))

        # S2: Straight west, (h,d) → (2d,d)
        length = h - 2 * d
        n = _num_points(length, ppm)
        segments.append((*_straight_points(h, d, 2*d, d, np.pi, n),
                         'straight', 2))

        # A2: Quarter circle CW at inner corner, center (2d,2d), r=d
        # θ: 3π/2 → π (decreasing, CW)
        n = _num_points(np.pi * d / 2, ppm)
        segments.append((
            *_arc_points(2*d, 2*d, d, 3*np.pi/2, np.pi, n, clockwise=True),
            'quarter_right', 3))

        # S3: Straight north, (d,2d) → (d,v)
        length = v - 2 * d
        n = _num_points(length, ppm)
        segments.append((*_straight_points(d, 2*d, d, v, np.pi/2, n),
                         'straight', 4))

        # A3: Semicircle CCW at top, center (0,v), r=d
        # θ: 0 → π
        n = _num_points(np.pi * d, ppm)
        segments.append((*_arc_points(0, v, d, 0, np.pi, n),
                         'semicircle_left', 5))

        # S4: Straight south, (-d,v) → (-d,0)
        n = _num_points(v, ppm)
        segments.append((*_straight_points(-d, v, -d, 0, -np.pi/2, n),
                         'straight', 6))

        # A4: Quarter circle CCW at bottom-left, center (0,0), r=d
        # θ: π → 3π/2
        n = _num_points(np.pi * d / 2, ppm)
        segments.append((*_arc_points(0, 0, d, np.pi, 3*np.pi/2, n),
                         'quarter_left', 7))

        return _concat_segments(segments)

    def expected_total_length(self):
        """Theoretical total course length."""
        h, v, d = self.h_len, self.v_len, self.d
        straights = h + (h - 2*d) + (v - 2*d) + v
        arcs = 2 * np.pi * d + np.pi * d  # 2 semicircles + 2 quarters
        return straights + arcs

    def to_mean_course(self):
        """Generate and return as MeanCourse object."""
        x, y, heading, distance, meta = self.generate()
        return MeanCourse(
            x=x, y=y, heading=heading, distance=distance,
            metadata={
                'type': 'L_shape',
                'h_len': self.h_len,
                'v_len': self.v_len,
                'd': self.d,
                'segments': meta,
            }
        )


class UShapeCourse:
    """U-shaped course geometry.

    The course traces the outside of the letter 'U' at offset distance
    d. Counter-clockwise traversal with 8 geometric segments:

        S1: Straight north   (right outer arm)
        A1: Semicircle 180°  (top-right, CCW)
        S2: Straight south   (right inner arm)
        A2: Semicircle 180°  (bottom inner, CW)
        S3: Straight north   (left inner arm)
        A3: Semicircle 180°  (top-left, CCW)
        S4: Straight south   (left outer arm)
        A4: Semicircle 180°  (bottom outer, CCW)

    The U is shifted vertically so Y-crossing (neg→pos) occurs
    only in A4 (outer bottom semicircle), once per lap.

    Args:
        width: Distance between U arms (default 6.0m)
        v_len: Arm height (default 12.0m)
        d: Offset distance / turn radius (default 1.0m)
    """

    def __init__(self, width=6.0, v_len=12.0, d=1.0,
                 points_per_meter=10):
        if width <= 2 * d:
            raise ValueError(
                f"width ({width}) must be > 2*d ({2*d})")
        self.width = width
        self.v_len = v_len
        self.d = d
        self.ppm = points_per_meter

    def generate(self):
        """Generate course geometry.

        Returns:
            (x, y, heading, distance, segment_metadata) tuple
        """
        w, v, d, ppm = self.width, self.v_len, self.d, self.ppm
        r_inner = w / 2 - d
        r_outer = w / 2 + d
        # Shift so A4 bottom is below y=0 and A2 bottom is above y=0
        y_shift = w / 2 - d + 0.01

        segments = []

        # S1: Straight north, right outer arm
        # x = w + d, y from y_shift to v + y_shift
        n = _num_points(v, ppm)
        segments.append((*_straight_points(
            w + d, y_shift, w + d, v + y_shift, np.pi/2, n),
            'straight', 0))

        # A1: Semicircle CCW at top-right, center (w, v+y_shift), r=d
        # θ: 0 → π
        n = _num_points(np.pi * d, ppm)
        segments.append((*_arc_points(
            w, v + y_shift, d, 0, np.pi, n),
            'semicircle_left', 1))

        # S2: Straight south, right inner arm
        # x = w - d, y from v + y_shift down to y_shift
        n = _num_points(v, ppm)
        segments.append((*_straight_points(
            w - d, v + y_shift, w - d, y_shift, -np.pi/2, n),
            'straight', 2))

        # A2: Semicircle CW at bottom inner, center (w/2, y_shift)
        # r = w/2 - d, θ: 0 → -π (CW)
        n = _num_points(np.pi * r_inner, ppm)
        segments.append((*_arc_points(
            w / 2, y_shift, r_inner, 0, -np.pi, n, clockwise=True),
            'semicircle_right', 3))

        # S3: Straight north, left inner arm
        # x = d, y from y_shift to v + y_shift
        n = _num_points(v, ppm)
        segments.append((*_straight_points(
            d, y_shift, d, v + y_shift, np.pi/2, n),
            'straight', 4))

        # A3: Semicircle CCW at top-left, center (0, v+y_shift), r=d
        # θ: 0 → π
        n = _num_points(np.pi * d, ppm)
        segments.append((*_arc_points(
            0, v + y_shift, d, 0, np.pi, n),
            'semicircle_left', 5))

        # S4: Straight south, left outer arm
        # x = -d, y from v + y_shift down to y_shift
        n = _num_points(v, ppm)
        segments.append((*_straight_points(
            -d, v + y_shift, -d, y_shift, -np.pi/2, n),
            'straight', 6))

        # A4: Semicircle CCW at bottom outer, center (w/2, y_shift)
        # r = w/2 + d, θ: π → 2π
        n = _num_points(np.pi * r_outer, ppm)
        segments.append((*_arc_points(
            w / 2, y_shift, r_outer, np.pi, 2 * np.pi, n),
            'semicircle_left', 7))

        return _concat_segments(segments)

    def expected_total_length(self):
        """Theoretical total course length."""
        w, v, d = self.width, self.v_len, self.d
        straights = 4 * v
        r_inner = w / 2 - d
        r_outer = w / 2 + d
        arcs = 2 * np.pi * d + np.pi * r_inner + np.pi * r_outer
        return straights + arcs

    def to_mean_course(self):
        """Generate and return as MeanCourse object."""
        x, y, heading, distance, meta = self.generate()
        return MeanCourse(
            x=x, y=y, heading=heading, distance=distance,
            metadata={
                'type': 'U_shape',
                'width': self.width,
                'v_len': self.v_len,
                'd': self.d,
                'segments': meta,
            }
        )


def ornstein_uhlenbeck(distances, theta=5.0, sigma=0.15, seed=None):
    """Generate cross-track error using Ornstein-Uhlenbeck process.

    The OU process models realistic driving: smooth, mean-reverting
    deviations from the course centerline. Parameterized by arc length
    (not time) so noise statistics are speed-independent.

        dε = -θ·ε·ds + σ·dW

    Args:
        distances: Cumulative distance array (arc-length parameter)
        theta: Mean-reversion rate (higher = snaps back faster)
        sigma: Noise intensity (volatility)
        seed: Random seed for reproducibility

    Returns:
        Cross-track error array (same length as distances)
    """
    rng = np.random.RandomState(seed)
    n = len(distances)
    epsilon = np.zeros(n)
    for i in range(1, n):
        ds = distances[i] - distances[i - 1]
        if ds <= 0:
            continue
        sqrt_ds = np.sqrt(ds)
        epsilon[i] = (epsilon[i - 1]
                      - theta * epsilon[i - 1] * ds
                      + sigma * sqrt_ds * rng.randn())
    return epsilon


def simulate_driving(course_x, course_y, course_heading,
                     course_distance, num_laps=5, theta=5.0,
                     sigma=0.15, base_speed=2.0,
                     speed_variation=0.3, seed=42):
    """Simulate realistic multi-lap driving around a course.

    Uses Ornstein-Uhlenbeck process for cross-track error and speed
    variation. Each lap gets an independent noise realization, ensuring
    zero-mean errors that cancel when averaging laps.

    Args:
        course_x: Course centerline X coordinates
        course_y: Course centerline Y coordinates
        course_heading: Course heading angles (radians)
        course_distance: Cumulative distance along course
        num_laps: Number of laps to simulate
        theta: OU mean-reversion rate for cross-track error
        sigma: OU noise intensity for cross-track error
        base_speed: Base driving speed (m/s)
        speed_variation: OU noise intensity for speed variation
        seed: Master random seed

    Returns:
        PathData with concatenated multi-lap driving data
    """
    rng = np.random.RandomState(seed)
    all_x, all_y, all_h, all_t, all_v = [], [], [], [], []
    t_current = 0.0

    for lap in range(num_laps):
        lap_seed = rng.randint(0, 2**31)

        # Cross-track error (OU process)
        epsilon = ornstein_uhlenbeck(
            course_distance, theta=theta, sigma=sigma,
            seed=lap_seed)

        # Apply perpendicular offset: (-sin h, cos h) is the left
        # normal
        x_noisy = course_x + epsilon * (-np.sin(course_heading))
        y_noisy = course_y + epsilon * np.cos(course_heading)

        # Speed variation (OU process, smaller mean-reversion)
        speed_noise = ornstein_uhlenbeck(
            course_distance, theta=3.0, sigma=speed_variation,
            seed=lap_seed + 1)
        velocity = base_speed * (1 + speed_noise)
        velocity = np.maximum(velocity, 0.1)

        # Heading from actual trajectory
        dx = np.diff(x_noisy)
        dy = np.diff(y_noisy)
        heading = np.arctan2(dy, dx)
        heading = np.append(heading, heading[-1])

        # Timestamps from distance / speed
        seg_dist = np.sqrt(dx**2 + dy**2)
        avg_speed = (velocity[:-1] + velocity[1:]) / 2
        seg_time = seg_dist / avg_speed
        timestamps = np.concatenate([[0], np.cumsum(seg_time)])
        timestamps += t_current
        t_current = timestamps[-1] + 0.01

        all_x.append(x_noisy)
        all_y.append(y_noisy)
        all_h.append(heading)
        all_t.append(timestamps)
        all_v.append(velocity)

    return PathData(
        timestamp=np.concatenate(all_t),
        x=np.concatenate(all_x),
        y=np.concatenate(all_y),
        heading=np.concatenate(all_h),
        velocity=np.concatenate(all_v),
    )


def save_as_csv(path_data, filepath):
    """Save PathData as CSV in imupath format.

    Args:
        path_data: PathData object
        filepath: Output CSV file path
    """
    import pandas as pd
    df = pd.DataFrame({
        't': path_data.timestamp,
        'x': path_data.x,
        'y': path_data.y,
        'h': path_data.heading,
        'v': path_data.velocity,
    })
    df.to_csv(filepath, index=False)
