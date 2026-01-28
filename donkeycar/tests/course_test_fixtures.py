"""
Shared test fixtures for course analysis tests.

Provides common course generation functions to avoid duplication
across test modules.
"""
import numpy as np
from donkeycar.course_analysis import (
    MeanCourse,
    CourseSegmenter,
    GradientSegmentation,
    ThresholdSegmentation,
)


def create_figure8_course(radius=5.0, num_points=400):
    """
    Create a figure-8 course that naturally has multiple segments.

    This creates clear curvature changes that segment detection can find.

    Args:
        radius: Size of figure-8 loops
        num_points: Number of points in course

    Returns:
        MeanCourse object with figure-8 geometry
    """
    t = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    # Figure-8 parametric equations
    x = radius * np.sin(t)
    y = radius * np.sin(t) * np.cos(t)

    # Compute heading from derivatives
    dx_dt = radius * np.cos(t)
    dy_dt = radius * (np.cos(t)**2 - np.sin(t)**2)
    heading = np.arctan2(dy_dt, dx_dt)

    # Compute cumulative distance
    dx = np.gradient(x)
    dy = np.gradient(y)
    ds = np.sqrt(dx**2 + dy**2)
    distance = np.cumsum(ds)

    return MeanCourse(
        x=x, y=y, heading=heading, distance=distance,
        metadata={'type': 'figure8', 'radius': radius}
    )


def create_segmented_course(method='threshold', radius=5.0, num_points=400):
    """
    Create a figure-8 course with segment boundaries.

    Args:
        method: 'threshold' or 'gradient' segmentation method
        radius: Size of figure-8 loops
        num_points: Number of points in course

    Returns:
        (mean_course, segmentation) tuple
    """
    mean_course = create_figure8_course(radius=radius, num_points=num_points)

    if method == 'threshold':
        strategy = ThresholdSegmentation()
    else:
        strategy = GradientSegmentation()

    segmenter = CourseSegmenter(
        strategy,
        params={
            'min_segment_length': 0.3,
            'straight_curvature_threshold': 0.05
        }
    )
    segmentation = segmenter.segment(mean_course)

    return mean_course, segmentation


def create_oval_course(length=20.0, width=10.0, num_points=400):
    """
    Generate oval test course with straights and hairpins.

    Args:
        length: Length of straight sections
        width: Width of oval (radius of turns)
        num_points: Number of points in course

    Returns:
        x, y, heading, distance arrays
    """
    # Parametric oval: two straights + two semicircles
    straight_len = length / 2
    turn_radius = width / 2

    # Arc lengths for each section
    straight_arc = straight_len
    turn_arc = np.pi * turn_radius
    total_arc = 2 * straight_arc + 2 * turn_arc

    # Normalized arc positions
    s = np.linspace(0, total_arc, num_points, endpoint=False)

    x = np.zeros(num_points)
    y = np.zeros(num_points)
    heading = np.zeros(num_points)

    for i, si in enumerate(s):
        if si < straight_arc:
            # First straight (going right)
            x[i] = si
            y[i] = 0
            heading[i] = 0
        elif si < straight_arc + turn_arc:
            # First turn (top semicircle)
            angle = (si - straight_arc) / turn_radius
            x[i] = straight_arc + turn_radius * np.sin(angle)
            y[i] = turn_radius * (1 - np.cos(angle))
            heading[i] = angle
        elif si < 2 * straight_arc + turn_arc:
            # Second straight (going left)
            x[i] = straight_arc - (si - straight_arc - turn_arc)
            y[i] = 2 * turn_radius
            heading[i] = np.pi
        else:
            # Second turn (bottom semicircle)
            angle = (si - 2 * straight_arc - turn_arc) / turn_radius
            x[i] = -turn_radius * np.sin(angle)
            y[i] = turn_radius * (1 + np.cos(angle))
            heading[i] = np.pi + angle

    # Verify heading is correct using actual path derivatives
    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.arctan2(dy, dx)

    # Compute distance
    dx_diff = np.diff(x)
    dy_diff = np.diff(y)
    ds = np.sqrt(dx_diff**2 + dy_diff**2)
    distance = np.concatenate([[0], np.cumsum(ds)])

    return x, y, heading, distance


def create_chicane_course(num_chicanes=3, amplitude=1.0, num_points=200):
    """
    Generate chicane course with rapid left-right-left transitions.

    Args:
        num_chicanes: Number of chicane segments
        amplitude: Amplitude of chicane oscillations
        num_points: Number of points in course

    Returns:
        x, y, heading, distance arrays
    """
    t = np.linspace(0, num_chicanes * 2 * np.pi, num_points)
    x = t / (2 * np.pi)  # Forward progression
    y = amplitude * np.sin(t)

    # Compute heading
    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.arctan2(dy, dx)

    # Compute cumulative distance
    ds = np.sqrt(dx**2 + dy**2)
    distance = np.cumsum(ds)

    return x, y, heading, distance


def create_mean_course_from_arrays(x, y, heading, distance):
    """
    Helper to create MeanCourse object from numpy arrays.

    Args:
        x: X coordinates
        y: Y coordinates
        heading: Heading angles (radians)
        distance: Cumulative distance

    Returns:
        MeanCourse object
    """
    return MeanCourse(
        x=np.array(x),
        y=np.array(y),
        heading=np.array(heading),
        distance=np.array(distance),
        metadata={'length': distance[-1] if len(distance) > 0 else 0.0}
    )


def simulate_perfect_lap(mean_course):
    """
    Simulate perfect driving along mean course.

    Args:
        mean_course: MeanCourse object

    Returns:
        x_path, y_path arrays
    """
    return np.array(mean_course.x), np.array(mean_course.y)


def simulate_wobbly_lap(mean_course, wobble_amplitude=0.1, wobble_freq=5):
    """
    Simulate lateral oscillations around mean course.

    Args:
        mean_course: MeanCourse object
        wobble_amplitude: Amplitude of lateral deviation (meters)
        wobble_freq: Frequency of oscillation (cycles per lap)

    Returns:
        x_path, y_path arrays
    """
    n = len(mean_course.x)
    t = np.linspace(0, 2 * np.pi * wobble_freq, n)
    heading_rad = mean_course.heading
    wobble = wobble_amplitude * np.sin(t)

    # Perpendicular direction (left of course)
    perp_x = -np.sin(heading_rad)
    perp_y = np.cos(heading_rad)

    x_path = mean_course.x + wobble * perp_x
    y_path = mean_course.y + wobble * perp_y

    return x_path, y_path
