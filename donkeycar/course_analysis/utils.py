"""
Utility functions for angle calculations

Common utilities used across course analysis modules for handling
circular quantities (angles).
"""

import numpy as np


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
