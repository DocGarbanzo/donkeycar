"""
Test segment identification in multi-lap IMU path visualization

Tests the complete UI workflow: loading multi-lap data, computing mean course
from N laps, and assigning segments to the driven path. This simulates what
happens when a user selects different lap counts in the visualization UI.

CRITICAL: These tests simulate the REAL UI workflow in imu_visualization.py:
1. Load multi-lap CSV data
2. User selects N laps in UI
3. Mean course is computed from laps 1..N (NOT just lap 1!)
4. Segments are assigned to the full driven path
5. Check that lap 1 has proper segment transitions
"""

import unittest
import numpy as np
import pandas as pd
import pytest
import tempfile
import os
from donkeycar.course_analysis import (
    MultiLapData, MeanCourse, CourseSegmentation
)


def create_multilap_csv(num_laps=3, points_per_lap=200):
    """
    Create a CSV file with multi-lap circular data

    Simulates real recorded data with multiple laps that can be
    loaded by MultiLapData and processed like the UI does.

    Each lap starts just below y=0, makes a full circle, and
    returns to cross y=0 from below (negative to positive).
    """
    all_t, all_x, all_y, all_heading, all_velocity = [], [], [], [], []

    for lap in range(num_laps):
        # Start at bottom of circle (θ=-π/2, y=-2) and make full circle
        # to return to near y=0 for crossing detection
        # Offset slightly to ensure clear crossing
        theta = np.linspace(-np.pi/2 + 0.1, 3*np.pi/2 + 0.1, points_per_lap)

        x = 2.0 * np.cos(theta)
        y = 2.0 * np.sin(theta)
        heading = theta + np.pi/2
        velocity = np.ones(len(theta)) * 2.0  # Constant velocity

        # Add noise
        np.random.seed(42 + lap)
        x += np.random.normal(0, 0.05, len(x))
        y += np.random.normal(0, 0.05, len(y))

        t = np.linspace(lap * 10, (lap + 1) * 10, points_per_lap)

        all_t.extend(t)
        all_x.extend(x)
        all_y.extend(y)
        all_heading.extend(heading)
        all_velocity.extend(velocity)

    # Create CSV file with correct column names (t, x, y, h, v)
    temp_csv = tempfile.NamedTemporaryFile(
        mode='w', suffix='.csv', delete=False
    )
    temp_csv.write('t,x,y,h,v\n')
    for i in range(len(all_t)):
        temp_csv.write(
            f'{all_t[i]},{all_x[i]},{all_y[i]},{all_heading[i]},{all_velocity[i]}\n'
        )
    temp_csv.close()
    return temp_csv.name


def compute_mean_course_from_n_laps(csv_file, num_laps_to_use):
    """
    Compute mean course using first N laps

    This simulates what the UI does when user selects N laps:
    apply_lap_selection() -> compute_mean_course_with_laps(N)
    """
    from donkeycar.course_analysis import (
        CSVPathDataSource, YCrossingLapDetector, 
        MeanCourseBuilder
    )
    
    # Load path data from CSV
    source = CSVPathDataSource(csv_file)
    
    # Detect laps using Y-crossing detector with specified params
    detector = YCrossingLapDetector(params={
        'y_threshold': 0.1,
        'min_loop_distance': 1.0,
        'min_lap_length': 50
    })
    
    # Create MultiLapData from source and detector
    multilap_data = MultiLapData.from_source(source, detector)

    # Limit to N laps (like the real code does)
    if multilap_data.num_laps > num_laps_to_use:
        limited_boundaries = multilap_data.lap_boundaries[:num_laps_to_use]
        multilap_data = MultiLapData(multilap_data.path_data, limited_boundaries)

    # Build mean course from these laps
    builder = MeanCourseBuilder()
    mean_course = builder.build(multilap_data)
    
    return mean_course, multilap_data


@pytest.mark.parametrize("segment_method", ["extrema", "gradient", "threshold", "hybrid"])
@pytest.mark.parametrize("num_laps_for_mean", [1, 2, 3])
def test_segment_assignment_with_n_lap_mean_course(segment_method, num_laps_for_mean):
    """
    Test segment assignment when mean course is computed from N laps

    This is the REAL UI workflow test:
    - Load 3-lap data
    - Compute mean course from first N laps
    - Assign segments to full 3-lap path
    - Verify lap 1 has multiple segments

    BUG: When num_laps_for_mean > 1, lap 1 gets stuck at segment 0
    """
    from donkeycar.course_analysis import (
        CourseSegmenter, GradientSegmentation, ThresholdSegmentation,
        ExtremaSegmentation, HybridSegmentation, SegmentAssigner
    )
    
    csv_file = create_multilap_csv(num_laps=3)

    try:
        # Load full driven path (all 3 laps)
        df = pd.read_csv(csv_file)

        # Simulate user selecting N laps
        mean_course, multilap_data = compute_mean_course_from_n_laps(
            csv_file, num_laps_to_use=num_laps_for_mean
        )

        # Create segmentation from N-lap mean course using new API
        if segment_method == 'gradient':
            strategy = GradientSegmentation()
        elif segment_method == 'threshold':
            strategy = ThresholdSegmentation()
        elif segment_method == 'extrema':
            strategy = ExtremaSegmentation()
        elif segment_method == 'hybrid':
            strategy = HybridSegmentation()
        else:
            pytest.skip(f"Unknown segmentation method: {segment_method}")
            
        segmenter = CourseSegmenter(strategy)
        segmentation = segmenter.segment(mean_course)

        if segmentation.total_segments == 0:
            pytest.skip(f"No segments detected with {segment_method}")

        if segmentation.total_segments == 1:
            pytest.skip(f"Only 1 segment detected with {segment_method} "
                       f"(circle too simple for this method)")

        # Assign segments to FULL path using new API
        assigner = SegmentAssigner(segmentation)
        path_segment_ids = assigner.assign(df['x'].values, df['y'].values)

        # Get ACTUAL lap 1 end from lap detection boundary
        if len(multilap_data.lap_boundaries) > 0:
            lap1_end_idx = multilap_data.lap_boundaries[0].end_index
        else:
            pytest.skip("No laps detected")

        # Check lap 1 has multiple segments
        lap1_segments = set(path_segment_ids[:lap1_end_idx + 1])

        # CRITICAL ASSERTION: This should fail when num_laps_for_mean > 1
        assert len(lap1_segments) > 1, \
            f"BUG DETECTED: Lap 1 has only {len(lap1_segments)} segment(s): " \
            f"{lap1_segments}. " \
            f"Method={segment_method}, mean_from_laps={num_laps_for_mean}. " \
            f"Lap 1 has {lap1_end_idx + 1} points, " \
            f"total segments={segmentation.total_segments}"

    finally:
        if os.path.exists(csv_file):
            os.unlink(csv_file)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
