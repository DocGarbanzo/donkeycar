"""
IMU Path Visualization Utilities

This module provides utilities for visualizing and analyzing IMU
path data, including real-time plotting and post-processing
visualization with drift correction.

Functions:
    visualize_imu_path: Main function for interactive IMU path visualization
    correct_loop_drift: Drift correction algorithm for closed-loop paths
    plot: Real-time plotting function

Classes:
    PathPlotter: Real-time path plotting utility for IMU data
"""

import multiprocessing
from multiprocessing import Process, Value
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider, TextBox
import time
import numpy as np
import pandas as pd
import os
from datetime import datetime


def plot(mlist, limit=5, update_freq=0, running=Value('i', 1)):
    """
    Real-time plotting function for PathPlotter.

    Args:
        mlist: Shared list containing path data points
        limit: Plot axis limits (-limit to +limit)
        update_freq: Update frequency in Hz (0 for unlimited)
        running: Shared value to control plot loop
    """
    plt.style.use('dark_background')
    plt.ion()
    fig = plt.figure()
    ax = plt.axes(xlim=(-limit, limit), ylim=(-limit, limit))
    line, = ax.plot([], [], lw=2)
    fig.canvas.draw()
    tic = time.time()
    count = 0
    while running.value == 1:
        try:
            np_path = np.array(mlist)
            line.set_xdata(np_path[:, 1])
            line.set_ydata(np_path[:, 2])
            fig.canvas.draw()
            fig.canvas.flush_events()
        except Exception as e:
            pass
        toc = time.time()
        dtime = toc - tic
        if update_freq and dtime < 1.0 / update_freq:
            time.sleep(1.0 / update_freq - dtime)
        tic = toc
        count += 1
    print(f"Ran plot job for {count} iterations")


class PathPlotter:
    """
    Real-time path plotting utility for IMU data.

    Creates a separate process that continuously plots path data as it's
    collected from the IMU sensor.
    """

    def __init__(self, update_freq=0, limit=5):
        """
        Initialize PathPlotter.

        Args:
            update_freq: Update frequency in Hz (0 for unlimited)
            limit: Plot axis limits (-limit to +limit)
        """
        self.manager = multiprocessing.Manager()
        self.mlist = self.manager.list()
        self.update_freq = update_freq
        self.plot_proc = None
        self.running = Value('i', 1)
        self.limit = limit

    def start(self):
        """Start the plotting process."""
        self.plot_proc = Process(
            target=plot, args=(self.mlist, self.limit,
                               self.update_freq, self.running))
        self.plot_proc.start()

    def stop(self):
        """Stop the plotting process."""
        self.running.value = 0
        if self.plot_proc is not None:
            self.plot_proc.join()

    def append(self, x):
        """
        Add a new data point to the plot.

        Args:
            x: 2-element sequence containing (x, y) coordinates
        """
        if len(x) != 2:
            raise ValueError("Input must be a 2-sequence")
        self.mlist.append((time.time(), *x))


def correct_loop_drift(df, min_loop_distance=1.0, max_distance=0.5,
                       max_loops=10):
    """
    Correct drift in closed-loop IMU path data using iterative
    loop-by-loop approach. Uses weighted average reversal point
    detection for accurate loop end identification. Detects and
    corrects multiple loops until no more are found or max_loops
    reached.

    Args:
        df (DataFrame): IMU data with columns t, x, y, z, v
        min_loop_distance (float): Minimum distance to travel before
                                    considering loop closure
        max_distance (float): Maximum distance from origin to start
                              looking for reversal point
        max_loops (int): Maximum number of loops to detect (default 10)

    Returns:
        tuple: (corrected_df, list_of_loop_end_indices,
                list_of_drift_amounts)
    """
    if len(df) < 2:
        return df.copy(), [], []

    def show_sample_points(data, start_idx, end_idx, label,
                           show_timestamps=True):
        """Show 10 equidistant sample points from a range"""
        if end_idx <= start_idx:
            print(f"  {label}: No points to show")
            return

        num_points = min(10, end_idx - start_idx + 1)
        indices = np.linspace(start_idx, end_idx, num_points, dtype=int)
        print(f"  {label} (showing {len(indices)} sample points):")
        for i, idx in enumerate(indices):
            x = data.iloc[idx]['x']
            y = data.iloc[idx]['y']

            if not show_timestamps:
                print(f"    [{i+1}] idx={idx}, pos=[{x:.3f}, {y:.3f}]")
                continue

            t = data.iloc[idx]['t']
            print(f"    [{i+1}] idx={idx}, t={t:.1f}, "
                  f"pos=[{x:.3f}, {y:.3f}]")

    def find_loop_end(data, start_idx, min_distance_traveled,
                      max_distance_from_origin):
        """
        Find loop end using weighted average reversal point detection.

        Args:
            data (DataFrame): Path data
            start_idx (int): Loop start index
            min_distance_traveled (float): Minimum distance to travel
                                            before considering loop end
            max_distance_from_origin (float): Max distance from origin
                                               to start looking for
                                               reversal

        Returns:
            int or None: Index of loop end point, or None if not found
        """
        # Need at least 3 points for weighted average
        if start_idx >= len(data) - 3:
            return None

        # Calculate cumulative distance from loop start
        dx = np.diff(data['x'].iloc[start_idx:])
        dy = np.diff(data['y'].iloc[start_idx:])
        distances = np.sqrt(dx**2 + dy**2)
        cumulative_distance = np.concatenate([[0], np.cumsum(distances)])

        # Get loop start position (origin for first loop,
        # actual position for subsequent loops)
        if start_idx == 0:
            loop_start_pos = np.array([0.0, 0.0])
        else:
            loop_start_pos = np.array([data.iloc[start_idx]['x'],
                                       data.iloc[start_idx]['y']])

        print(f"  Looking for loop end starting at index {start_idx}")
        print(f"  Loop start position: "
              f"[{loop_start_pos[0]:.3f}, {loop_start_pos[1]:.3f}]")
        print(f"  Parameters: min_distance={min_distance_traveled:.1f}m, "
              f"max_distance={max_distance_from_origin:.1f}m")

        # Phase 1: Travel minimum distance
        min_distance_idx = None
        for i in range(len(cumulative_distance)):
            if cumulative_distance[i] >= min_distance_traveled:
                min_distance_idx = start_idx + i
                print(f"  Minimum distance {min_distance_traveled:.1f}m "
                      f"reached at index {min_distance_idx}")
                break

        if min_distance_idx is None:
            print(f"  Not enough distance traveled from loop start")
            return None

        # Phase 2: Find when we get close to the loop start
        vicinity_start_idx = None
        print(f"  Searching for vicinity from index "
              f"{min_distance_idx} to {len(data)-1}")

        # Debug: show some sample distances
        sample_start = min_distance_idx - start_idx
        sample_end = min(sample_start + 50, len(cumulative_distance))
        sample_indices = list(range(sample_start, sample_end, 10))
        print(f"  Sample distances after min_distance threshold:")
        for i in sample_indices:
            actual_idx = start_idx + i
            if actual_idx >= len(data):
                break
            current_pos = np.array([data.iloc[actual_idx]['x'],
                                    data.iloc[actual_idx]['y']])
            distance_to_start = np.linalg.norm(current_pos -
                                               loop_start_pos)
            print(f"    Index {actual_idx}: "
                  f"pos=[{current_pos[0]:.3f}, {current_pos[1]:.3f}], "
                  f"distance={distance_to_start:.3f}m")

        for i in range(min_distance_idx - start_idx,
                       len(cumulative_distance)):
            actual_idx = start_idx + i
            if actual_idx >= len(data):
                break

            current_pos = np.array([data.iloc[actual_idx]['x'],
                                    data.iloc[actual_idx]['y']])
            distance_to_start = np.linalg.norm(current_pos -
                                               loop_start_pos)

            if distance_to_start <= max_distance_from_origin:
                vicinity_start_idx = actual_idx
                print(f"  Entered vicinity of loop start at index "
                      f"{vicinity_start_idx}, "
                      f"distance={distance_to_start:.3f}m")
                break

        if vicinity_start_idx is None:
            print(f"  Never got within "
                  f"{max_distance_from_origin:.1f}m of loop start")
            print(f"  Try increasing max_distance parameter "
                  f"(current: {max_distance_from_origin:.1f}m)")
            return None

        # Phase 3: Find reversal point using weighted average
        print(f"  Searching for reversal point starting from index "
              f"{vicinity_start_idx}")

        # Calculate distances to loop start for all remaining points
        distances_to_start = []
        for i in range(vicinity_start_idx, len(data)):
            current_pos = np.array([data.iloc[i]['x'],
                                    data.iloc[i]['y']])
            dist = np.linalg.norm(current_pos - loop_start_pos)
            distances_to_start.append(dist)

        # Find reversal point: look for where distance stops
        # decreasing and starts increasing. Use weighted average
        # to smooth out measurement noise

        # Need at least 7 points for proper reversal detection
        if len(distances_to_start) < 7:
            print(f"  Not enough points for reversal detection")
            return None

        # Find the LAST reversal point (not the first) - the
        # actual loop closure. Look for all potential reversal
        # points and select the one with minimum distance
        potential_reversals = []

        for i in range(3, len(distances_to_start) - 3):
            # Calculate weighted averages for current and next few points
            current_avg = (distances_to_start[i - 1] * 0.25 +
                           distances_to_start[i] * 0.5 +
                           distances_to_start[i + 1] * 0.25)

            # Look ahead to see if distances are consistently increasing
            next_points = distances_to_start[i + 1:i + 4]  # Next 3 points
            if len(next_points) == 3:
                next_avg = sum(next_points) / len(next_points)

                # If next average is higher, this is a potential reversal
                # 0.1% tolerance for gentle increases
                if next_avg > current_avg * 1.001:
                    actual_idx = vicinity_start_idx + i
                    potential_reversals.append(
                        (actual_idx, current_avg, next_avg))

        # Select reversal occurring soon after entering vicinity
        # (prioritize early + close)
        # This prevents selecting late coincidental approaches to the origin
        if potential_reversals:
            # Filter for reversals that are reasonably close (within 2x of
            # max_distance)
            good_reversals = [
                r for r in potential_reversals
                if r[1] <= max_distance_from_origin * 2]

            if good_reversals:
                # Prioritize reversals that occur soon after entering vicinity
                # Use a weighted score: earlier time + closer distance
                # Look within next 2000 points
                vicinity_window = min(2000, len(data) - vicinity_start_idx)

                scored_reversals = []
                for reversal in good_reversals:
                    idx, distance, next_avg = reversal
                    # Only consider reversals within reasonable window of
                    # vicinity start
                    if idx <= vicinity_start_idx + vicinity_window:
                        time_factor = (idx - vicinity_start_idx) / \
                            vicinity_window  # 0 = earliest, 1 = latest
                        # 0 = closest, 1+ = farther
                        distance_factor = (distance /
                                           max_distance_from_origin)
                        # Prioritize early time
                        score = (time_factor * 0.7 +
                                 distance_factor * 0.3)
                        scored_reversals.append((score, reversal))

                if scored_reversals:
                    # Select the reversal with the best (lowest) score
                    best_score, best_reversal = min(scored_reversals)
                    reversal_idx, current_avg, next_avg = best_reversal
                    print(f"  Found {len(potential_reversals)} potential "
                          f"reversals, {len(good_reversals)} within 2x "
                          f"max_distance")
                    print(f"  {len(scored_reversals)} within vicinity "
                          f"window, selected best scoring at index "
                          f"{reversal_idx}")
                    print(f"    Score: {best_score:.3f} (lower=better), "
                          f"Distance: {current_avg:.3f}m")
                    pct_increase = ((next_avg/current_avg - 1)*100)
                    print(f"    Distance increase: {pct_increase:.1f}%")
                    return reversal_idx
                else:
                    # No reversals in vicinity window, take earliest good one
                    best_reversal = min(good_reversals, key=lambda x: x[0])
                    reversal_idx, current_avg, next_avg = best_reversal
                    print(f"  No reversals in vicinity window, "
                          f"selected earliest good reversal at "
                          f"index {reversal_idx}")
                    return reversal_idx
            else:
                # Fallback: if no good reversals, take the earliest of all
                best_reversal = min(potential_reversals, key=lambda x: x[0])
                reversal_idx, current_avg, next_avg = best_reversal
                print(f"  Found {len(potential_reversals)} potential "
                      f"reversals, none within 2x max_distance")
                print(f"  Selected EARLIEST reversal at index {reversal_idx}")
                print(f"    Current avg distance: {current_avg:.3f}m")
                return reversal_idx

        # Fallback: if no clear reversal found, use the minimum distance point
        min_distance = min(distances_to_start)
        min_idx = distances_to_start.index(min_distance)
        min_distance_idx = vicinity_start_idx + min_idx
        print(f"  No clear reversal found, using minimum distance "
              f"point at index {min_distance_idx}")
        print(f"    Minimum distance: {min_distance:.3f}m")
        return min_distance_idx

    # Work with a copy
    corrected_df = df.copy()

    print(f"Starting drift correction with {len(df)} total points")
    print(f"Parameters: min_loop_distance={min_loop_distance}, "
          f"max_distance={max_distance}, max_loops={max_loops}")

    loop_end_indices = []
    loop_drift_amounts = []
    current_loop_start = 0
    loop_num = 1

    # ============= ITERATIVE LOOP DETECTION =============
    while loop_num <= max_loops:
        print(f"\n" + "=" * 60)
        print(f"STEP {loop_num}: Finding Loop {loop_num}")
        print("=" * 60)

        # Find loop closure using the algorithm
        loop_end_idx = find_loop_end(
            corrected_df,
            current_loop_start,
            min_loop_distance,
            max_distance)

        if loop_end_idx is None:
            print(f"No Loop {loop_num} closure found!")
            if loop_num == 1:
                print("No loops detected in data")
                return corrected_df, [], []
            else:
                print(f"Found {loop_num-1} loops total")
                break

        # Found a loop!
        t = corrected_df.iloc[loop_end_idx]['t']
        print(f"{loop_num}) LOOP {loop_num} END FOUND: "
              f"index={loop_end_idx}, timestamp={t:.1f}")

        loop_end_indices.append(loop_end_idx)

        # Calculate loop drift
        loop_start_pos = np.array([
            corrected_df.iloc[current_loop_start]['x'],
            corrected_df.iloc[current_loop_start]['y']
        ])
        loop_end_pos = np.array([
            corrected_df.iloc[loop_end_idx]['x'],
            corrected_df.iloc[loop_end_idx]['y']
        ])
        loop_drift = loop_end_pos - loop_start_pos
        loop_drift_amounts.append(loop_drift)

        print(f"{loop_num+1}) CORRECTION AMOUNT for Loop {loop_num}: "
              f"[{loop_drift[0]:.3f}, {loop_drift[1]:.3f}]")

        # Show loop points before correction
        print(f"{loop_num+2}) LOOP {loop_num} BEFORE CORRECTION:")
        show_sample_points(
            corrected_df,
            current_loop_start,
            loop_end_idx,
            f"Loop {loop_num} points before correction")

        # Apply proportional correction to current loop
        loop_length = loop_end_idx - current_loop_start
        if loop_length > 0:
            for i in range(current_loop_start, loop_end_idx + 1):
                prop_factor = (i - current_loop_start) / loop_length
                correction = loop_drift * prop_factor
                corrected_df.iloc[i, corrected_df.columns.get_loc(
                    'x')] -= correction[0]
                corrected_df.iloc[i, corrected_df.columns.get_loc(
                    'y')] -= correction[1]

        # Debug: Check the actual corrected end position
        corrected_end_pos = np.array([
            corrected_df.iloc[loop_end_idx]['x'],
            corrected_df.iloc[loop_end_idx]['y']
        ])
        print(f"   DEBUG: Loop {loop_num} end position after "
              f"correction: [{corrected_end_pos[0]:.6f}, "
              f"{corrected_end_pos[1]:.6f}]")

        # Show loop points after correction
        print(f"{loop_num+2}) LOOP {loop_num} AFTER CORRECTION:")
        show_sample_points(
            corrected_df,
            current_loop_start,
            loop_end_idx,
            f"Loop {loop_num} points after correction")

        # Apply absolute drift correction to all remaining points
        if loop_end_idx < len(corrected_df) - 1:
            print(f"{loop_num+3}) REMAINING POINTS BEFORE GLOBAL SHIFT:")
            show_sample_points(
                corrected_df,
                loop_end_idx + 1,
                len(corrected_df) - 1,
                "Remaining points before shift")

            # Apply full drift correction to all remaining points
            for i in range(loop_end_idx + 1, len(corrected_df)):
                corrected_df.iloc[i, corrected_df.columns.get_loc(
                    'x')] -= loop_drift[0]
                corrected_df.iloc[i, corrected_df.columns.get_loc(
                    'y')] -= loop_drift[1]

            print(f"{loop_num+3}) REMAINING POINTS AFTER GLOBAL SHIFT:")
            show_sample_points(
                corrected_df,
                loop_end_idx + 1,
                len(corrected_df) - 1,
                "Remaining points after shift")

        # Set up for next loop
        current_loop_start = loop_end_idx
        loop_num += 1

    # Final summary
    print(f"\n" + "=" * 60)
    print("DRIFT CORRECTION COMPLETE")
    print("=" * 60)

    num_loops_found = len(loop_end_indices)
    print(f"Found and corrected {num_loops_found} loops:")
    for i, end_idx in enumerate(loop_end_indices):
        loop_start = 0 if i == 0 else loop_end_indices[i - 1]
        print(f"  Loop {i+1}: indices {loop_start} to {end_idx}")
        end_x = corrected_df.iloc[end_idx]['x']
        end_y = corrected_df.iloc[end_idx]['y']
        print(f"    End position: [{end_x:.6f}, {end_y:.6f}]")

    print(f"\nFinal result: {len(corrected_df)} points with "
          f"{num_loops_found} loops corrected")
    print(f"Final data range: "
          f"x=[{corrected_df['x'].min():.3f}, "
          f"{corrected_df['x'].max():.3f}], "
          f"y=[{corrected_df['y'].min():.3f}, "
          f"{corrected_df['y'].max():.3f}]")

    return corrected_df, loop_end_indices, loop_drift_amounts


def visualize_imu_path(data_source='imu.csv', correct_drift=False,
                       min_loop_distance=1.0, max_distance=0.5,
                       downsample_factor=None, segment_method='gradient'):
    """
    Load and visualize IMU path data with interactive time slider.

    Args:
        data_source (str): Path to CSV file or Tub directory containing
                           IMU path data
        correct_drift (bool): Apply loop drift correction to path data
                              (default: False)
        min_loop_distance (float): Minimum distance to travel before
                                    considering loop closure
        max_distance (float): Maximum distance from origin to start
                              looking for reversal point
        downsample_factor (int): Downsample factor for display
                                 (default: auto-calculate to ~10000 pts)
        segment_method (str): Segmentation method - 'threshold', 'extrema',
                              'gradient', or 'hybrid' (default: 'gradient')
    """
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider
    import math

    # Expand user path
    data_source = os.path.expanduser(data_source)

    # Check if source exists
    if not os.path.exists(data_source):
        print(f"File or directory {data_source} not found.")
        return

    # Determine if source is CSV or Tub directory
    is_csv = os.path.isfile(data_source) and data_source.endswith('.csv')
    is_tub = os.path.isdir(data_source)

    if not is_csv and not is_tub:
        print(f"Source must be either a CSV file or a Tub directory")
        return

    # Load data based on source type
    if is_csv:
        df = pd.read_csv(data_source)
        if df.empty:
            print(f"No data found in {data_source}")
            return
        # CSV format: t, x, y, h (heading in degrees), v (velocity)
        # Ensure expected columns exist
        if not all(col in df.columns for col in ['t', 'x', 'y', 'v']):
            print(f"CSV must contain columns: t, x, y, v")
            return
        print(f"Loaded {len(df)} data points from CSV: {data_source}")
    else:
        # Load from Tub
        from donkeycar.parts.tub_v2 import Tub
        tub = Tub(data_source, read_only=True)

        # Extract data from tub records
        data_rows = []
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

            # Calculate heading from euler angles (car/euler is [x, y, z] in degrees)
            euler = record.get('car/euler', [0, 0, 0])
            h = 90.0 - euler[2]

            data_rows.append({'t': t, 'x': x, 'y': y, 'h': h, 'v': v})

        tub.close()

        if not data_rows:
            print(f"No IMU path data found in Tub: {data_source}")
            return

        df = pd.DataFrame(data_rows)
        print(f"Loaded {len(df)} data points from Tub: {data_source}")

    # Compute mean course BEFORE drift correction (uses original data)
    # Create helper function to compute mean course with specific lap count
    def compute_mean_course_with_laps(num_laps_to_use):
        """Compute mean course using only first N laps"""
        try:
            from donkeycar.parts.course_analysis import MultiLapData, MeanCourse
            import tempfile
            import math

            print(f"\nComputing mean course from {num_laps_to_use} lap(s)...")

            # Create temporary CSV file for course analysis
            temp_csv = tempfile.NamedTemporaryFile(
                mode='w', suffix='.csv', delete=False)
            temp_csv.write('timestamp,x,y,heading\n')
            for idx in range(len(df)):
                t = df.iloc[idx]['t']
                x = df.iloc[idx]['x']
                y = df.iloc[idx]['y']
                h = df.iloc[idx]['h']
                heading_rad = math.radians(h)
                temp_csv.write(f'{t},{x},{y},{heading_rad}\n')
            temp_csv.close()

            # Run course analysis on original data
            multilap_data = MultiLapData()
            multilap_data.load_data(
                temp_csv.name,
                lap_detection_method='y_crossing',
                min_loop_distance=min_loop_distance,
                y_threshold=0.1,
                min_lap_length=50
            )

            # Limit to requested number of laps
            if multilap_data.num_laps > num_laps_to_use:
                print(f"  Limiting from {multilap_data.num_laps} to "
                      f"{num_laps_to_use} lap(s)")
                multilap_data.laps = multilap_data.laps[:num_laps_to_use]
                multilap_data.num_laps = num_laps_to_use

            if multilap_data.num_laps > 0:
                print(f"  Using {multilap_data.num_laps} lap(s) for mean course")
                mean_course = MeanCourse(multilap_data)
                mean_course.compute()
                result = {
                    'x': mean_course.x.copy(),
                    'y': mean_course.y.copy(),
                    'heading': mean_course.heading.copy(),
                    'distance': mean_course.distance.copy(),
                    'length': mean_course.distance[-1],
                    'max_laps': multilap_data.num_laps,
                    'mean_course_obj': mean_course
                }
                print(f"  Mean course length: {result['length']:.2f}m")
                # Clean up temp file
                os.unlink(temp_csv.name)
                return result
            else:
                print("  No laps detected - mean course not available")
                os.unlink(temp_csv.name)
                return None

        except Exception as e:
            print(f"  Could not compute mean course: {e}")
            import traceback
            traceback.print_exc()
            return None

    # Initial computation with all detected laps
    # Also get lap boundary indices for filtering display
    lap_end_indices = []
    try:
        from donkeycar.parts.course_analysis import MultiLapData
        import tempfile
        import math

        # Detect laps to get boundary indices
        temp_csv = tempfile.NamedTemporaryFile(
            mode='w', suffix='.csv', delete=False)
        temp_csv.write('timestamp,x,y,heading\n')
        for idx in range(len(df)):
            t = df.iloc[idx]['t']
            x = df.iloc[idx]['x']
            y = df.iloc[idx]['y']
            h = df.iloc[idx]['h']
            heading_rad = math.radians(h)
            temp_csv.write(f'{t},{x},{y},{heading_rad}\n')
        temp_csv.close()

        multilap_data = MultiLapData()
        multilap_data.load_data(
            temp_csv.name,
            lap_detection_method='y_crossing',
            min_loop_distance=min_loop_distance,
            y_threshold=0.1,
            min_lap_length=50
        )

        # Get lap end indices
        for lap in multilap_data.laps:
            # Find the index in original df for the last point of this lap
            last_t = lap['timestamp'][-1]
            end_idx = df[df['t'] <= last_t].index[-1]
            lap_end_indices.append(end_idx)

        os.unlink(temp_csv.name)
        print(f"  Lap boundaries: {lap_end_indices}")

    except Exception as e:
        print(f"  Could not determine lap boundaries: {e}")
        lap_end_indices = []

    mean_course_data = compute_mean_course_with_laps(100)  # Use all laps
    max_laps_detected = mean_course_data['max_laps'] if mean_course_data else 1

    # Segment the mean course (if available)
    segmentation = None
    if mean_course_data is not None:
        try:
            from donkeycar.parts.course_analysis import CourseSegmentation
            segmentation = CourseSegmentation(
                mean_course=mean_course_data['mean_course_obj'],
                params={'boundary_method': segment_method})
            segmentation.compute()
            print(f"Segmented mean course into "
                  f"{segmentation.total_segments} segments "
                  f"using '{segment_method}' method")
        except Exception as exc:
            print(f"Could not compute segmentation: {exc}")
            segmentation = None

    # Apply drift correction if requested (AFTER computing mean course)
    loop_drift_amounts = []
    if correct_drift:
        print("\nApplying loop drift correction...")
        df, loop_end_indices, loop_drift_amounts = correct_loop_drift(
            df, min_loop_distance, max_distance)
        print("Drift correction completed.")
    else:
        # Use lap boundaries detected by MultiLapData
        loop_end_indices = lap_end_indices

        # DEBUG: Verify what data the visualization actually received
        print(f"\nVISUALIZATION DEBUG - Data received for plotting:")
        print(f"  Total points: {len(df)}")
        print(f"  Final 3 points to be visualized:")
        for i in range(max(0, len(df) - 3), len(df)):
            x = df.iloc[i]['x']
            y = df.iloc[i]['y']
            t = df.iloc[i]['t']
            print(f"    idx={i}, pos=[{x:.6f}, {y:.6f}], t={t:.1f}")
        print(f"  Data range for plot: "
              f"x=[{df['x'].min():.3f}, {df['x'].max():.3f}], "
              f"y=[{df['y'].min():.3f}, {df['y'].max():.3f}]")

    lap_start_indices = [0]
    for end_idx in loop_end_indices[:-1]:
        next_idx = min(len(df) - 1, end_idx + 1)
        lap_start_indices.append(next_idx)

    cumulative_distance = np.zeros(len(df), dtype=np.float64)
    if len(df) > 1:
        dx = np.diff(df['x'].to_numpy())
        dy = np.diff(df['y'].to_numpy())
        step_dist = np.sqrt(dx**2 + dy**2)
        cumulative_distance[1:] = np.cumsum(step_dist)

    # Convert UTC timestamps to datetime objects for better display
    start_time = datetime.fromtimestamp(df['t'].min())
    end_time = datetime.fromtimestamp(df['t'].max())
    print(f"Time range: "
          f"{start_time.isoformat(timespec='milliseconds')} to "
          f"{end_time.isoformat(timespec='milliseconds')}")
    dist_traveled = np.sqrt(df['x'].iloc[-1]**2 + df['y'].iloc[-1]**2)
    print(f"Distance traveled: {dist_traveled:.2f} units")

    # Set up the figure and axis
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(12, 8))
    fig.canvas.manager.set_window_title('Donkey imupath')
    plt.subplots_adjust(bottom=0.2, right=0.95, top=0.87,
                        left=0.08)

    # Initial plot setup
    ax.set_xlabel('X Position (Right)')
    ax.set_ylabel('Y Position (Forward)')
    ax.set_title('IMU 2D Path Visualization')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Create color map based on speed
    speeds: np.ndarray = df['v'].to_numpy(dtype=np.float32)

    # Plot full trajectory (faded) - no downsampling for accurate navigation
    if downsample_factor is None:
        downsample_factor = 1  # No downsampling
    df_display = df[::downsample_factor]
    speeds_display = speeds[::downsample_factor]

    # Use median speed color for legend instead of min/max
    median_speed = np.median(speeds_display)
    full_path_scatter = ax.scatter(
        df_display['x'],
        df_display['y'],
        c=speeds_display,
        cmap='viridis',
        alpha=0.3,
        s=8,
        label='Full path')

    # Create a dummy artist for legend with median color
    from matplotlib.lines import Line2D
    full_path_legend = Line2D([0], [0], marker='o', color='w',
                              markerfacecolor=plt.cm.viridis(median_speed / speeds_display.max() if speeds_display.max() > 0 else 0.5),
                              markersize=8, label='Full path', alpha=0.6)

    # Current position marker
    current_pos = ax.scatter(
        [],
        [],
        c='red',
        s=100,
        marker='o',
        label='Current position')

    # Path up to current time
    current_path, = ax.plot([], [], color='#FF6B6B', linewidth=1,
                            alpha=0.8, label='Path to current time')

    # Mean course overlay (if available)
    mean_course_line = None
    mean_course_segments = []

    def refresh_mean_course():
        """Draw mean course with each segment in a different color."""
        nonlocal mean_course_line, mean_course_segments

        # Clear existing segment lines
        for line in mean_course_segments:
            line.remove()
        mean_course_segments = []

        # Clear existing legend line
        if mean_course_line is not None:
            mean_course_line.remove()
            mean_course_line = None

        if mean_course_data is None:
            return

        if segmentation is None or segmentation.total_segments == 0:
            _draw_unsegmented_mean_course()
            return

        _draw_segmented_mean_course()

    def _draw_unsegmented_mean_course():
        """Draw mean course as single orange line."""
        nonlocal mean_course_line
        course_len = mean_course_data["length"]
        mean_course_line, = ax.plot(
            mean_course_data['x'],
            mean_course_data['y'],
            color='#C04A00',
            linewidth=1.8,
            alpha=0.95,
            label=f'Mean course ({course_len:.1f}m)',
            visible=True
        )

    def _draw_segmented_mean_course():
        """Draw mean course with all segments in orange."""
        nonlocal mean_course_line, mean_course_segments

        # Draw all segments in the same orange color
        for seg in segmentation.segments:
            line, = ax.plot(
                seg.x, seg.y,
                color='#C04A00',
                linewidth=2.5,
                alpha=0.95,
                visible=True)
            mean_course_segments.append(line)

        if not mean_course_segments:
            return

        course_len = mean_course_data["length"]
        mean_course_line, = ax.plot(
            [], [],
            color='#C04A00',
            linewidth=2.5,
            alpha=0.95,
            label=f'Mean course ({course_len:.1f}m)',
            visible=True
        )

    refresh_mean_course()

    # Segment boundary markers (perpendicular ticks on mean course)
    segment_markers = []

    def refresh_segment_markers():
        """Draw short perpendicular markers at segment boundaries."""
        nonlocal segment_markers
        for line in segment_markers:
            line.remove()
        segment_markers = []
        if segmentation is None or mean_course_data is None:
            return
        if segmentation.total_segments == 0:
            return

        headings = mean_course_data['heading']
        marker_len = max(0.3, 0.02 * mean_course_data['length'])
        max_idx = len(mean_course_data['x']) - 1

        # Collect unique boundary indices
        # For closed loops:
        # - Skip index 0 (arbitrary start point)
        # - Skip last index (same physical location as index 0)
        # Only show actual transition points between segments
        boundaries = []
        for seg in segmentation.segments:
            # Skip start_index if it's 0 (arbitrary loop start)
            if seg.start_index != 0:
                boundaries.append((seg.start_index, seg.segment_id))
            # Skip end_index if it's the last point (loop closure)
            if seg.end_index < max_idx:
                boundaries.append((seg.end_index, seg.segment_id))

        seen = set()
        for idx, seg_id in boundaries:
            if idx in seen:
                continue
            seen.add(idx)

            x_val = mean_course_data['x'][idx]
            y_val = mean_course_data['y'][idx]
            heading = headings[idx]
            dx = np.cos(heading + np.pi/2.0) * marker_len * 0.5
            dy = np.sin(heading + np.pi/2.0) * marker_len * 0.5

            line, = ax.plot([x_val - dx, x_val + dx],
                            [y_val - dy, y_val + dy],
                            color='#C04A00',
                            linewidth=2.5,
                            alpha=1.0,
                            visible=True)
            segment_markers.append(line)

    refresh_segment_markers()


    # Helper function to create status text elements
    def _create_status_text(fig, y_pos, text='', color='white'):
        """Create a standardized text element for status display."""
        return fig.text(0.02, y_pos, text, fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='black',
                                  alpha=0.8),
                        color=color)

    # File name text - position in top area above plot
    source_type = "CSV" if is_csv else "Tub"
    expanded_path = os.path.abspath(os.path.expanduser(data_source))
    home_dir = os.path.expanduser('~')
    if expanded_path.startswith(home_dir):
        display_path = expanded_path.replace(home_dir, '~', 1)
    else:
        display_path = expanded_path
    file_text = _create_status_text(
        fig, 0.95, f'{source_type}: {display_path}')

    # Speed text - position in top area above plot
    speed_text = _create_status_text(fig, 0.91)

    # Time text - position in top area above plot
    time_text = _create_status_text(fig, 0.87)

    # Position text - below time text
    pos_text = _create_status_text(fig, 0.83)

    # Lap text - below position text
    lap_text = _create_status_text(fig, 0.79)

    total_dist_text = _create_status_text(fig, 0.75, 'Total dist: --')
    lap_dist_text = _create_status_text(fig, 0.71, 'Lap dist: --')

    # Debug text - below loop text
    debug_text = _create_status_text(fig, 0.67)

    # Drift correction text - below debug text
    drift_text = _create_status_text(fig, 0.63)

    # Controls text - below drift text
    _create_status_text(fig, 0.58,
                        'Controls: \u2190/\u2192 arrows = navigate',
                        color='cyan')

    # Toggle buttons for driven path and mean course
    check_widget = None
    # Create checkbox for toggling driven path and mean course (below segment method)
    rax = plt.axes([0.02, 0.23, 0.09, 0.09])
    rax.set_facecolor('#1a1a1a')

    # Determine which checkboxes to show based on available data
    labels = ['Driven Path']
    visibility = [True]  # Driven path visible by default

    if mean_course_line is not None:
        labels.append('Mean Course')
        visibility.append(mean_course_line.get_visible())

    check_widget = CheckButtons(rax, labels, visibility)

    def toggle_display(label):
        if label == 'Driven Path':
            _toggle_driven_path()
            fig.canvas.draw_idle()
            return

        if label == 'Mean Course':
            _toggle_mean_course()
            fig.canvas.draw_idle()
            return

    def _toggle_driven_path():
        """Toggle driven path visibility."""
        visible = not full_path_scatter.get_visible()
        full_path_scatter.set_visible(visible)

    def _toggle_mean_course():
        """Toggle mean course and segment visibility."""
        if mean_course_line is None:
            return

        visible = not mean_course_line.get_visible()
        mean_course_line.set_visible(visible)

        for line in mean_course_segments:
            line.set_visible(visible)

        for line in segment_markers:
            line.set_visible(visible)

    check_widget.on_clicked(toggle_display)

    # Set axis limits with some padding
    x_margin = (df['x'].max() - df['x'].min()) * 0.1
    y_margin = (df['y'].max() - df['y'].min()) * 0.1
    ax.set_xlim(df['x'].min() - x_margin, df['x'].max() + x_margin)
    ax.set_ylim(df['y'].min() - y_margin, df['y'].max() + y_margin)

    # Add colorbar for speed using actual data range
    norm = plt.Normalize(vmin=speeds_display.min(),
                         vmax=speeds_display.max())
    sm = plt.cm.ScalarMappable(cmap='viridis', norm=norm)
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Speed')

    # Create slider without time display (redundant with top-left display)
    ax_slider = plt.axes([0.15, 0.1, 0.7, 0.03])

    time_slider = Slider(ax_slider, 'Time', df['t'].min(), df['t'].max(),
                         valinit=df['t'].min(), valfmt='')

    # Hide the slider's value text since we show time in top-left corner
    time_slider.valtext.set_visible(False)

    def apply_lap_selection(num_laps):
        nonlocal mean_course_data, segmentation, legend
        num_laps = max(1, min(num_laps, max_laps_detected))
        print(f"\nUpdating to show {num_laps} lap(s)...")

        new_mean_course = compute_mean_course_with_laps(num_laps)
        if new_mean_course is not None:
            mean_course_data = new_mean_course

            try:
                segmentation = CourseSegmentation(
                    mean_course=new_mean_course['mean_course_obj'],
                    params={'boundary_method': segment_method})
                segmentation.compute()
                print("  Updated segmentation for new mean course "
                      f"({segmentation.total_segments} segments) "
                      f"using '{segment_method}' method")
            except Exception as exc:
                print(f"  Could not update segmentation: {exc}")
                segmentation = None

            # Refresh both mean course segments and boundary markers
            refresh_mean_course()
            refresh_segment_markers()

            # Recreate legend with updated mean course line
            if mean_course_line is not None:
                mean_course_line.set_label(
                    f'Mean course ({new_mean_course["length"]:.1f}m)')

                # Rebuild legend handles
                legend_handles = [full_path_legend, current_pos, current_path]
                if mean_course_line is not None:
                    legend_handles.append(mean_course_line)

                # Remove old legend and create new one
                legend.remove()
                legend = ax.legend(
                    handles=legend_handles,
                    bbox_to_anchor=(0.02, 0.54),
                    loc='upper left',
                    framealpha=0.9,
                    ncol=1,
                    fontsize=10,
                    bbox_transform=fig.transFigure)

        if num_laps <= len(lap_end_indices):
            end_idx = lap_end_indices[num_laps - 1]
            df_filtered = df.iloc[:end_idx + 1]
            speeds_filtered = speeds[:end_idx + 1]

            new_offsets = np.c_[df_filtered['x'].values,
                                df_filtered['y'].values]
            full_path_scatter.set_offsets(new_offsets)
            full_path_scatter.set_array(speeds_filtered)

            time_slider.valmax = df_filtered['t'].max()
            time_slider.ax.set_xlim(time_slider.valmin, time_slider.valmax)
            if time_slider.val > time_slider.valmax:
                time_slider.set_val(time_slider.valmin)
        else:
            full_path_scatter.set_offsets(
                np.c_[df['x'].values, df['y'].values])
            full_path_scatter.set_array(speeds)
            time_slider.valmax = df['t'].max()
            time_slider.ax.set_xlim(time_slider.valmin, time_slider.valmax)

        fig.canvas.draw_idle()

    if mean_course_line is not None and max_laps_detected > 1 and lap_end_indices:
        lap_value = [max_laps_detected]

        fig.text(0.12, 0.20, 'Laps', color='white', fontsize=8,
                 bbox=dict(boxstyle='round', facecolor='black', alpha=0.8))

        lap_text_ax = plt.axes([0.12, 0.155, 0.065, 0.035])
        lap_text_ax.set_facecolor('#d3d3d3')
        lap_textbox = TextBox(lap_text_ax, '', initial=str(lap_value[0]))
        lap_textbox.text_disp.set_color('black')

        lap_minus_ax = plt.axes([0.12, 0.11, 0.03, 0.03])
        lap_plus_ax = plt.axes([0.16, 0.11, 0.03, 0.03])
        for ax_button in (lap_minus_ax, lap_plus_ax):
            ax_button.set_facecolor('#1a1a1a')
        lap_minus_button = Button(lap_minus_ax, '-', color='#1a1a1a',
                                  hovercolor='#333333')
        lap_plus_button = Button(lap_plus_ax, '+', color='#1a1a1a',
                                 hovercolor='#333333')

        def update_text_display(val):
            lap_textbox.text_disp.set_text(str(val))

        def set_lap_value(new_val):
            new_val = int(np.clip(new_val, 1, max_laps_detected))
            if new_val == lap_value[0]:
                update_text_display(new_val)
                fig.canvas.draw_idle()
                return
            lap_value[0] = new_val
            update_text_display(new_val)
            apply_lap_selection(new_val)

        def on_text_submit(text):
            try:
                val = int(float(text))
            except ValueError:
                val = lap_value[0]
            set_lap_value(val)

        def on_minus(_event):
            set_lap_value(lap_value[0] - 1)

        def on_plus(_event):
            set_lap_value(lap_value[0] + 1)

        lap_textbox.on_submit(on_text_submit)
        lap_minus_button.on_clicked(on_minus)
        lap_plus_button.on_clicked(on_plus)

    # Segment method selector (if segmentation exists) - between legend and checkboxes
    if mean_course_data is not None:
        current_segment_method = [segment_method]

        fig.text(0.02, 0.38, 'Segment Method', color='white', fontsize=8,
                 bbox=dict(boxstyle='round', facecolor='black', alpha=0.8))

        segment_method_ax = plt.axes([0.02, 0.26, 0.18, 0.11])
        segment_method_ax.set_facecolor('#1a1a1a')

        segment_radio = RadioButtons(
            segment_method_ax,
            ('Threshold', 'Extrema', 'Gradient', 'Hybrid'),
            active={'threshold': 0, 'extrema': 1, 'gradient': 2,
                    'hybrid': 3}.get(segment_method, 2))

        def on_segment_method_change(label):
            nonlocal segmentation, mean_course_data, legend
            method_map = {
                'Threshold': 'threshold',
                'Extrema': 'extrema',
                'Gradient': 'gradient',
                'Hybrid': 'hybrid'
            }
            new_method = method_map[label]
            current_segment_method[0] = new_method

            print(f"\nChanging segmentation method to '{new_method}'...")

            try:
                from donkeycar.parts.course_analysis import CourseSegmentation
                segmentation = CourseSegmentation(
                    mean_course=mean_course_data['mean_course_obj'],
                    params={'boundary_method': new_method})
                segmentation.compute()
                print(f"  Segmented into {segmentation.total_segments} "
                      f"segments using '{new_method}' method")

                # Refresh visualizations
                refresh_mean_course()
                refresh_segment_markers()

                # Update legend
                boundary_handle = None
                if segmentation.total_segments > 0:
                    boundary_handle = Line2D(
                        [0], [0],
                        color='#C04A00',
                        linewidth=2.5,
                        label=f'Segment boundaries ({segmentation.total_segments})')

                legend_handles = [full_path_legend, current_pos, current_path]
                if mean_course_line is not None:
                    legend_handles.append(mean_course_line)
                if boundary_handle is not None:
                    legend_handles.append(boundary_handle)

                legend.remove()
                legend = ax.legend(
                    handles=legend_handles,
                    bbox_to_anchor=(0.02, 0.54),
                    loc='upper left',
                    framealpha=0.9,
                    ncol=1,
                    fontsize=10,
                    bbox_transform=fig.transFigure)

                fig.canvas.draw_idle()

            except Exception as exc:
                print(f"  Could not update segmentation: {exc}")
                import traceback
                traceback.print_exc()

        segment_radio.on_clicked(on_segment_method_change)

    # Performance optimization: throttle updates
    last_update_time = [0]

    def should_throttle_update(now):
        """Check if update should be throttled for performance"""
        if now - last_update_time[0] < 0.1:
            return True
        last_update_time[0] = now
        return False

    def get_display_data(current_data, max_points=1000):
        """Downsample data for display performance"""
        if len(current_data) <= max_points:
            return current_data

        step = len(current_data) // max_points
        return current_data[::step]

    def find_current_loop(current_idx, loop_end_indices):
        """Determine which loop the current index belongs to"""
        if not loop_end_indices:
            return None

        for i, end_idx in enumerate(loop_end_indices):
            if current_idx <= end_idx:
                return i + 1

        return len(loop_end_indices) + 1

    def get_drift_display_text(current_loop):
        """Get drift correction display text for current loop"""
        if not loop_drift_amounts:
            return 'Drift: --'
        if not current_loop:
            return 'Drift: --'
        if current_loop > len(loop_drift_amounts):
            return 'Drift: --'

        drift = loop_drift_amounts[current_loop - 1]
        drift_magnitude = np.sqrt(drift[0]**2 + drift[1]**2)
        return (f'Drift ({current_loop}): '
                f'[{drift[0]:.3f}, {drift[1]:.3f}] '
                f'(mag: {drift_magnitude:.3f}m)')

    def update_displays_with_data(last_point, current_time_val,
                                  current_idx, loop_end_indices):
        """Update all text displays with current data"""
        current_datetime = datetime.fromtimestamp(current_time_val)
        distance_to_origin = np.sqrt(last_point["x"]**2 +
                                     last_point["y"]**2)

        total_distance = cumulative_distance[current_idx]
        total_dist_text.set_text(f'Total dist: {total_distance:.2f}m')

        lap_distance_display = '--'
        speed_text.set_text(f'Speed: {last_point["v"]:.2f}')
        time_text.set_text(
            f'Time: {current_datetime.isoformat(timespec="milliseconds")}')
        pos_text.set_text(
            f'Position: [{last_point["x"]:.3f}, {last_point["y"]:.3f}]')
        debug_text.set_text(
            f'Index: {current_idx} | '
            f'Dist to origin: {distance_to_origin:.3f}m')

        current_loop = find_current_loop(current_idx, loop_end_indices)
        if current_loop is None:
            lap_text.set_text('Lap: --')
            lap_start_idx = 0
        elif current_loop <= len(loop_end_indices):
            lap_text.set_text(f'Lap: {current_loop}')
            lap_start_idx = lap_start_indices[current_loop - 1]
        else:
            lap_text.set_text(f'Lap: After {len(loop_end_indices)}')
            lap_start_idx = lap_start_indices[-1] \
                if lap_start_indices else 0

        if lap_start_indices:
            lap_start_idx = min(lap_start_idx, len(cumulative_distance) - 1)
            lap_distance = total_distance - cumulative_distance[lap_start_idx]
            lap_distance_display = f'{max(lap_distance, 0.0):.2f}m'

        lap_dist_text.set_text(f'Lap dist: {lap_distance_display}')
        drift_text.set_text(get_drift_display_text(current_loop))

    def update_displays_empty(current_time_val):
        """Update displays when no data is available"""
        current_datetime = datetime.fromtimestamp(current_time_val)

        current_pos.set_offsets([[]])
        current_path.set_data([], [])
        speed_text.set_text('Speed: --')
        time_text.set_text(
            f'Time: {current_datetime.isoformat(timespec="milliseconds")}')
        pos_text.set_text('Position: [---, ---]')
        debug_text.set_text('Index: -- | Dist to origin: --m')
        lap_text.set_text('Lap: --')
        total_dist_text.set_text('Total dist: --')
        lap_dist_text.set_text('Lap dist: --')
        drift_text.set_text('Drift: --')

    def update_plot(_val):
        import time as time_module
        current_time_val = time_slider.val

        if should_throttle_update(time_module.time()):
            return

        mask = df['t'] <= current_time_val
        current_data = df[mask]

        if len(current_data) == 0:
            update_displays_empty(current_time_val)
            fig.canvas.draw_idle()
            return

        last_point = current_data.iloc[-1]
        current_pos.set_offsets([[last_point['x'], last_point['y']]])

        display_data = get_display_data(current_data)
        current_path.set_data(display_data['x'], display_data['y'])

        current_idx = len(current_data) - 1
        update_displays_with_data(last_point, current_time_val,
                                  current_idx, loop_end_indices)

        fig.canvas.draw_idle()

    # Connect slider to update function
    time_slider.on_changed(update_plot)

    def on_key_press(event):
        """Handle keyboard navigation - advance exactly one index at a time"""
        current_val = time_slider.val

        # Find current index
        mask = df['t'] <= current_val
        current_idx = len(df[mask]) - 1

        if event.key == 'left':
            # Go to previous index
            new_idx = max(0, current_idx - 1)
            new_val = df['t'].iloc[new_idx]
            # Respect the time slider's min limit
            if new_val >= time_slider.valmin:
                time_slider.set_val(new_val)
        elif event.key == 'right':
            # Go to next index
            new_idx = min(len(df) - 1, current_idx + 1)
            new_val = df['t'].iloc[new_idx]
            # Respect the time slider's max limit (constrained by lap slider)
            if new_val <= time_slider.valmax:
                time_slider.set_val(new_val)

    # Connect keyboard event handler
    fig.canvas.mpl_connect('key_press_event', on_key_press)

    # Initial update
    update_plot(df['t'].min())

    # Add legend in top area underneath controls - vertical arrangement
    # Use custom handles to control legend appearance
    boundary_handle = None
    if segmentation is not None and segmentation.total_segments > 0:
        boundary_handle = Line2D(
            [0], [0],
            color='#C04A00',
            linewidth=2.5,
            label=f'Segment boundaries ({segmentation.total_segments})')

    legend_handles = [full_path_legend, current_pos, current_path]
    if mean_course_line is not None:
        legend_handles.append(mean_course_line)
    if boundary_handle is not None:
        legend_handles.append(boundary_handle)

    legend = ax.legend(handles=legend_handles,
                      bbox_to_anchor=(0.02, 0.54), loc='upper left',
                      framealpha=0.9, ncol=1, fontsize=10,
                      bbox_transform=fig.transFigure)

    plt.show()
