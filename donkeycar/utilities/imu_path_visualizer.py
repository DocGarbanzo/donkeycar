#!/usr/bin/env python3
"""
IMU Path Visualization Utilities

This module provides utilities for visualizing and analyzing IMU
path data, including real-time plotting and post-processing
visualization with drift correction.
"""

import multiprocessing
from multiprocessing import Process, Value
import matplotlib.pyplot as plt
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
            if show_timestamps:
                t = data.iloc[idx]['t']
                print(f"    [{i+1}] idx={idx}, t={t:.1f}, "
                      f"pos=[{x:.3f}, {y:.3f}]")
            else:
                print(f"    [{i+1}] idx={idx}, pos=[{x:.3f}, {y:.3f}]")

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

                # If the next average is higher, this is a potential reversal
                # Use smaller threshold for real data (0.1% instead of 2%)
                if next_avg > current_avg * 1.001:  # 0.1% tolerance for gentle increases
                    actual_idx = vicinity_start_idx + i
                    potential_reversals.append(
                        (actual_idx, current_avg, next_avg))

        # Select reversal that occurs soon after entering vicinity (prioritize early + close)
        # This prevents selecting late coincidental approaches to the origin
        if potential_reversals:
            # Filter for reversals that are reasonably close (within 2x of
            # max_distance)
            good_reversals = [
                r for r in potential_reversals if r[1] <= max_distance_from_origin * 2]

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
                        distance_factor = distance / max_distance_from_origin  # 0 = closest, 1+ = farther
                        score = time_factor * 0.7 + distance_factor * 0.3  # Prioritize early time
                        scored_reversals.append((score, reversal))

                if scored_reversals:
                    # Select the reversal with the best (lowest) score
                    best_score, best_reversal = min(scored_reversals)
                    reversal_idx, current_avg, next_avg = best_reversal
                    print(
                        f"  Found {len(potential_reversals)} potential reversals, {len(good_reversals)} within 2x max_distance")
                    print(
                        f"  {len(scored_reversals)} within vicinity window, selected best scoring at index {reversal_idx}")
                    print(
                        f"    Score: {best_score:.3f} (lower=better), Distance: {current_avg:.3f}m")
                    print(
                        f"    Distance increase: {((next_avg/current_avg - 1)*100):.1f}%")
                    return reversal_idx
                else:
                    # No reversals in vicinity window, take earliest good one
                    best_reversal = min(good_reversals, key=lambda x: x[0])
                    reversal_idx, current_avg, next_avg = best_reversal
                    print(
                        f"  No reversals in vicinity window, selected earliest good reversal at index {reversal_idx}")
                    return reversal_idx
            else:
                # Fallback: if no good reversals, take the earliest of all
                best_reversal = min(potential_reversals, key=lambda x: x[0])
                reversal_idx, current_avg, next_avg = best_reversal
                print(
                    f"  Found {len(potential_reversals)} potential reversals, none within 2x max_distance")
                print(f"  Selected EARLIEST reversal at index {reversal_idx}")
                print(f"    Current avg distance: {current_avg:.3f}m")
                return reversal_idx

        # Fallback: if no clear reversal found, use the minimum distance point
        min_distance = min(distances_to_start)
        min_idx = distances_to_start.index(min_distance)
        min_distance_idx = vicinity_start_idx + min_idx
        print(
            f"  No clear reversal found, using minimum distance point at index {min_distance_idx}")
        print(f"    Minimum distance: {min_distance:.3f}m")
        return min_distance_idx

    # Work with a copy
    corrected_df = df.copy()

    print(f"Starting drift correction with {len(df)} total points")
    print(
        f"Parameters: min_loop_distance={min_loop_distance}, max_distance={max_distance}, max_loops={max_loops}")

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
        print(
            f"{loop_num}) LOOP {loop_num} END FOUND: index={loop_end_idx}, timestamp={t:.1f}")

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

        print(
            f"{loop_num+1}) CORRECTION AMOUNT for Loop {loop_num}: [{loop_drift[0]:.3f}, {loop_drift[1]:.3f}]")

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
        print(
            f"    End position: [{corrected_df.iloc[end_idx]['x']:.6f}, {corrected_df.iloc[end_idx]['y']:.6f}]")

    print(
        f"\nFinal result: {len(corrected_df)} points with {num_loops_found} loops corrected")
    print(f"Final data range: "
          f"x=[{corrected_df['x'].min():.3f}, "
          f"{corrected_df['x'].max():.3f}], "
          f"y=[{corrected_df['y'].min():.3f}, "
          f"{corrected_df['y'].max():.3f}]")

    return corrected_df, loop_end_indices, loop_drift_amounts


def visualize_imu_path(csv_file='imu.csv', correct_drift=False,
                       min_loop_distance=1.0, max_distance=0.5,
                       downsample_factor=None):
    """
    Load and visualize IMU path data with interactive time slider.

    Args:
        csv_file (str): Path to CSV file containing t,x,y,z,v columns
        correct_drift (bool): Apply loop drift correction to path data
                              (default: False)
        min_loop_distance (float): Minimum distance to travel before
                                    considering loop closure
        max_distance (float): Maximum distance from origin to start
                              looking for reversal point
        downsample_factor (int): Downsample factor for display
                                 (default: auto-calculate to ~10000 pts)
    """
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    # Check if file exists
    if not os.path.exists(csv_file):
        print(
            f"File {csv_file} not found. Run the IMU with record_path=True first.")
        return

    # Load CSV data
    df = pd.read_csv(csv_file)
    if df.empty:
        print(f"No data found in {csv_file}")
        return

    print(f"Loaded {len(df)} data points from {csv_file}")

    # Apply drift correction if requested
    loop_end_indices = []
    loop_drift_amounts = []
    if correct_drift:
        print("Applying loop drift correction...")
        df, loop_end_indices, loop_drift_amounts = correct_loop_drift(
            df, min_loop_distance, max_distance)
        print("Drift correction completed.")

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
    plt.subplots_adjust(bottom=0.25, right=0.95, top=0.85,
                        left=0.08)  # Room for legend at top

    # Initial plot setup
    ax.set_xlabel('X Position (Forward)')
    ax.set_ylabel('Y Position (Left)')
    ax.set_title('IMU 2D Path Visualization')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Create color map based on speed
    speeds = df['v'].values

    # Plot full trajectory (faded) - downsample for better performance
    if downsample_factor is None:
        downsample_factor = max(1, len(df) // 10000)  # ~10000 points
    df_display = df[::downsample_factor]
    speeds_display = speeds[::downsample_factor]
    ax.scatter(
        df_display['x'],
        df_display['y'],
        c=speeds_display,
        cmap='viridis',
        alpha=0.3,
        s=8,
        label='Full path')

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

    # Helper function to create status text elements
    def _create_status_text(fig, y_pos, text='', color='white'):
        """Create a standardized text element for status display."""
        return fig.text(0.02, y_pos, text, fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='black',
                                  alpha=0.8),
                        color=color)

    # File name text - position in top area above plot
    file_name = os.path.basename(csv_file)
    file_text = _create_status_text(fig, 0.97, f'File: {file_name}')

    # Speed text - position in top area above plot
    speed_text = _create_status_text(fig, 0.89)

    # Time text - position in top area above plot
    time_text = _create_status_text(fig, 0.85)

    # Position text - below time text
    pos_text = _create_status_text(fig, 0.81)

    # Loop text - below position text
    loop_text = _create_status_text(fig, 0.77)

    # Debug text - below loop text
    debug_text = _create_status_text(fig, 0.73)

    # Drift correction text - below debug text
    drift_text = _create_status_text(fig, 0.65)

    # Controls text - below drift text
    _create_status_text(fig, 0.57,
                        'Controls: \u2190/\u2192 arrows = navigate',
                        color='cyan')

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
            loop_text.set_text('Loop: --')
        elif current_loop <= len(loop_end_indices):
            loop_text.set_text(f'Loop: {current_loop}')
        else:
            loop_text.set_text(f'Loop: After {len(loop_end_indices)}')

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
        loop_text.set_text('Loop: --')
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

    # Calculate time step for arrow key navigation
    # Use median time delta between points for single-step navigation
    time_deltas = np.diff(df['t'].values)
    time_step = np.median(time_deltas)

    def on_key_press(event):
        """Handle keyboard navigation"""
        current_val = time_slider.val

        if event.key == 'left':
            new_val = max(df['t'].min(), current_val - time_step)
            time_slider.set_val(new_val)
        elif event.key == 'right':
            new_val = min(df['t'].max(), current_val + time_step)
            time_slider.set_val(new_val)

    # Connect keyboard event handler
    fig.canvas.mpl_connect('key_press_event', on_key_press)

    # Initial update
    update_plot(df['t'].min())

    # Add legend in top area underneath controls - vertical arrangement
    ax.legend(bbox_to_anchor=(0.02, 0.65), loc='upper left',
              framealpha=0.9, ncol=1, fontsize=10,
              bbox_transform=fig.transFigure)

    plt.show()
