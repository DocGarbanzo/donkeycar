"""
IMU Path Visualization

Interactive visualization for recorded vehicle paths.
Features time slider, lap selector, segment method selector, and real-time
navigation.

Command: donkey imupath <data_source>

Example:
    donkey imupath ./recording.csv
    donkey imupath ./tub_directory
    donkey imupath --lap-method drift --segment-method hybrid ./data.csv
"""

import argparse
import os

from donkeycar.course_analysis import (
    CSVPathDataSource,
    TubPathDataSource,
)
from donkeycar.utilities.interactive_imu_viz import InteractiveIMUVisualizer


class ImuPathCommand:
    """Command for visualizing IMU path data."""

    def parse_args(self, args):
        parser = argparse.ArgumentParser(
            prog='imupath',
            usage='%(prog)s [options] [data_source]',
            description='Visualize IMU path data')

        parser.add_argument('data_source', nargs='?', default='imu.csv',
                           help='path to CSV file or Tub directory')
        parser.add_argument('--lap-method', type=str,
                           choices=['y_crossing', 'drift'],
                           default='y_crossing',
                           help='lap detection method (default: y_crossing)')
        parser.add_argument('--segment-method', type=str,
                           choices=['threshold', 'extrema', 'gradient',
                                    'hybrid'],
                           default='gradient',
                           help='segmentation method (default: gradient)')
        parser.add_argument('--min-loop-distance', type=float, default=1.0,
                           help='minimum loop distance in meters')
        parser.add_argument('--num-laps', type=int, default=None,
                           help='number of laps for mean course (default: all)')

        return parser.parse_args(args)

    def run(self, args):
        args = self.parse_args(args)
        data_source = os.path.expanduser(args.data_source)

        if not os.path.exists(data_source):
            print(f"Error: File or directory {data_source} not found.")
            return

        print("=" * 70)
        print("IMU Path Analysis")
        print("=" * 70)
        print(f"Data source: {data_source}")
        print(f"Lap detection: {args.lap_method}")
        print(f"Segmentation: {args.segment_method}")
        print("=" * 70)

        print("\nLoading data...")
        try:
            if os.path.isfile(data_source) and data_source.endswith('.csv'):
                source = CSVPathDataSource(data_source)
            elif os.path.isdir(data_source):
                source = TubPathDataSource(data_source)
            else:
                print("Error: Source must be CSV file or Tub directory")
                return

            path_data = source.load()
            print(f"  {len(path_data.timestamp)} data points")
            print(f"  Total distance: {path_data.total_distance:.2f}m")
            print(f"  Duration: {path_data.duration:.2f}s")
        except Exception as e:
            print(f"Error loading data: {e}")
            import traceback
            traceback.print_exc()
            return

        print("\nInitializing visualization...")
        try:
            viz = InteractiveIMUVisualizer(
                path_data=path_data,
                cfg=None,
                lap_method=args.lap_method,
                segment_method=args.segment_method,
                file_path=data_source
            )
            viz.setup_ui()

            print("\n" + "=" * 70)
            print("Controls:")
            print("  Time slider: Navigate through recorded path")
            print("  Lap selector: Change number of laps for mean course")
            print("  Segment method: Switch segmentation algorithm")
            print("  Display toggles: Show/hide driven path and mean course")
            print("  Keyboard: left/right arrows for frame-by-frame navigation")
            print("=" * 70)

            viz.show()

        except Exception as e:
            print(f"Error creating visualization: {e}")
            import traceback
            traceback.print_exc()
