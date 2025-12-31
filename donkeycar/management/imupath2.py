"""
IMU Path Visualization using NEW course_analysis API

Interactive visualization with full UI features (Phase 7b complete).
Features time slider, lap selector, segment method selector, and real-time
navigation.

Command: donkey imupath2 <data_source>

Example:
    donkey imupath2 ./recording.csv
    donkey imupath2 ./tub_directory
    donkey imupath2 --lap-method drift --segment-method hybrid ./data.csv
"""

import argparse
import os

# Import NEW course_analysis API
from donkeycar.course_analysis import (
    CSVPathDataSource,
    TubPathDataSource,
)

# Import interactive visualizer
from donkeycar.utilities.interactive_imu_viz import InteractiveIMUVisualizer


class ImuPath2Command:
    """Command for testing new course_analysis API with visualization"""

    def parse_args(self, args):
        parser = argparse.ArgumentParser(
            prog='imupath2',
            usage='%(prog)s [options] [data_source]',
            description='Visualize IMU path using NEW course_analysis API')

        parser.add_argument('data_source', nargs='?', default='imu.csv',
                           help='path to CSV file or Tub directory')

        parser.add_argument('--lap-method', type=str,
                           choices=['y_crossing', 'drift'],
                           default='y_crossing',
                           help='lap detection method (default: y_crossing)')

        parser.add_argument('--segment-method', type=str,
                           choices=['threshold', 'extrema', 'gradient', 'hybrid'],
                           default='gradient',
                           help='segmentation method (default: gradient)')

        parser.add_argument('--min-loop-distance', type=float, default=1.0,
                           help='minimum loop distance in meters (default: 1.0)')

        parser.add_argument('--num-laps', type=int, default=None,
                           help='number of laps to use for mean course (default: all)')

        parsed_args = parser.parse_args(args)
        return parsed_args

    def run(self, args):
        args = self.parse_args(args)

        # Expand path
        data_source = os.path.expanduser(args.data_source)

        # Check if source exists
        if not os.path.exists(data_source):
            print(f"Error: File or directory {data_source} not found.")
            return

        print("=" * 70)
        print("IMU Path Analysis - Interactive Visualization (Phase 7b)")
        print("=" * 70)
        print(f"Data source: {data_source}")
        print(f"Lap detection: {args.lap_method}")
        print(f"Segmentation: {args.segment_method}")
        print("=" * 70)

        # Load data
        print("\nLoading data...")
        try:
            if os.path.isfile(data_source) and data_source.endswith('.csv'):
                source = CSVPathDataSource(data_source)
                print(f"  ✓ Loaded CSV file")
            elif os.path.isdir(data_source):
                source = TubPathDataSource(data_source)
                print(f"  ✓ Loaded Tub directory")
            else:
                print("  ✗ Source must be CSV file or Tub directory")
                return

            path_data = source.load()
            print(f"  ✓ {len(path_data.timestamp)} data points")
            print(f"  ✓ Total distance: {path_data.total_distance:.2f}m")
            print(f"  ✓ Duration: {path_data.duration:.2f}s")
        except Exception as e:
            print(f"  ✗ Error loading data: {e}")
            import traceback
            traceback.print_exc()
            return

        # Create interactive visualizer
        # (It handles all data processing internally)
        print("\nInitializing interactive visualization...")
        print("  • Detecting laps...")
        print("  • Building mean course...")
        print("  • Segmenting course...")
        print("  • Creating interactive UI...")

        try:
            viz = InteractiveIMUVisualizer(
                path_data=path_data,
                cfg=None,  # Uses default parameters
                lap_method=args.lap_method,
                segment_method=args.segment_method,
                file_path=data_source
            )

            # Set up UI and show
            viz.setup_ui()

            print("\n" + "=" * 70)
            print("Interactive visualization ready!")
            print("=" * 70)
            print("Controls:")
            print("  • Time slider: Navigate through recorded path")
            print("  • Lap selector: Change number of laps for mean course")
            print("  • Segment method: Switch segmentation algorithm")
            print("  • Display toggles: Show/hide driven path and mean course")
            print("  • Keyboard: ← → arrows for frame-by-frame navigation")
            print("=" * 70)

            viz.show()

        except Exception as e:
            print(f"\n✗ Error creating visualization: {e}")
            import traceback
            traceback.print_exc()
            return


# Make command available
def execute(args):
    """Entry point for donkey imupath2 command"""
    cmd = ImuPath2Command()
    cmd.run(args)
