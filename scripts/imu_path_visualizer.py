#!/usr/bin/env python3
"""
IMU Path Visualizer Script

Command-line interface for visualizing IMU path data.
All logic is implemented in donkeycar.utilities.imu_visualization module.
"""

import argparse
from donkeycar.utilities.imu_visualization import visualize_imu_path


def main():
    """Parse command line arguments and call visualization function."""
    parser = argparse.ArgumentParser(
        description='Visualize IMU path data from CSV files or Tub directories',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s imu.csv
  %(prog)s ~/mycar/data/tub_1
  %(prog)s imu.csv --drift-correction
  %(prog)s data/ --downsample-factor 5
        """
    )

    parser.add_argument('data_source', nargs='?', default='imu.csv',
                       help='path to IMU CSV file or Tub directory '
                            '(default: imu.csv)')
    parser.add_argument('--drift-correction', action='store_true',
                       help='enable loop drift correction')
    parser.add_argument('--downsample-factor', type=int, default=None,
                       help='downsample factor for display '
                            '(default: auto-calculate to ~10000 pts)')
    parser.add_argument('--min-loop-distance', type=float, default=1.0,
                       help='minimum distance to travel before considering '
                            'loop closure (meters, default: 1.0)')
    parser.add_argument('--max-distance', type=float, default=0.5,
                       help='maximum distance from origin to start looking '
                            'for reversal point (meters, default: 0.5)')

    args = parser.parse_args()

    # Call the visualization function with parsed arguments
    visualize_imu_path(
        data_source=args.data_source,
        correct_drift=args.drift_correction,
        min_loop_distance=args.min_loop_distance,
        max_distance=args.max_distance,
        downsample_factor=args.downsample_factor
    )


if __name__ == '__main__':
    main()
