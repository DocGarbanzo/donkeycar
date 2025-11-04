#!/usr/bin/env python3
"""
Course Analyzer Command-Line Tool

Analyzes multi-lap course data to create mean courses and segmentation.

Usage:
    python analyzer.py <input_csv> [options]

Examples:
    # Basic analysis
    python analyzer.py laps.csv

    # Save outputs
    python analyzer.py laps.csv --output-dir ./output

    # Adjust parameters
    python analyzer.py laps.csv --lap-threshold 3.0 --curvature-threshold 0.1

Author: DonkeyCar Community
Date: 2025
"""

import argparse
import sys
import os
import json
import numpy as np
from pathlib import Path

# Add parent directory to path to import donkeycar modules
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from donkeycar.parts.course_analysis import (
    MultiLapData, MeanCourse, CourseSegmentation, SegmentEstimator, SegmentType
)


def main():
    parser = argparse.ArgumentParser(
        description='Analyze multi-lap course data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s laps.csv
  %(prog)s laps.csv --output-dir ./output
  %(prog)s laps.csv --lap-threshold 3.0 --resampling 0.2
        """
    )

    # Input/output
    parser.add_argument('input_csv', help='Input CSV file with multi-lap data')
    parser.add_argument('--output-dir', '-o', default='./course_output',
                        help='Output directory for results (default: ./course_output)')

    # Lap detection parameters
    parser.add_argument('--lap-threshold', type=float, default=2.0,
                        help='Distance threshold for lap detection (meters, default: 2.0)')
    parser.add_argument('--min-lap-points', type=int, default=50,
                        help='Minimum points per lap (default: 50)')

    # Mean course parameters
    parser.add_argument('--resampling', type=float, default=0.1,
                        help='Resampling interval (meters, default: 0.1)')
    parser.add_argument('--outlier-threshold', type=float, default=2.5,
                        help='Outlier detection threshold (std dev, default: 2.5)')
    parser.add_argument('--smoothing-window', type=int, default=11,
                        help='Position smoothing window (points, default: 11)')

    # Segmentation parameters
    parser.add_argument('--curvature-threshold', type=float, default=0.05,
                        help='Straight segment curvature threshold (rad/m, default: 0.05)')
    parser.add_argument('--min-segment-length', type=float, default=0.5,
                        help='Minimum segment length (meters, default: 0.5)')

    # Output options
    parser.add_argument('--no-mean-course', action='store_true',
                        help='Skip saving mean course')
    parser.add_argument('--no-segmentation', action='store_true',
                        help='Skip segmentation')
    parser.add_argument('--format', choices=['json', 'csv', 'both'], default='both',
                        help='Output format (default: both)')

    args = parser.parse_args()

    # Check input file exists
    if not os.path.exists(args.input_csv):
        print(f"Error: Input file '{args.input_csv}' not found")
        return 1

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 70)
    print("DonkeyCar Course Analyzer")
    print("=" * 70)
    print(f"Input file: {args.input_csv}")
    print(f"Output directory: {args.output_dir}")
    print()

    # Step 1: Load multi-lap data
    print("Step 1: Loading multi-lap data...")
    print("-" * 70)

    multilap_data = MultiLapData()
    try:
        multilap_data.load_csv(
            args.input_csv,
            lap_detection_threshold=args.lap_threshold,
            min_lap_length=args.min_lap_points
        )
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return 1

    print(f"  Loaded {len(multilap_data.raw_data)} data points")
    print(f"  Detected {multilap_data.num_laps} laps")

    if multilap_data.num_laps == 0:
        print("Error: No laps detected. Try adjusting --lap-threshold")
        return 1

    # Print lap statistics
    laps = multilap_data.get_laps()
    lap_lengths = [len(lap) for lap in laps]
    print(f"  Lap points: min={min(lap_lengths)}, max={max(lap_lengths)}, mean={np.mean(lap_lengths):.0f}")
    print()

    # Step 2: Compute mean course
    print("Step 2: Computing mean course...")
    print("-" * 70)

    mean_params = {
        'resampling_interval': args.resampling,
        'outlier_std_threshold': args.outlier_threshold,
        'position_smoothing_window': args.smoothing_window,
    }

    mean_course = MeanCourse(multilap_data, mean_params)
    try:
        mean_course.compute()
    except Exception as e:
        print(f"Error computing mean course: {e}")
        return 1

    print(f"  Course length: {mean_course.distance[-1]:.2f} meters")
    print(f"  Course points: {len(mean_course.x)}")

    # Print quality metrics
    if 'x_std' in mean_course.metadata:
        x_std = mean_course.metadata['x_std']
        y_std = mean_course.metadata['y_std']
        pos_std = np.sqrt(x_std**2 + y_std**2)
        print(f"  Position std dev: mean={np.mean(pos_std):.3f}m, max={np.max(pos_std):.3f}m")

    if 'heading_std' in mean_course.metadata:
        h_std = mean_course.metadata['heading_std']
        print(f"  Heading std dev: mean={np.mean(h_std):.2f}°, max={np.max(h_std):.2f}°")

    # Save mean course
    if not args.no_mean_course:
        if args.format in ['json', 'both']:
            mean_json = os.path.join(args.output_dir, 'mean_course.json')
            mean_course.save(mean_json)
            print(f"  Saved: {mean_json}")

        if args.format in ['csv', 'both']:
            mean_csv = os.path.join(args.output_dir, 'mean_course.csv')
            mean_course.save(mean_csv)
            print(f"  Saved: {mean_csv}")
    print()

    # Step 3: Segment course
    if not args.no_segmentation:
        print("Step 3: Segmenting course...")
        print("-" * 70)

        seg_params = {
            'straight_curvature_threshold': args.curvature_threshold,
            'min_segment_length': args.min_segment_length,
        }

        segmentation = CourseSegmentation(mean_course, seg_params)
        try:
            segmentation.compute()
        except Exception as e:
            print(f"Error computing segmentation: {e}")
            return 1

        print(f"  Total segments: {segmentation.total_segments}")
        print()
        print("  Segment counts:")
        for seg_type, count in segmentation.segment_counts.items():
            print(f"    {seg_type.value:15s}: {count:3d}")

        # Print segment details
        print()
        print("  Segment details:")
        print(f"  {'ID':<4} {'Type':<15} {'Length (m)':<12} {'Mean κ':<10} {'Heading Δ':<10}")
        print("  " + "-" * 65)

        for segment in segmentation.segments:
            print(f"  {segment.segment_id:<4} "
                  f"{segment.segment_type.value:<15} "
                  f"{segment.length:<12.2f} "
                  f"{segment.mean_curvature:<10.4f} "
                  f"{segment.total_heading_change:<10.2f}°")

        # Save segmentation
        seg_json = os.path.join(args.output_dir, 'segmentation.json')
        segmentation.save(seg_json)
        print()
        print(f"  Saved: {seg_json}")
        print()

        # Step 4: Create segment estimator
        print("Step 4: Creating segment estimator...")
        print("-" * 70)

        estimator = SegmentEstimator(segmentation)
        print("  Estimator ready for real-time segment detection")

        # Test estimator at a few points
        print()
        print("  Testing estimator at sample points:")
        test_indices = [0, len(mean_course.x) // 4, len(mean_course.x) // 2]

        for idx in test_indices:
            x = mean_course.x[idx]
            y = mean_course.y[idx]
            heading = mean_course.heading[idx]

            estimate = estimator.estimate(x, y, heading)

            print(f"    Point {idx}: segment={estimate.segment_id}, "
                  f"confidence={estimate.confidence:.2f}, "
                  f"distance={estimate.distance_to_course:.3f}m")

        print()

    # Summary
    print("=" * 70)
    print("Analysis complete!")
    print("=" * 70)
    print(f"Results saved to: {args.output_dir}")
    print()

    return 0


if __name__ == '__main__':
    sys.exit(main())
