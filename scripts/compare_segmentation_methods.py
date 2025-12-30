#!/usr/bin/env python3
"""
Compare different segmentation methods

This script tests all available segmentation methods (threshold, extrema,
gradient, hybrid) and creates comparison visualizations.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from donkeycar.course_analysis.old.course_analysis import (
    MultiLapData, MeanCourse, CourseSegmentation)


def compare_segmentation_methods(tub_path, output_path="~/segmentation_comparison.png"):
    """
    Compare all segmentation methods on the same course data

    Args:
        tub_path: Path to tub directory with IMU data
        output_path: Where to save comparison plot
    """
    tub_path = os.path.expanduser(tub_path)
    output_path = os.path.expanduser(output_path)

    print(f"Loading data from: {tub_path}")

    # Load and process data
    multilap_data = MultiLapData()
    multilap_data.load_data(
        tub_path,
        lap_detection_method='y_crossing',
        min_loop_distance=1.0,
        y_threshold=0.1,
        min_lap_length=50
    )

    print(f"Detected {multilap_data.num_laps} laps")

    # Compute mean course
    mean_course = MeanCourse(multilap_data)
    mean_course.compute()

    print(f"Course length: {mean_course.distance[-1]:.2f}m")

    # Test each method
    methods = ['threshold', 'extrema', 'gradient', 'hybrid']
    segmentations = {}

    for method in methods:
        print(f"\nTesting {method} method...")
        seg = CourseSegmentation(mean_course, params={'boundary_method': method})
        seg.compute(use_adaptive_threshold=True)
        segmentations[method] = seg
        print(f"  Found {seg.total_segments} segments")
        for s in seg.segments:
            print(f"    {s.segment_id}: {s.segment_type.value} ({s.length:.2f}m)")

    # Create comparison plot
    fig, axes = plt.subplots(len(methods), 1, figsize=(14, 4 * len(methods)), sharex=True)

    distance = mean_course.distance

    for idx, method in enumerate(methods):
        ax = axes[idx]
        seg = segmentations[method]

        # Recompute curvature for plotting
        curvature = seg._calculate_curvature()

        # Plot curvature
        ax.plot(distance, curvature, 'b-', linewidth=1, label='Curvature')
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)

        # Show threshold
        threshold = seg.params['straight_curvature_threshold']
        ax.axhline(y=threshold, color='red', linestyle='--', alpha=0.7,
                  label=f'Threshold: ±{threshold:.3f}')
        ax.axhline(y=-threshold, color='red', linestyle='--', alpha=0.7)

        # Mark segment boundaries
        for s in seg.segments:
            ax.axvline(x=distance[s.start_index], color='orange',
                      linestyle=':', alpha=0.7, linewidth=2)

        # Color segments by type
        segment_colors = {
            'straight': 'green',
            'left_turn': 'blue',
            'right_turn': 'red',
        }

        for s in seg.segments:
            seg_dist = distance[s.start_index:s.end_index + 1]
            seg_curv = curvature[s.start_index:s.end_index + 1]
            color = segment_colors.get(s.segment_type.value, 'purple')
            ax.fill_between(seg_dist, seg_curv, 0, alpha=0.2, color=color)

        ax.set_ylabel('Curvature (rad/m)')
        ax.set_title(f'{method.upper()} Method ({seg.total_segments} segments)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Distance along course (m)')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nSaved comparison plot to: {output_path}")

    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for method in methods:
        seg = segmentations[method]
        print(f"{method.upper():12s}: {seg.total_segments} segments")
    print("="*60)


def main():
    if len(sys.argv) < 2:
        print("Usage: python compare_segmentation_methods.py <tub_path>")
        print("Example: python compare_segmentation_methods.py ~/cars/hyper/data")
        return

    tub_path = sys.argv[1]
    compare_segmentation_methods(tub_path)


if __name__ == '__main__':
    main()
