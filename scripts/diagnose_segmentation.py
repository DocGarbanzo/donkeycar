#!/usr/bin/env python3
"""
Diagnostic script for course segmentation analysis

This script analyzes curvature and segmentation on a course to help
debug segmentation issues.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from donkeycar.course_analysis.old.course_analysis import MultiLapData, MeanCourse, CourseSegmentation


def plot_curvature_analysis(mean_course, segmentation):
    """
    Create diagnostic plots showing curvature and segmentation

    Args:
        mean_course: MeanCourse object
        segmentation: CourseSegmentation object
    """
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # Manually compute curvature using same method as segmentation
    curvature = segmentation._calculate_curvature()
    distance = mean_course.distance

    # Plot 1: Curvature vs distance
    ax1 = axes[0]
    ax1.plot(distance, curvature, 'b-', linewidth=1, label='Curvature')
    ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.5)

    # Show threshold lines
    threshold = segmentation.params['straight_curvature_threshold']
    ax1.axhline(y=threshold, color='red', linestyle='--',
                alpha=0.7, label=f'Straight threshold: ±{threshold:.3f}')
    ax1.axhline(y=-threshold, color='red', linestyle='--', alpha=0.7)

    # Mark segment boundaries
    for seg in segmentation.segments:
        ax1.axvline(x=distance[seg.start_index], color='orange',
                   linestyle=':', alpha=0.7, linewidth=2)

    ax1.set_ylabel('Curvature (rad/m)')
    ax1.set_title('Curvature Analysis')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Segment classification
    ax2 = axes[1]

    # Color code segments by type
    colors = {
        'straight': 'green',
        'left_turn': 'blue',
        'right_turn': 'red',
        's_curve_lr': 'purple',
        's_curve_rl': 'purple',
        'chicane': 'orange'
    }

    for seg in segmentation.segments:
        seg_dist = distance[seg.start_index:seg.end_index + 1]
        seg_curv = curvature[seg.start_index:seg.end_index + 1]
        color = colors.get(seg.segment_type.value, 'gray')
        label = f'{seg.segment_type.value} ({seg.length:.1f}m)'
        ax2.plot(seg_dist, seg_curv, color=color, linewidth=3,
                alpha=0.7, label=label)

    ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax2.axhline(y=threshold, color='red', linestyle='--', alpha=0.3)
    ax2.axhline(y=-threshold, color='red', linestyle='--', alpha=0.3)
    ax2.set_ylabel('Curvature (rad/m)')
    ax2.set_title('Segment Classification')
    ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Point type classification
    ax3 = axes[2]

    # Recreate point classification
    point_types = segmentation._classify_point_types(
        curvature, threshold)

    # Map to colors
    type_colors = []
    for pt in point_types:
        if pt == 0:
            type_colors.append('green')  # straight
        elif pt == 1:
            type_colors.append('blue')   # left
        else:
            type_colors.append('red')    # right

    ax3.scatter(distance, point_types, c=type_colors, s=2, alpha=0.6)
    ax3.set_ylabel('Point Type\n(-1=right, 0=straight, 1=left)')
    ax3.set_xlabel('Distance along course (m)')
    ax3.set_title('Point-by-Point Classification')
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(-1.5, 1.5)

    plt.tight_layout()
    return fig


def main():
    if len(sys.argv) < 2:
        print("Usage: python diagnose_segmentation.py <tub_path>")
        print("Example: python diagnose_segmentation.py ~/cars/hyper/data")
        return

    tub_path = os.path.expanduser(sys.argv[1])

    if not os.path.exists(tub_path):
        print(f"Error: Path not found: {tub_path}")
        return

    print(f"Analyzing segmentation for: {tub_path}")
    print()

    # Load and process data
    print("Loading multi-lap data...")
    multilap_data = MultiLapData()
    multilap_data.load_data(
        tub_path,
        lap_detection_method='y_crossing',
        min_loop_distance=1.0,
        y_threshold=0.1,
        min_lap_length=50
    )

    print(f"  Detected {multilap_data.num_laps} laps")
    print()

    # Compute mean course
    print("Computing mean course...")
    mean_course = MeanCourse(multilap_data)
    mean_course.compute()
    print(f"  Course length: {mean_course.distance[-1]:.2f}m")
    print()

    # Segment the course
    print("Segmenting course...")
    segmentation = CourseSegmentation(mean_course)
    print(f"  Using parameters:")
    for key, val in segmentation.params.items():
        print(f"    {key}: {val}")
    print()

    segmentation.compute()
    print(f"  Found {segmentation.total_segments} segments:")
    for seg in segmentation.segments:
        print(f"    Segment {seg.segment_id}: {seg.segment_type.value} "
              f"({seg.length:.2f}m, mean_curv={seg.mean_curvature:.3f})")
    print()

    # Analyze curvature statistics
    print("Curvature statistics:")
    curvature = segmentation._calculate_curvature()
    print(f"  Min curvature: {np.min(curvature):.4f} rad/m")
    print(f"  Max curvature: {np.max(curvature):.4f} rad/m")
    print(f"  Mean abs curvature: {np.mean(np.abs(curvature)):.4f} rad/m")
    print(f"  Std curvature: {np.std(curvature):.4f} rad/m")
    print()

    # Create diagnostic plots
    print("Creating diagnostic plots...")
    fig = plot_curvature_analysis(mean_course, segmentation)

    # Save plot instead of showing it
    output_path = os.path.expanduser("~/curvature_diagnostic.png")
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved diagnostic plot to: {output_path}")
    print()
    print("Done!")


if __name__ == '__main__':
    main()
