"""
IMU Path Visualization using NEW course_analysis API

This command demonstrates the new refactored course_analysis API (Phases 1-6).
Use 'donkey imupath' for the original implementation.

Command: donkey imupath2 <data_source>

Example:
    donkey imupath2 ./recording.csv
    donkey imupath2 ./tub_directory
"""

import argparse
import os
import matplotlib.pyplot as plt
import numpy as np

# Import NEW course_analysis API
from donkeycar.course_analysis import (
    CSVPathDataSource,
    TubPathDataSource,
    YCrossingLapDetector,
    DriftLapDetector,
    MultiLapData,
    MeanCourseBuilder,
    GradientSegmentation,
    ThresholdSegmentation,
    ExtremaSegmentation,
    HybridSegmentation,
    CourseSegmenter,
    SegmentAssigner,
    SegmentType
)


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
        print("IMU Path Analysis - NEW API (Phases 1-6)")
        print("=" * 70)
        print(f"Data source: {data_source}")
        print(f"Lap detection: {args.lap_method}")
        print(f"Segmentation: {args.segment_method}")
        print("=" * 70)

        # Step 1: Load data
        print("\n[1/5] Loading data...")
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

        # Step 2: Detect laps
        print("\n[2/5] Detecting laps...")
        try:
            if args.lap_method == 'y_crossing':
                detector = YCrossingLapDetector(
                    params={
                        'min_loop_distance': args.min_loop_distance,
                        'y_threshold': 0.1,
                        'min_lap_length': 50
                    }
                )
            else:  # drift
                detector = DriftLapDetector(
                    params={'min_loop_distance': args.min_loop_distance}
                )

            multilap_data = MultiLapData.from_source(source, detector)
            print(f"  ✓ Detected {multilap_data.num_laps} laps")

            for i, boundary in enumerate(multilap_data.lap_boundaries):
                print(f"    Lap {i+1}: {boundary.num_points} points, "
                      f"{boundary.duration:.2f}s")

            if multilap_data.num_laps == 0:
                print("  ⚠ No laps detected - cannot compute mean course")
                self._visualize_path_only(path_data)
                return
        except Exception as e:
            print(f"  ✗ Error detecting laps: {e}")
            import traceback
            traceback.print_exc()
            return

        # Limit laps if requested
        if args.num_laps is not None:
            num_laps = min(args.num_laps, multilap_data.num_laps)
            if num_laps < multilap_data.num_laps:
                print(f"  → Limiting to {num_laps} laps")
                limited_boundaries = multilap_data.lap_boundaries[:num_laps]
                multilap_data = MultiLapData(
                    multilap_data.path_data, limited_boundaries)

        # Step 3: Build mean course
        print("\n[3/5] Building mean course...")
        try:
            builder = MeanCourseBuilder()
            mean_course = builder.build(multilap_data)
            print(f"  ✓ Mean course length: {mean_course.length:.2f}m")
            print(f"  ✓ Resampled to {len(mean_course.x)} points")
        except Exception as e:
            print(f"  ✗ Error building mean course: {e}")
            import traceback
            traceback.print_exc()
            return

        # Step 4: Segment mean course
        print(f"\n[4/5] Segmenting course ({args.segment_method})...")
        try:
            strategy_map = {
                'threshold': ThresholdSegmentation(),
                'extrema': ExtremaSegmentation(),
                'gradient': GradientSegmentation(),
                'hybrid': HybridSegmentation()
            }
            strategy = strategy_map[args.segment_method]

            segmenter = CourseSegmenter(strategy)
            segmentation = segmenter.segment(mean_course)

            print(f"  ✓ Detected {segmentation.num_segments} segments")

            # Count segment types
            type_counts = {}
            for seg in segmentation.segments:
                seg_type = seg.segment_type.name
                type_counts[seg_type] = type_counts.get(seg_type, 0) + 1

            for seg_type, count in sorted(type_counts.items()):
                print(f"    {seg_type}: {count}")
        except Exception as e:
            print(f"  ✗ Error segmenting course: {e}")
            import traceback
            traceback.print_exc()
            return

        # Step 5: Assign segments to path
        print("\n[5/5] Assigning segments to driven path...")
        try:
            assigner = SegmentAssigner(segmentation)
            segment_ids = assigner.assign(path_data.x, path_data.y)

            unique_segments = len(set(segment_ids))
            print(f"  ✓ Assigned {len(segment_ids)} points")
            print(f"  ✓ Path traversed {unique_segments} unique segments")

            # Show per-lap segment distribution
            for i, boundary in enumerate(multilap_data.lap_boundaries):
                lap_segs = set(segment_ids[
                    boundary.start_index:boundary.end_index + 1])
                print(f"    Lap {i+1}: {len(lap_segs)} segments")
        except Exception as e:
            print(f"  ✗ Error assigning segments: {e}")
            import traceback
            traceback.print_exc()
            return

        # Visualize results
        print("\n" + "=" * 70)
        print("Opening visualization...")
        print("=" * 70)
        self._visualize_results(
            path_data, multilap_data, mean_course,
            segmentation, segment_ids, args)

    def _visualize_path_only(self, path_data):
        """Visualize just the driven path (no laps detected)"""
        fig, ax = plt.subplots(figsize=(12, 10))

        # Plot path with velocity coloring
        scatter = ax.scatter(
            path_data.x, path_data.y,
            c=path_data.velocity, cmap='viridis', s=10, alpha=0.6)

        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Driven Path (No Laps Detected)')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        plt.colorbar(scatter, ax=ax, label='Velocity (m/s)')
        plt.tight_layout()
        plt.show()

    def _visualize_results(self, path_data, multilap_data, mean_course,
                          segmentation, segment_ids, args):
        """Create comprehensive visualization of results"""

        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)

        # Define colors for segment types
        segment_colors = {
            SegmentType.STRAIGHT: '#888888',
            SegmentType.LEFT_TURN: '#3498db',
            SegmentType.RIGHT_TURN: '#e74c3c',
            SegmentType.S_CURVE_LR: '#9b59b6',
            SegmentType.S_CURVE_RL: '#f39c12',
            SegmentType.CHICANE: '#1abc9c'
        }

        # Plot 1: Driven path with velocity
        ax1 = fig.add_subplot(gs[0, 0])
        scatter1 = ax1.scatter(
            path_data.x, path_data.y,
            c=path_data.velocity, cmap='viridis', s=5, alpha=0.6)

        # Mark lap boundaries
        for i, boundary in enumerate(multilap_data.lap_boundaries):
            idx = boundary.end_index
            ax1.plot(path_data.x[idx], path_data.y[idx], 'r*',
                    markersize=15, label=f'Lap {i+1} end' if i == 0 else '')

        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title(f'Driven Path - {multilap_data.num_laps} Laps Detected')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        if multilap_data.num_laps > 0:
            ax1.legend()
        plt.colorbar(scatter1, ax=ax1, label='Velocity (m/s)')

        # Plot 2: Mean course with segments
        ax2 = fig.add_subplot(gs[0, 1])

        # Plot each segment with its type color
        for seg in segmentation.segments:
            start_idx = seg.start_index
            end_idx = seg.end_index + 1
            color = segment_colors.get(seg.segment_type, '#000000')
            ax2.plot(
                mean_course.x[start_idx:end_idx],
                mean_course.y[start_idx:end_idx],
                color=color, linewidth=3, alpha=0.7,
                label=seg.segment_type.name if seg.segment_id == 0
                      or seg.segment_type.name not in
                      [s.segment_type.name for s in segmentation.segments[:seg.segment_id]]
                      else '')

        ax2.set_xlabel('X Position (m)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title(f'Mean Course - {segmentation.num_segments} Segments '
                     f'({args.segment_method})')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        ax2.legend(loc='best')

        # Plot 3: Driven path with segment coloring
        ax3 = fig.add_subplot(gs[1, 0])

        # Create segment color array for path
        path_colors = np.zeros((len(segment_ids), 3))
        for i, seg_id in enumerate(segment_ids):
            if 0 <= seg_id < segmentation.num_segments:
                seg = segmentation.segments[seg_id]
                color_hex = segment_colors.get(seg.segment_type, '#888888')
                # Convert hex to RGB
                color_rgb = tuple(int(color_hex[i:i+2], 16)/255.0
                                for i in (1, 3, 5))
                path_colors[i] = color_rgb

        ax3.scatter(path_data.x, path_data.y, c=path_colors, s=5, alpha=0.6)
        ax3.set_xlabel('X Position (m)')
        ax3.set_ylabel('Y Position (m)')
        ax3.set_title('Driven Path - Segment Assignment')
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')

        # Plot 4: Statistics
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.axis('off')

        stats_text = f"""
NEW COURSE ANALYSIS API - RESULTS

Data Source: {os.path.basename(args.data_source)}
═══════════════════════════════════════

PATH DATA:
  • Total points: {len(path_data.timestamp)}
  • Total distance: {path_data.total_distance:.2f} m
  • Duration: {path_data.duration:.2f} s
  • Mean velocity: {path_data.mean_velocity:.2f} m/s

LAP DETECTION ({args.lap_method}):
  • Laps detected: {multilap_data.num_laps}
  • Min loop distance: {args.min_loop_distance} m

MEAN COURSE:
  • Course length: {mean_course.length:.2f} m
  • Resampled points: {len(mean_course.x)}

SEGMENTATION ({args.segment_method}):
  • Total segments: {segmentation.num_segments}
"""

        # Add segment type breakdown
        type_counts = {}
        for seg in segmentation.segments:
            seg_type = seg.segment_type.name
            type_counts[seg_type] = type_counts.get(seg_type, 0) + 1

        stats_text += "\nSEGMENT TYPES:\n"
        for seg_type, count in sorted(type_counts.items()):
            stats_text += f"  • {seg_type}: {count}\n"

        ax4.text(0.1, 0.5, stats_text, transform=ax4.transAxes,
                fontfamily='monospace', fontsize=10,
                verticalalignment='center')

        plt.suptitle('IMU Path Analysis - NEW API', fontsize=14, fontweight='bold')
        plt.show()


# Make command available
def execute(args):
    """Entry point for donkey imupath2 command"""
    cmd = ImuPath2Command()
    cmd.run(args)
