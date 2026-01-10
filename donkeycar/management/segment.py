"""
Segment Assignment Command

Computes segment assignments for tub data and stores them in records.
Uses lap detection, mean course computation, and course segmentation to
divide the track into geometric segments and assign each record to a segment.

Command: donkey segment <tub_path>

Example:
    donkey segment ./data/tub_1
    donkey segment --lap-detector ycrossing --strategy hybrid ./data/tub_1
"""

import argparse
import os

from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics
from donkeycar.config import load_config


class SegmentCommand:
    """Command for computing segment assignments in tub data."""

    def parse_args(self, args):
        parser = argparse.ArgumentParser(
            prog='segment',
            usage='%(prog)s [options] <tub_path>',
            description='Compute and assign segments to tub records')

        parser.add_argument('tub', type=str,
                           help='path to Tub directory')
        parser.add_argument('--lap-detector', type=str,
                           choices=['ycrossing', 'drift'],
                           default='ycrossing',
                           help='lap detection method (default: ycrossing)')
        parser.add_argument('--strategy', type=str,
                           choices=['threshold', 'extrema', 'gradient',
                                    'hybrid'],
                           default='hybrid',
                           help='segmentation strategy (default: hybrid)')
        parser.add_argument('--min-segment-length', type=float, default=1.0,
                           help='minimum segment length in meters (default: 1.0)')
        parser.add_argument('--curvature-threshold', type=float, default=0.1,
                           help='curvature threshold for segmentation (default: 0.1)')
        parser.add_argument('--config', type=str, default=None,
                           help='path to config file for field aggregations')
        parser.add_argument('--visualize', action='store_true',
                           help='show visualization after segmentation')

        return parser.parse_args(args)

    def run(self, args):
        args = self.parse_args(args)
        tub_path = os.path.expanduser(args.tub)

        if not os.path.exists(tub_path):
            print(f"Error: Tub directory {tub_path} not found.")
            return

        if not os.path.isdir(tub_path):
            print(f"Error: {tub_path} is not a directory.")
            return

        print("=" * 70)
        print("Segment Assignment")
        print("=" * 70)
        print(f"Tub path: {tub_path}")
        print(f"Lap detector: {args.lap_detector}")
        print(f"Segmentation strategy: {args.strategy}")
        print(f"Min segment length: {args.min_segment_length}m")
        print(f"Curvature threshold: {args.curvature_threshold}")
        print("=" * 70)

        print("\nLoading tub...")
        try:
            tub = Tub(tub_path, read_only=False)
            print(f"  {len(tub)} records found")
        except Exception as e:
            print(f"Error loading tub: {e}")
            import traceback
            traceback.print_exc()
            return

        # Load config if provided
        config = None
        if args.config:
            config_path = os.path.expanduser(args.config)
            if os.path.exists(config_path):
                print(f"\nLoading config from {config_path}...")
                try:
                    config = load_config(config_path)
                    print("  Config loaded successfully")
                except Exception as e:
                    print(f"Warning: Failed to load config: {e}")
                    print("  Using default field aggregations")
            else:
                print(f"Warning: Config file not found: {config_path}")
                print("  Using default field aggregations")

        print("\nComputing segment assignments...")
        try:
            stats = TubStatistics(tub, config=config)
            stats.compute_segment_assignments(
                lap_detector=args.lap_detector,
                segmentation_strategy=args.strategy,
                min_segment_length=args.min_segment_length,
                curvature_threshold=args.curvature_threshold
            )
            print("  Segment assignments computed successfully")
        except Exception as e:
            print(f"Error computing segments: {e}")
            import traceback
            traceback.print_exc()
            tub.close()
            return

        # Print summary
        print("\n" + "=" * 70)
        print("Summary:")
        print("=" * 70)

        sessions = tub.manifest.manifest_metadata['sessions']['all_full_ids']
        for session_id in sessions:
            session_dict = tub.manifest.metadata.get(session_id)
            if session_dict and 'segmentation' in session_dict:
                seg_info = session_dict['segmentation']
                num_segments = seg_info['num_segments']
                print(f"Session {session_id}: {num_segments} segments")
            else:
                print(f"Session {session_id}: No segmentation data")

        tub.close()
        print("\nSegment assignment complete!")

        if args.visualize:
            print("\nVisualization not yet implemented")
            # TODO: Add visualization using matplotlib or similar


def segment(args):
    """
    Entry point for segment command.

    Args:
        args: Command line arguments
    """
    cmd = SegmentCommand()
    cmd.run(args)
