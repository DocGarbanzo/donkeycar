"""
IMU Path Visualization

Interactive visualization for recorded vehicle paths.
Features time slider, lap selector, segment method selector, and real-time
navigation.

Command: donkey imupath <data_source>

Example:
    # Matplotlib UI (default)
    donkey imupath ./recording.csv
    donkey imupath ./tub_directory
    donkey imupath --lap-method drift --segment-method hybrid ./data.csv
    
    # Web-based UI
    donkey imupath --web ./recording.csv
    donkey imupath --web ./tub_directory
    
    # Web UI with custom config
    donkey imupath --web --config ./mycar/config.py ./tub_directory

Note: The --web flag launches a web server (default port 8887) for browser-based
visualization. Access at http://localhost:8887/imupath
"""

import argparse
import os

from donkeycar.config import load_config
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
        parser.add_argument('--config', type=str, default='./config.py',
                           help='path to config file for segment stats')
        parser.add_argument('--web', action='store_true',
                           help='launch web-based UI instead of matplotlib')

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

        cfg = None
        if args.config:
            config_path = os.path.expanduser(args.config)
            if os.path.exists(config_path):
                print("\nLoading config...")
                try:
                    cfg = load_config(config_path)
                    print(f"  Loaded config from {config_path}")
                except Exception as e:
                    print(f"Warning: Failed to load config: {e}")
                    print("  Using default segment fields")
            elif args.config != './config.py':
                print(f"Warning: Config file not found: {config_path}")
                print("  Using default segment fields")

        print("\nLoading data...")
        try:
            if os.path.isfile(data_source) and data_source.endswith('.csv'):
                source = CSVPathDataSource(data_source)
                tub_path = None
            elif os.path.isdir(data_source):
                source = TubPathDataSource(data_source)
                tub_path = data_source
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

        # Check for web mode
        if args.web:
            self._run_web_mode(args, path_data, cfg, data_source, tub_path)
            return

        # Check for segment statistics if Tub data
        if tub_path:
            print("\nComputing segment statistics...")

        print("\nInitializing visualization...")
        try:
            viz = InteractiveIMUVisualizer(
                path_data=path_data,
                cfg=cfg,
                lap_method=args.lap_method,
                segment_method=args.segment_method,
                file_path=data_source,
                tub_path=tub_path
            )
            viz.setup_ui()

            # Print segment stats info
            if tub_path:
                if viz.segment_rankings:
                    print(f"  ✓ Loaded segment rankings for "
                         f"{len(viz.segment_rankings)} records")
                    print(f"  Available metrics: "
                         f"{', '.join(viz.available_ranking_keys)}")
                    print("  Stats are computed on the fly (not written)")
                    if not args.config:
                        print("  Tip: Use --config to include additional "
                              "FIELD_AGGREGATIONS")
                else:
                    print("  ⓘ No segment performance data found")
                    print("  Check lap detection/segmentation settings")

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
    
    def _run_web_mode(self, args, path_data, cfg, data_source, tub_path):
        """Run web-based visualization mode."""
        print("\n" + "=" * 70)
        print("Web Mode - Starting Donkey Web Server")
        print("=" * 70)
        
        try:
            # Import web components
            from donkeycar.parts.web_controller.web import LocalWebController
            from donkeycar.web.imupath_data import IMUPathDataBuilder
            
            # Create data builder
            print("\nPreparing data for web visualization...")
            builder = IMUPathDataBuilder(
                path_data=path_data,
                cfg=cfg,
                lap_method=args.lap_method,
                segment_method=args.segment_method,
                tub_path=tub_path
            )
            
            # Print info
            print(f"  Detected {builder.multilap_data.num_laps} laps")
            print(f"  Mean course length: {builder.mean_course.length:.1f}m")
            print(f"  Segments: {builder.segmentation.num_segments}")
            
            if tub_path and builder.available_ranking_keys:
                print(f"  Available metrics: "
                      f"{', '.join(builder.available_ranking_keys)}")
            
            # Create web server
            port = 8887
            if cfg and hasattr(cfg, 'WEB_CONTROL_PORT'):
                port = cfg.WEB_CONTROL_PORT
            
            web = LocalWebController(port=port)
            web.imupath_builder = builder
            
            print("\n" + "=" * 70)
            print(f"Web UI available at:")
            print(f"  http://localhost:{port}/imupath")
            from socket import gethostname
            print(f"  http://{gethostname()}.local:{port}/imupath")
            print("\nPress Ctrl+C to stop the server")
            print("=" * 70)
            
            # Start server (blocking)
            web.update()
            
        except KeyboardInterrupt:
            print("\n\nShutting down web server...")
        except Exception as e:
            print(f"\nError starting web server: {e}")
            import traceback
            traceback.print_exc()

