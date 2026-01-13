"""
Interactive IMU Path Visualization

This module provides an interactive visualization for IMU path analysis using
the refactored course_analysis API. It includes widgets for real-time
navigation, lap filtering, and segmentation method selection.

Classes:
    InteractiveIMUVisualizer: Main class for interactive visualization
"""

import itertools
import os
import time
from datetime import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import (
    Slider, CheckButtons, RadioButtons, Button, TextBox
)
import logging

from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics, FieldAccumulator
from donkeycar.pipeline.transformations import default_lap_sorting_strategy

# Import new refactored course_analysis API
from donkeycar.course_analysis import (
    YCrossingLapDetector,
    DriftLapDetector,
    MultiLapData,
    MeanCourseBuilder,
    CourseSegmenter,
    SegmentAssigner,
    ThresholdSegmentation,
    ExtremaSegmentation,
    GradientSegmentation,
    HybridSegmentation,
)

logger = logging.getLogger(__name__)

# Color constants
MEAN_COURSE_COLOR = '#808080'  # Grey for mean course and segments


class InteractiveIMUVisualizer:
    """
    Interactive matplotlib visualization for IMU path analysis.
    Uses the refactored course_analysis API.

    Features:
    - Time slider for navigating through recorded path
    - Lap selector to change number of laps used for mean course
    - Segment method selector to switch between segmentation strategies
    - Display toggles for driven path and mean course visibility
    - Keyboard navigation (left/right arrows)
    - Real-time status panel with position, speed, lap, segment info
    - Segment boundary markers and labels
    - Performance optimizations (throttling, downsampling)
    """

    def __init__(self, path_data, cfg, lap_method='y_crossing',
                 segment_method='gradient', file_path='', tub_path=None):
        """
        Initialize the interactive visualizer.

        Args:
            path_data: PathData object with immutable position/velocity data
            cfg: Configuration object with parameters
            lap_method: Initial lap detection method ('y_crossing' or 'drift')
            segment_method: Initial segmentation method
                ('threshold', 'extrema', 'gradient', 'hybrid')
            file_path: Path to source data file (for display)
            tub_path: Path to Tub directory if source is Tub
                (for segment stats)
        """
        # Store immutable data
        self.path_data = path_data
        self.cfg = cfg
        self.file_path = file_path
        self.tub_path = tub_path
        self.is_tub_data = (tub_path is not None)

        # Create DataFrame for easier time-based indexing
        self.df = pd.DataFrame({
            't': path_data.timestamp,
            'x': path_data.x,
            'y': path_data.y,
            'h': path_data.heading,
            'v': path_data.velocity
        })

        # Current settings (mutable UI state)
        self.lap_method = lap_method
        self.segment_method = segment_method
        self.num_laps_for_mean = None  # Will be set after lap detection

        # Data processing results (will be set by _initialize_data_pipeline)
        self.multilap_data = None
        self.mean_course = None
        self.segmentation = None
        self.segment_ids = None

        # Matplotlib figure and axis
        self.fig = None
        self.ax = None

        # Plot artists (will be created in setup_ui)
        self.full_path_scatter = None
        self.current_pos_marker = None
        self.current_path_line = None
        self.mean_course_line = None
        self.mean_course_segments = []
        self.segment_markers = []
        self.segment_labels = []
        self.colorbar = None

        # Widgets (will be created in setup_ui)
        self.time_slider = None
        self.display_toggles = None
        self.lap_textbox = None
        self.lap_minus_button = None
        self.lap_plus_button = None
        self.segment_radio = None
        self.stats_radio = None

        # Status text elements (will be created in setup_ui)
        self.status_texts = {}
        self.panel_background_text = None
        self.panel_line_texts = {}

        # Segment statistics (for Tub data only)
        self.segment_rankings = {}
        self.available_ranking_keys = []
        self.current_stats_field = None

        # Performance tracking
        self.last_update_time = 0
        self.throttle_interval = 0.1  # 100ms minimum between updates

        # Initialize data processing pipeline
        self._initialize_data_pipeline()

        # Load segment statistics if Tub data
        self._load_segment_statistics()

    def _initialize_data_pipeline(self):
        """
        Initialize the complete data processing pipeline:
        Phase 2: Lap detection
        Phase 3: Mean course building
        Phase 4: Segmentation
        Phase 5: Segment assignment
        """
        logger.info("Initializing data processing pipeline...")

        # Phase 2: Lap Detection
        if self.lap_method == 'y_crossing':
            detector = YCrossingLapDetector(cfg=self.cfg)
        else:
            detector = DriftLapDetector(cfg=self.cfg)

        # Detect lap boundaries
        lap_boundaries = detector.detect_laps(self.path_data)

        # Create MultiLapData
        self.multilap_data = MultiLapData(self.path_data, lap_boundaries)

        # Default: use all laps for mean course
        self.num_laps_for_mean = self.multilap_data.num_laps

        logger.info(f"Detected {self.multilap_data.num_laps} laps")

        # Phase 3: Mean Course Building
        self._rebuild_mean_course()

    def _rebuild_mean_course(self):
        """
        Rebuild mean course from current lap selection.
        Also rebuilds segmentation and assignment (downstream dependencies).
        """
        # Create limited MultiLapData if user selected fewer laps
        if self.num_laps_for_mean < self.multilap_data.num_laps:
            limited_boundaries = self.multilap_data.lap_boundaries[
                :self.num_laps_for_mean]
            limited_data = MultiLapData(
                self.multilap_data.path_data, limited_boundaries)
        else:
            limited_data = self.multilap_data

        # Build mean course
        builder = MeanCourseBuilder(cfg=self.cfg)
        self.mean_course = builder.build(limited_data)

        logger.info(f"Built mean course from {self.num_laps_for_mean} laps, "
                   f"length: {self.mean_course.length:.1f}m")

        # Rebuild segmentation (depends on mean course)
        self._rebuild_segmentation()

    def _rebuild_segmentation(self):
        """
        Rebuild segmentation using current method.
        Also rebuilds segment assignment (downstream dependency).
        """
        # Create segmentation strategy
        if self.segment_method == 'threshold':
            strategy = ThresholdSegmentation()
        elif self.segment_method == 'extrema':
            strategy = ExtremaSegmentation()
        elif self.segment_method == 'gradient':
            strategy = GradientSegmentation()
        else:  # hybrid
            strategy = HybridSegmentation()

        # Segment the mean course
        segmenter = CourseSegmenter(strategy, cfg=self.cfg)
        self.segmentation = segmenter.segment(self.mean_course)

        logger.info(f"Segmented course using {self.segment_method}, "
                   f"found {self.segmentation.num_segments} segments")

        # Assign segments to full driven path
        assigner = SegmentAssigner(self.segmentation)
        self.segment_ids = assigner.assign(
            self.path_data.x, self.path_data.y)

        logger.info("Assigned segments to driven path")

    def _load_segment_statistics(self):
        """
        Compute segment rankings using visualization's detected laps.

        Uses the on-the-fly lap detection (multilap_data) and segment
        assignments (segment_ids) to compute performance rankings.
        This approach works regardless of whether car/lap is set in records.
        """
        if not self.is_tub_data or not self.tub_path:
            return
        if self.segment_ids is None or self.multilap_data is None:
            return
        if self.segmentation is None or self.segmentation.num_segments == 0:
            return

        try:
            print("Computing segment statistics...")
            self._compute_segment_rankings_from_visualization()
            self._initialize_ranking_keys()
        except Exception as e:
            logger.error(f"Failed to compute segment statistics: {e}")
            self.segment_rankings = {}
            self.available_ranking_keys = []

    def _compute_segment_rankings_from_visualization(self):
        """
        Compute segment rankings using visualization's lap/segment data.

        Uses self.multilap_data.lap_boundaries for lap info and
        self.segment_ids for segment assignments. Reads tub records
        to aggregate field values per (lap, segment) combination.
        """
        logger.info("Computing segment performance rankings...")

        tub = Tub(self.tub_path, read_only=True)
        try:
            field_specs, sorting_strategy = self._load_stats_config(tub)
            lap_end_indices = self._build_lap_end_indices()
            segment_instances = self._collect_segment_instances(
                tub, lap_end_indices, field_specs)
        finally:
            tub.close()

        active_laps = len(self._active_lap_boundaries())
        segment_lap_rankings = self._compute_segment_lap_rankings(
            segment_instances, sorting_strategy, active_laps)
        self._map_indices_to_rankings(segment_lap_rankings)

    def _load_stats_config(self, tub):
        """Load field aggregation specs and sorting strategy."""
        stats = TubStatistics(tub, config=self.cfg)
        return stats.field_aggregations, stats.sorting_strategy

    def _build_lap_end_indices(self):
        """Build set of lap end indices for O(1) lookup."""
        return {b.end_index for b in self._active_lap_boundaries()}

    def _collect_segment_instances(self, tub, lap_end_indices, field_specs):
        """Collect segment instances with metrics from tub records."""
        from collections import defaultdict

        segment_instances = defaultdict(list)
        state = self._init_segment_tracking_state(field_specs)
        last_timestamp_ms = 0
        last_distance = 0.0
        max_idx = self._last_boundary_index()

        for idx, record in enumerate(tub):
            if max_idx is not None and idx > max_idx:
                break
            if idx >= len(self.segment_ids):
                break

            last_timestamp_ms = record.get('_timestamp_ms', 0)
            last_distance = record.get('car/distance', 0.0)

            self._process_record_for_segment(
                idx, record, state, segment_instances, lap_end_indices,
                field_specs)

        # Finalize last segment using tracked values
        self._finalize_segment_metrics(
            segment_instances, state['lap'], state['segment'],
            state['start_time'], last_timestamp_ms,
            state['start_dist'], last_distance, state['field_accumulators'])

        return segment_instances

    def _init_segment_tracking_state(self, field_specs):
        """Initialize state for segment tracking."""
        initial_segment = (
            self.segment_ids[0] if len(self.segment_ids) > 0 else None)
        return {
            'lap': 0,
            'segment': initial_segment,
            'start_time': None,
            'start_dist': 0.0,
            'field_accumulators': self._create_field_accumulators(field_specs)
        }

    def _process_record_for_segment(self, idx, record, state,
                                    segment_instances, lap_end_indices,
                                    field_specs):
        """Process a single record for segment metrics collection."""
        segment = self.segment_ids[idx]
        timestamp_ms = record.get('_timestamp_ms', 0)
        distance = record.get('car/distance', 0.0)

        # Initialize start values on first record
        if state['start_time'] is None:
            state['start_time'] = timestamp_ms
            state['start_dist'] = distance

        self._accumulate_field_values(
            record, field_specs, state['field_accumulators'])

        # Check for segment or lap change
        if segment == state['segment'] and idx not in lap_end_indices:
            return

        # Finalize current segment instance
        self._finalize_segment_metrics(
            segment_instances, state['lap'], state['segment'],
            state['start_time'], timestamp_ms,
            state['start_dist'], distance, state['field_accumulators'])

        # Update lap if boundary crossed
        if idx in lap_end_indices:
            state['lap'] += 1

        # Reset for new segment
        state['segment'] = segment
        state['start_time'] = timestamp_ms
        state['start_dist'] = distance
        state['field_accumulators'] = self._create_field_accumulators(
            field_specs)

    def _create_field_accumulators(self, field_specs):
        """Create accumulators for each configured field."""
        return {
            spec.output_key: FieldAccumulator(spec.aggregation)
            for spec in field_specs
        }

    def _accumulate_field_values(self, record, field_specs, accumulators):
        """Accumulate configured field values from a tub record."""
        for spec in field_specs:
            value = spec.extract(record)
            if value is None:
                continue
            accumulators[spec.output_key].add(value)

    def _finalize_segment_metrics(self, segment_instances, lap, segment_id,
                                  start_time, end_time, start_dist, end_dist,
                                  field_accumulators):
        """Finalize segment metrics and add to collection."""
        if segment_id is None or start_time is None:
            return

        seg_time = (end_time - start_time) / 1000.0
        seg_dist = end_dist - start_dist

        instance = {
            'lap': lap,
            'time': seg_time,
            'distance': seg_dist
        }
        for output_key, accumulator in field_accumulators.items():
            value = accumulator.compute()
            if value is None:
                continue
            instance[output_key] = value

        segment_instances[segment_id].append(instance)

    def _compute_segment_lap_rankings(self, segment_instances,
                                      sorting_strategy, num_laps):
        """Compute rankings for each segment's lap instances."""
        segment_lap_rankings = {}
        for segment_id, instances in segment_instances.items():
            if not instances:
                continue
            segment_lap_rankings[segment_id] = self._rank_segment_instances(
                instances, sorting_strategy, num_laps)
        return segment_lap_rankings

    def _rank_segment_instances(self, instances, sorting_strategy, num_laps):
        """Rank all instances of a single segment by configured criteria."""
        strategy = sorting_strategy or default_lap_sorting_strategy()
        filtered = instances
        if num_laps is not None:
            filtered = [inst for inst in instances
                       if inst.get('lap') is not None and
                       inst['lap'] < num_laps]
        deduped = self._select_best_instances_by_lap(filtered, strategy)
        num_buckets = num_laps or None
        rankings = strategy.rank_laps(deduped, num_buckets)
        lap_rankings = {}
        for inst_idx, inst_rankings in rankings.items():
            lap = deduped[inst_idx]['lap']
            lap_rankings[lap] = inst_rankings
        return lap_rankings

    def _map_indices_to_rankings(self, segment_lap_rankings):
        """Map record indices to their segment rankings."""
        active_laps = len(self._active_lap_boundaries())
        for idx, segment_id in enumerate(self.segment_ids):
            lap = self._find_lap_for_index(idx)
            if lap >= active_laps:
                continue
            rankings = segment_lap_rankings.get(segment_id, {}).get(lap)
            if rankings:
                self.segment_rankings[idx] = rankings

    def _find_lap_for_index(self, idx):
        """Find which lap contains the given index."""
        boundaries = self._active_lap_boundaries()
        for lap_idx, boundary in enumerate(boundaries):
            if idx <= boundary.end_index:
                return lap_idx
        return len(boundaries)

    def _active_lap_boundaries(self):
        """Return lap boundaries for the current lap selection."""
        if self.num_laps_for_mean is None:
            boundaries = self.multilap_data.lap_boundaries
        else:
            boundaries = self.multilap_data.lap_boundaries[
                :self.num_laps_for_mean]
        return sorted(boundaries, key=lambda b: b.end_index)

    def _last_boundary_index(self):
        """Return last boundary index or None if no boundaries."""
        boundaries = self._active_lap_boundaries()
        if not boundaries:
            return None
        return boundaries[-1].end_index

    def _lap_start_index(self, lap_idx):
        """Compute lap start index for a given lap index."""
        if lap_idx <= 0:
            return 0
        boundaries = self._active_lap_boundaries()
        if lap_idx - 1 < len(boundaries):
            return boundaries[lap_idx - 1].end_index + 1
        if boundaries:
            return boundaries[-1].end_index + 1
        return 0

    def _initialize_ranking_keys(self):
        """Initialize available ranking keys from first ranking."""
        if not self.segment_rankings:
            return

        first_ranking = next(iter(self.segment_rankings.values()))
        self.available_ranking_keys = sorted(first_ranking.keys())
        self.current_stats_field = self.available_ranking_keys[0]
        logger.info(f"Loaded rankings: "
                   f"{', '.join(self.available_ranking_keys)}")

    def _select_best_instances_by_lap(self, instances, strategy):
        """Pick one instance per lap using the sorting criteria."""
        if not instances:
            return []

        criteria = strategy.criteria

        def sort_key(instance):
            key = []
            for spec in criteria:
                value = instance.get(spec['key'])
                if value is None:
                    value = float('inf')
                else:
                    value = spec['transform'](value)
                if spec['reverse']:
                    value = -value
                key.append(value)
            return tuple(key)

        best_by_lap = {}
        for instance in instances:
            lap = instance.get('lap')
            if lap is None:
                continue
            key = sort_key(instance)
            if lap not in best_by_lap or key < best_by_lap[lap][0]:
                best_by_lap[lap] = (key, instance)

        return [best_by_lap[lap][1] for lap in sorted(best_by_lap)]

    def setup_ui(self):
        """
        Create all matplotlib widgets and set up event handlers.
        """
        # Create figure with single axis
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.canvas.manager.set_window_title('Donkey imupath')

        # Adjust layout to make room for widgets
        plt.subplots_adjust(bottom=0.10, right=0.95, top=0.87, left=0.08)

        # Set up axis
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')

        # Create visualization elements
        self._create_plot_artists()
        self._create_status_panel()
        self._create_widgets()

        # Initial plot update
        self._update_plot(self.df['t'].min())

        # Status panel now includes legend and controls

    def _create_styled_text(self, y_pos, text='', color='white'):
        """Create styled text element for UI (status panel and legend)"""
        return self.fig.text(
            0.02, y_pos, text,
            transform=self.fig.transFigure,
            fontsize=9,
            color=color,
            va='top',
            bbox=dict(boxstyle='round', facecolor='black', alpha=0.8)
        )

    def _create_plot_artists(self):
        """Create all plot artists (scatter, markers, lines, etc.)"""
        # Full path scatter plot (velocity-colored, faded)
        self.full_path_scatter = self.ax.scatter(
            self.df['x'], self.df['y'],
            c=self.df['v'],
            cmap='viridis',
            s=8,
            alpha=0.8
        )

        # Colorbar for velocity
        self.colorbar = self.fig.colorbar(
            self.full_path_scatter, ax=self.ax, label='Speed (m/s)')

        # Current position marker (red circle)
        self.current_pos_marker, = self.ax.plot(
            [], [],
            'o',
            color='#FF6B6B',
            markersize=10,
            label='Current position'
        )

        # Path to current time (red line)
        self.current_path_line, = self.ax.plot(
            [], [],
            color='#FF6B6B',
            linewidth=1,
            alpha=0.8,
            label='Current path'
        )

        # Mean course visualization (will be filled by _refresh_mean_course)
        self._refresh_mean_course()

    def _create_status_panel(self):
        """Create unified status panel text box."""
        self.status_texts = {}
        self._update_status_panel(0)

    def _create_widgets(self):
        """Create all interactive widgets"""
        # Time slider
        ax_slider = plt.axes([0.02, 0.005, 0.96, 0.018])
        self.time_slider = Slider(
            ax_slider, '', self.df['t'].min(), self.df['t'].max(),
            valinit=self.df['t'].min(), valfmt=''
        )
        self.time_slider.on_changed(self._on_time_slider_changed)

        # Display toggles - wider box to fit text
        ax_toggle = plt.axes([0.02, 0.23, 0.16, 0.05])
        toggle_labels = ['Driven Path', 'Mean Course']
        toggle_actives = [True, True]
        self.display_toggles = CheckButtons(
            ax_toggle, toggle_labels, toggle_actives)
        # Make checkbox borders white and check marks cyan
        for rect in self.display_toggles.rectangles:
            rect.set_edgecolor('white')
            rect.set_linewidth(1.5)
        all_lines = itertools.chain.from_iterable(self.display_toggles.lines)
        for line in all_lines:
            line.set_color('cyan')
            line.set_linewidth(2.5)
        self.display_toggles.on_clicked(self._on_display_toggle)

        # Lap selector (only if multiple laps detected)
        if self.multilap_data.num_laps > 1:
            # Label
            self.fig.text(0.02, 0.20, 'Laps',
                          transform=self.fig.transFigure, fontsize=10)

            # TextBox
            ax_textbox = plt.axes([0.02, 0.165, 0.065, 0.03])
            self.lap_textbox = TextBox(
                ax_textbox, '', initial=str(self.num_laps_for_mean),
                color='white', hovercolor='lightgray')
            self.lap_textbox.label.set_color('black')
            self.lap_textbox.text_disp.set_color('black')
            self.lap_textbox.on_submit(self._on_lap_text_submit)

            # Minus button
            ax_minus = plt.axes([0.02, 0.135, 0.03, 0.025])
            self.lap_minus_button = Button(
                ax_minus, '−', color='#1a1a1a', hovercolor='#333333')
            self.lap_minus_button.on_clicked(self._on_lap_minus)

            # Plus button
            ax_plus = plt.axes([0.06, 0.135, 0.03, 0.025])
            self.lap_plus_button = Button(
                ax_plus, '+', color='#1a1a1a', hovercolor='#333333')
            self.lap_plus_button.on_clicked(self._on_lap_plus)

        # Segment method selector - wider box to fit text
        ax_radio = plt.axes([0.02, 0.30, 0.16, 0.10], facecolor='#1a1a1a')
        self.fig.text(0.02, 0.42, 'Segment Method',
                     transform=self.fig.transFigure, fontsize=10)
        radio_labels = ['Threshold', 'Extrema', 'Gradient', 'Hybrid']
        active_idx = {'threshold': 0, 'extrema': 1,
                     'gradient': 2, 'hybrid': 3}[self.segment_method]
        self.segment_radio = RadioButtons(
            ax_radio, radio_labels, active=active_idx)
        # Make radio button circles white
        for circle in self.segment_radio.circles:
            circle.set_edgecolor('white')
            circle.set_linewidth(1.5)
        self.segment_radio.on_clicked(self._on_segment_method_changed)

        # Segment statistics field selector (only if data available)
        if self.available_ranking_keys:
            ax_stats = plt.axes([0.02, 0.04, 0.16, 0.055],
                               facecolor='#1a1a1a')
            self.fig.text(0.02, 0.11, 'Segment Stats',
                          transform=self.fig.transFigure, fontsize=10)

            # Create display labels (capitalize, replace underscores)
            display_labels = [
                key.replace('_', ' ').title()
                for key in self.available_ranking_keys
            ]

            self.stats_radio = RadioButtons(
                ax_stats, display_labels, active=0)

            # Style radio buttons (match segment method selector style)
            for circle in self.stats_radio.circles:
                circle.set_edgecolor('white')
                circle.set_linewidth(1.5)

            self.stats_radio.on_clicked(self._on_stats_field_changed)

        # Keyboard navigation
        self.fig.canvas.mpl_connect('key_press_event',
                                    self._on_keyboard_press)

    def _update_plot(self, timestamp):
        """Update plot to show data up to given timestamp"""
        # Throttle updates for performance
        current_time = time.time()
        if current_time - self.last_update_time < self.throttle_interval:
            return
        self.last_update_time = current_time

        # Filter data to current timestamp
        mask = self.df['t'] <= timestamp
        current_data = self.df[mask]

        if len(current_data) == 0:
            return

        # Get current index for status panel
        current_idx = len(current_data) - 1

        # Update current position marker
        self.current_pos_marker.set_data(
            [current_data.iloc[-1]['x']],
            [current_data.iloc[-1]['y']]
        )

        # Update current path line (downsample if needed)
        if len(current_data) > 1000:
            step = len(current_data) // 1000
            display_data = current_data.iloc[::step]
        else:
            display_data = current_data

        self.current_path_line.set_data(
            display_data['x'], display_data['y'])

        # Update status panel
        self._update_status_panel(current_idx)

        # Redraw
        self.fig.canvas.draw_idle()

    def _refresh_mean_course(self):
        """Refresh mean course visualization"""
        # Clear old visualization
        if self.mean_course_line is not None:
            self.mean_course_line.remove()
            self.mean_course_line = None

        for line in self.mean_course_segments:
            line.remove()
        self.mean_course_segments = []

        if self.mean_course is None:
            return

        # Draw based on segmentation availability
        if (self.segmentation is None or
                self.segmentation.num_segments == 0):
            # Unsegmented: single grey line
            self.mean_course_line, = self.ax.plot(
                self.mean_course.x,
                self.mean_course.y,
                color=MEAN_COURSE_COLOR,
                linewidth=1.8,
                alpha=0.95,
                label=f'Mean course ({self.mean_course.length:.1f}m)',
                visible=True
            )
        else:
            # Segmented: draw each segment
            for seg in self.segmentation.segments:
                # Extract segment data from mean course
                start_idx = seg.start_index
                end_idx = seg.end_index + 1
                line, = self.ax.plot(
                    self.mean_course.x[start_idx:end_idx],
                    self.mean_course.y[start_idx:end_idx],
                    color=MEAN_COURSE_COLOR,
                    linewidth=2.5,
                    alpha=0.95,
                    visible=True
                )
                self.mean_course_segments.append(line)

            # Create invisible line for legend
            self.mean_course_line, = self.ax.plot(
                [], [],
                color=MEAN_COURSE_COLOR,
                linewidth=2.5,
                alpha=0.95,
                label=f'Mean course ({self.mean_course.length:.1f}m)',
                visible=True
            )

        # Refresh segment markers and labels
        self._refresh_segment_markers()
        self._refresh_segment_labels()

    def _refresh_segment_markers(self):
        """Refresh segment boundary markers"""
        # Clear old markers
        for line in self.segment_markers:
            line.remove()
        self.segment_markers = []

        if (self.segmentation is None or self.mean_course is None or
                self.segmentation.num_segments == 0):
            return

        # Calculate marker length (2% of course length, min 0.3m)
        marker_len = max(0.3, 0.02 * self.mean_course.length)

        # Get boundaries
        boundaries = getattr(self.segmentation, 'segment_boundaries', None)
        if not boundaries:
            return

        # Draw perpendicular tick at each boundary
        for boundary in boundaries:
            self._draw_boundary_marker(boundary, marker_len)

    def _draw_boundary_marker(self, boundary, marker_len):
        """Draw a single boundary marker"""
        point = boundary.get('point')
        normal = boundary.get('normal')
        if point is None or normal is None:
            return

        # Normalize the normal vector
        norm = np.linalg.norm(normal)
        if norm == 0:
            return

        normal_vec = normal / norm

        # Calculate tick endpoints
        dx = normal_vec[0] * marker_len * 0.5
        dy = normal_vec[1] * marker_len * 0.5

        # Draw marker
        line, = self.ax.plot(
            [point[0] - dx, point[0] + dx],
            [point[1] - dy, point[1] + dy],
            color=MEAN_COURSE_COLOR,
            linewidth=2.5,
            alpha=1.0,
            visible=True
        )
        self.segment_markers.append(line)

    def _refresh_segment_labels(self):
        """Refresh segment number labels"""
        # Clear old labels
        for label in self.segment_labels:
            label.remove()
        self.segment_labels = []

        if (self.segmentation is None or
                self.segmentation.num_segments == 0):
            return

        # Draw label at each segment midpoint
        for seg in self.segmentation.segments:
            self._draw_segment_label(seg)

    def _draw_segment_label(self, seg):
        """Draw a single segment label"""
        # Extract segment data from mean course
        start_idx = seg.start_index
        end_idx = seg.end_index + 1
        seg_len = end_idx - start_idx

        if seg_len == 0:
            return

        # Calculate midpoint index
        mid_offset = seg_len // 2
        mid_idx = start_idx + mid_offset
        mid_x = self.mean_course.x[mid_idx]
        mid_y = self.mean_course.y[mid_idx]

        # Create label with circle background
        label = self.ax.text(
            mid_x, mid_y, str(seg.segment_id),
            fontsize=10,
            color='black',
            ha='center', va='center',
            bbox=dict(
                boxstyle='circle',
                facecolor='white',
                edgecolor='darkred',
                alpha=0.7
            )
        )
        self.segment_labels.append(label)

    def _update_status_panel(self, current_idx):
        """Update unified status panel text."""
        if current_idx < 0 or current_idx >= len(self.df):
            return

        # Get current data point
        row = self.df.iloc[current_idx]
        panel_lines = []

        display_path = self._format_display_path()
        panel_lines.append(('file', f'File: {display_path}', 'white'))
        panel_lines.append(('speed', f'Speed: {row["v"]:.2f} m/s', 'white'))

        # Update time
        current_datetime = datetime.fromtimestamp(row['t'])
        time_str = current_datetime.strftime('%Y-%m-%d, %H:%M:%S')
        panel_lines.append(('time', f'Time: {time_str}', 'white'))

        # Update position
        panel_lines.append(
            ('position', f'Position: [{row["x"]:.2f}, {row["y"]:.2f}]',
             'white'))

        # Calculate current lap
        lap_idx = self._find_lap_for_index(current_idx)
        boundaries = self._active_lap_boundaries()
        if boundaries and current_idx > boundaries[-1].end_index:
            lap_display = len(boundaries) + 1
        else:
            lap_display = lap_idx + 1
        panel_lines.append(('lap', f'Lap: {lap_display}', 'white'))

        # Calculate total distance (cumulative)
        if current_idx > 0:
            dx = np.diff(self.df['x'][:current_idx + 1])
            dy = np.diff(self.df['y'][:current_idx + 1])
            total_dist = np.sum(np.sqrt(dx**2 + dy**2))
        else:
            total_dist = 0.0

        panel_lines.append(
            ('total_dist', f'Total distance: {total_dist:.2f}m', 'white'))

        # Calculate lap distance
        lap_start_idx = self._lap_start_index(lap_idx)
        if current_idx > lap_start_idx:
            dx = np.diff(self.df['x'][lap_start_idx:current_idx + 1])
            dy = np.diff(self.df['y'][lap_start_idx:current_idx + 1])
            lap_dist = np.sum(np.sqrt(dx**2 + dy**2))
        else:
            lap_dist = 0.0

        panel_lines.append(
            ('lap_dist', f'Lap distance: {lap_dist:.2f}m', 'white'))

        # Get current segment
        if self.segment_ids is not None and current_idx < len(
                self.segment_ids):
            seg_id = self.segment_ids[current_idx]
            seg_type = (self.segmentation.segments[seg_id].segment_type.name
                       if seg_id < len(self.segmentation.segments) else '--')
            panel_lines.append(
                ('segment', f'Segment: {seg_id} ({seg_type})', 'white'))
        else:
            panel_lines.append(('segment', 'Segment: --', 'white'))

        # Update segment ranking (only if available)
        seg_rank_line = self._segment_rank_line(current_idx)
        if seg_rank_line:
            panel_lines.append(('seg_rank', *seg_rank_line))

        # Debug info
        dist_to_origin = np.sqrt(row['x']**2 + row['y']**2)
        panel_lines.append(
            ('debug', f'Debug: idx={current_idx}, dist={dist_to_origin:.2f}m',
             'white'))

        panel_lines.append(
            ('controls', 'Controls: \u2190 \u2192 arrows to navigate',
             'yellow'))
        panel_lines.extend(self._legend_lines())

        self._render_panel_lines(panel_lines)

    def _should_show_segment_ranking(self, current_idx):
        """Check if segment ranking should be displayed."""
        return (self.segment_rankings and
                current_idx in self.segment_rankings and
                self.current_stats_field)

    def _display_segment_ranking(self, current_idx):
        """Display segment ranking with color coding."""
        ranking = self.segment_rankings[current_idx]
        value = ranking.get(self.current_stats_field)

        if value is None:
            self.status_texts['seg_ranking'].set_text('')
            return

        text, color = self._format_ranking_display(value)
        self.status_texts['seg_ranking'].set_text(text)
        self.status_texts['seg_ranking'].set_color(color)

    def _format_ranking_display(self, value):
        """Format ranking value with color coding."""
        pct = value * 100
        color = self._get_ranking_color(pct)

        field_display = (self.current_stats_field
                        .replace('_', ' ').title())
        text = f'Seg Rank ({field_display}): {pct:.0f}%'

        return text, color

    def _segment_rank_line(self, current_idx):
        """Return segment rank line if available."""
        if not self._should_show_segment_ranking(current_idx):
            return None
        ranking = self.segment_rankings[current_idx]
        value = ranking.get(self.current_stats_field)
        if value is None:
            return None
        field_display = (self.current_stats_field
                        .replace('_', ' ').title())
        pct = value * 100
        text, color = self._format_ranking_display(value)
        text = f'Seg Rank ({field_display}): {pct:.0f}%'
        return (text, color)

    def _legend_lines(self):
        """Return legend lines for unified status panel."""
        lines = [
            ('legend_pos', '\u2022 Current position', '#FF6B6B'),
            ('legend_path', '\u2022 Current path', '#FF6B6B')
        ]
        if self.mean_course is not None and self.mean_course.length > 0:
            lines.append(
                ('legend_mean',
                 f'\u2022 Mean course ({self.mean_course.length:.1f}m)',
                 MEAN_COURSE_COLOR))
        if self.segmentation is not None and self.segmentation.num_segments > 0:
            lines.append(
                ('legend_bounds',
                 f'\u2022 Segment boundaries '
                 f'({self.segmentation.num_segments})',
                 MEAN_COURSE_COLOR))
        return lines

    def _format_display_path(self):
        """Format file path for display."""
        expanded_path = os.path.abspath(os.path.expanduser(self.file_path))
        home_dir = os.path.expanduser('~')
        if expanded_path.startswith(home_dir):
            return expanded_path.replace(home_dir, '~', 1)
        return expanded_path

    def _render_panel_lines(self, lines):
        """Render colored lines inside a single background box."""
        y_top = 0.95
        x_left = 0.02
        line_height = 0.018

        plain_lines = [text for _, text, _ in lines]
        panel_text = '\n'.join(plain_lines)

        if self.panel_background_text is None:
            self.panel_background_text = self.fig.text(
                x_left, y_top, panel_text,
                transform=self.fig.transFigure,
                fontsize=9,
                color='white',
                va='top',
                linespacing=1.25,
                alpha=0.0,
                bbox=dict(boxstyle='round', facecolor='black',
                          alpha=0.8, edgecolor='white',
                          linewidth=1)
            )
        else:
            self.panel_background_text.set_text(panel_text)
            self.panel_background_text.set_position((x_left, y_top))

        existing = set(self.panel_line_texts.keys())
        seen = set()

        for idx, (key, text, color) in enumerate(lines):
            y_pos = y_top - idx * line_height
            seen.add(key)
            if key not in self.panel_line_texts:
                self.panel_line_texts[key] = self.fig.text(
                    x_left + 0.01, y_pos, text,
                    transform=self.fig.transFigure,
                    fontsize=9,
                    color=color,
                    va='top'
                )
                continue
            text_obj = self.panel_line_texts[key]
            text_obj.set_position((x_left + 0.01, y_pos))
            text_obj.set_text(text)
            text_obj.set_color(color)

        for key in existing - seen:
            self.panel_line_texts[key].remove()
            del self.panel_line_texts[key]

    def _get_ranking_color(self, pct):
        """Get color for ranking percentage."""
        if pct < 33:
            return '#4CAF50'  # Green
        if pct < 66:
            return '#FFC107'  # Yellow
        return '#F44336'  # Red

    # Widget event handlers

    def _on_time_slider_changed(self, val):
        """Handle time slider value change"""
        self._update_plot(val)

    def _on_display_toggle(self, label):
        """Handle display toggle checkbox clicks"""
        if label == 'Driven Path':
            self._toggle_driven_path_visibility()
            return

        if label == 'Mean Course':
            self._toggle_mean_course_visibility()
            return

        self.fig.canvas.draw_idle()

    def _toggle_driven_path_visibility(self):
        """Toggle driven path scatter visibility"""
        self.full_path_scatter.set_visible(
            not self.full_path_scatter.get_visible())
        self.fig.canvas.draw_idle()

    def _toggle_mean_course_visibility(self):
        """Toggle mean course and all related elements"""
        visible = not (self.mean_course_line.get_visible()
                      if self.mean_course_line else True)

        if not self.mean_course_line:
            return

        self.mean_course_line.set_visible(visible)
        for line in self.mean_course_segments:
            line.set_visible(visible)
        for marker in self.segment_markers:
            marker.set_visible(visible)
        for label_obj in self.segment_labels:
            label_obj.set_visible(visible)

        self.fig.canvas.draw_idle()

    def _on_lap_text_submit(self, text):
        """Handle lap textbox submission"""
        try:
            num_laps = int(text)
            self._set_lap_value(num_laps)
        except ValueError:
            # Reset to current value if invalid
            self.lap_textbox.set_val(str(self.num_laps_for_mean))

    def _on_lap_minus(self, event):
        """Handle lap minus button click"""
        self._set_lap_value(self.num_laps_for_mean - 1)

    def _on_lap_plus(self, event):
        """Handle lap plus button click"""
        self._set_lap_value(self.num_laps_for_mean + 1)

    def _refresh_visualizations(self):
        """Refresh mean course visualization and legend"""
        self._refresh_mean_course()
        self._refresh_segment_statistics()
        self.fig.canvas.draw_idle()

    def _refresh_segment_statistics(self):
        """Recompute segment rankings after lap/segment changes."""
        if not self.is_tub_data:
            return
        if self.segment_ids is None:
            return
        if self.segmentation is None or self.segmentation.num_segments == 0:
            return

        try:
            self.segment_rankings = {}
            self.available_ranking_keys = []
            self.current_stats_field = None
            self._compute_segment_rankings_from_visualization()
            self._initialize_ranking_keys()
            self._refresh_current_ranking_display()
        except Exception as e:
            logger.error(f"Failed to refresh segment statistics: {e}")
            self.segment_rankings = {}
            self.available_ranking_keys = []
            self.current_stats_field = None

    def _refresh_current_ranking_display(self):
        """Refresh ranking display for current slider position."""
        if not self.time_slider:
            return
        current_idx = len(self.df[self.df['t'] <= self.time_slider.val]) - 1
        self._update_status_panel(current_idx)

    def _set_lap_value(self, num_laps):
        """Set lap value with validation"""
        # Clip to valid range
        num_laps = max(1, min(num_laps, self.multilap_data.num_laps))

        if num_laps == self.num_laps_for_mean:
            return  # No change

        self.num_laps_for_mean = num_laps

        # Update textbox display
        if self.lap_textbox:
            self.lap_textbox.set_val(str(num_laps))

        # Rebuild mean course and downstream
        self._rebuild_mean_course()

        # Update time slider max (filter to selected laps)
        if num_laps < self.multilap_data.num_laps:
            max_idx = self.multilap_data.lap_boundaries[num_laps - 1].end_index
            max_time = self.df.iloc[max_idx]['t']
            self.time_slider.valmax = max_time
            self.time_slider.ax.set_xlim(
                self.time_slider.valmin, self.time_slider.valmax)

        # Refresh visualizations
        self._refresh_visualizations()

    def _on_segment_method_changed(self, label):
        """Handle segment method radio button change"""
        # Map label to method name
        method_map = {
            'Threshold': 'threshold',
            'Extrema': 'extrema',
            'Gradient': 'gradient',
            'Hybrid': 'hybrid'
        }
        new_method = method_map[label]

        if new_method == self.segment_method:
            return  # No change

        self.segment_method = new_method

        # Rebuild segmentation
        self._rebuild_segmentation()

        # Refresh visualizations
        self._refresh_visualizations()

    def _on_stats_field_changed(self, label):
        """Handle stats field radio button change."""
        # Map display label back to key
        label_to_key = {
            key.replace('_', ' ').title(): key
            for key in self.available_ranking_keys
        }
        new_field = label_to_key.get(label)

        if not new_field or new_field == self.current_stats_field:
            return

        self.current_stats_field = new_field

        # Update status panel immediately
        current_idx = len(self.df[self.df['t'] <= self.time_slider.val]) - 1
        self._update_status_panel(current_idx)

        self.fig.canvas.draw_idle()

    def _on_keyboard_press(self, event):
        """Handle keyboard navigation"""
        if event.key == 'left':
            # Find previous timestamp
            current_val = self.time_slider.val
            mask = self.df['t'] < current_val
            if mask.any():
                new_val = self.df[mask]['t'].iloc[-1]
                new_val = max(new_val, self.time_slider.valmin)
                self.time_slider.set_val(new_val)
        elif event.key == 'right':
            # Find next timestamp
            current_val = self.time_slider.val
            mask = self.df['t'] > current_val
            if mask.any():
                new_val = self.df[mask]['t'].iloc[0]
                new_val = min(new_val, self.time_slider.valmax)
                self.time_slider.set_val(new_val)

    def show(self):
        """Display the interactive plot"""
        plt.show()
