"""
Interactive IMU Path Visualization

This module provides an interactive visualization for IMU path analysis using
the refactored course_analysis API. It includes widgets for real-time
navigation, lap filtering, and segmentation method selection.

Classes:
    InteractiveIMUVisualizer: Main class for interactive visualization
"""

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import (
    Slider, CheckButtons, RadioButtons, Button, TextBox
)
from matplotlib.lines import Line2D
import logging

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
                 segment_method='gradient', file_path=''):
        """
        Initialize the interactive visualizer.

        Args:
            path_data: PathData object with immutable position/velocity data
            cfg: Configuration object with parameters
            lap_method: Initial lap detection method ('y_crossing' or 'drift')
            segment_method: Initial segmentation method
                ('threshold', 'extrema', 'gradient', 'hybrid')
            file_path: Path to source data file (for display)
        """
        # Store immutable data
        self.path_data = path_data
        self.cfg = cfg
        self.file_path = file_path

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
        self.legend = None

        # Widgets (will be created in setup_ui)
        self.time_slider = None
        self.display_toggles = None
        self.lap_textbox = None
        self.lap_minus_button = None
        self.lap_plus_button = None
        self.segment_radio = None

        # Status text elements (will be created in setup_ui)
        self.status_texts = {}

        # Performance tracking
        self.last_update_time = 0
        self.throttle_interval = 0.1  # 100ms minimum between updates

        # Initialize data processing pipeline
        self._initialize_data_pipeline()

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

    def setup_ui(self):
        """
        Create all matplotlib widgets and set up event handlers.
        """
        # Create figure with single axis
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.canvas.manager.set_window_title('Donkey imupath2')

        # Adjust layout to make room for widgets
        plt.subplots_adjust(bottom=0.2, right=0.95, top=0.87, left=0.08)

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

        # Create legend
        self._create_legend()

    def _create_plot_artists(self):
        """Create all plot artists (scatter, markers, lines, etc.)"""
        # Full path scatter plot (velocity-colored, faded)
        self.full_path_scatter = self.ax.scatter(
            self.df['x'], self.df['y'],
            c=self.df['v'],
            cmap='viridis',
            s=8,
            alpha=0.3,
            label='Full path'
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
        """Create status panel with 11 text fields"""
        # Helper to create styled text
        def create_text(y_pos, initial_text='', color='white'):
            return self.fig.text(
                0.02, y_pos, initial_text,
                transform=self.fig.transFigure,
                fontsize=9,
                color=color,
                bbox=dict(boxstyle='round', facecolor='black', alpha=0.8)
            )

        # Create all status text elements
        self.status_texts = {
            'file': create_text(0.95, f'File: {self.file_path}'),
            'speed': create_text(0.915, 'Speed: 0.00 m/s'),
            'time': create_text(0.88, 'Time: --'),
            'position': create_text(0.845, 'Position: [0.00, 0.00]'),
            'lap': create_text(0.81, 'Lap: 1'),
            'total_dist': create_text(0.775, 'Total distance: 0.00m'),
            'lap_dist': create_text(0.74, 'Lap distance: 0.00m'),
            'segment': create_text(0.705, 'Segment: --'),
            'debug': create_text(0.67, 'Debug: idx=0, dist=0.00m'),
            'controls': create_text(
                0.635, 'Controls: ← → arrows to navigate', color='yellow')
        }

    def _create_widgets(self):
        """Create all interactive widgets"""
        # Time slider
        ax_slider = plt.axes([0.02, 0.05, 0.96, 0.03])
        self.time_slider = Slider(
            ax_slider, '', self.df['t'].min(), self.df['t'].max(),
            valinit=self.df['t'].min(), valfmt=''
        )
        self.time_slider.on_changed(self._on_time_slider_changed)

        # Display toggles
        ax_toggle = plt.axes([0.02, 0.27, 0.12, 0.045])
        toggle_labels = ['Driven Path', 'Mean Course']
        toggle_actives = [True, True]
        self.display_toggles = CheckButtons(
            ax_toggle, toggle_labels, toggle_actives)
        self.display_toggles.on_clicked(self._on_display_toggle)

        # Lap selector (only if multiple laps detected)
        if self.multilap_data.num_laps > 1:
            # Label
            self.fig.text(0.02, 0.20, 'Laps',
                         transform=self.fig.transFigure, fontsize=10)

            # TextBox
            ax_textbox = plt.axes([0.02, 0.155, 0.065, 0.035])
            self.lap_textbox = TextBox(
                ax_textbox, '', initial=str(self.num_laps_for_mean))
            self.lap_textbox.on_submit(self._on_lap_text_submit)

            # Minus button
            ax_minus = plt.axes([0.02, 0.11, 0.03, 0.03])
            self.lap_minus_button = Button(
                ax_minus, '−', color='#1a1a1a', hovercolor='#333333')
            self.lap_minus_button.on_clicked(self._on_lap_minus)

            # Plus button
            ax_plus = plt.axes([0.06, 0.11, 0.03, 0.03])
            self.lap_plus_button = Button(
                ax_plus, '+', color='#1a1a1a', hovercolor='#333333')
            self.lap_plus_button.on_clicked(self._on_lap_plus)

        # Segment method selector
        ax_radio = plt.axes([0.02, 0.33, 0.18, 0.11], facecolor='#1a1a1a')
        self.fig.text(0.02, 0.45, 'Segment Method',
                     transform=self.fig.transFigure, fontsize=10)
        radio_labels = ['Threshold', 'Extrema', 'Gradient', 'Hybrid']
        active_idx = {'threshold': 0, 'extrema': 1,
                     'gradient': 2, 'hybrid': 3}[self.segment_method]
        self.segment_radio = RadioButtons(
            ax_radio, radio_labels, active=active_idx)
        self.segment_radio.on_clicked(self._on_segment_method_changed)

        # Keyboard navigation
        self.fig.canvas.mpl_connect('key_press_event',
                                    self._on_keyboard_press)

    def _create_legend(self):
        """Create legend with custom handles"""
        # Create custom legend handles
        full_path_legend = Line2D([0], [0], marker='o', color='w',
                                  markerfacecolor='gray', markersize=8,
                                  linestyle='', alpha=0.5,
                                  label='Full path')
        current_pos = Line2D([0], [0], marker='o', color='w',
                            markerfacecolor='#FF6B6B', markersize=10,
                            linestyle='', label='Current position')
        current_path = Line2D([0], [0], color='#FF6B6B', linewidth=2,
                             label='Current path')

        legend_handles = [full_path_legend, current_pos, current_path]

        # Add mean course to legend if available
        if self.mean_course_line is not None:
            legend_handles.append(self.mean_course_line)

        # Add segment boundaries if available
        if (self.segmentation is not None and
                self.segmentation.num_segments > 0):
            boundary_handle = Line2D(
                [0], [0],
                color=MEAN_COURSE_COLOR,
                linewidth=2.5,
                label=f'Segment boundaries '
                      f'({self.segmentation.num_segments})'
            )
            legend_handles.append(boundary_handle)

        # Create legend
        self.legend = self.ax.legend(
            handles=legend_handles,
            loc='upper left',
            bbox_to_anchor=(0.02, 0.60),
            framealpha=0.9
        )

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
            color='white',
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
        """Update all status text fields"""
        if current_idx < 0 or current_idx >= len(self.df):
            return

        # Get current data point
        row = self.df.iloc[current_idx]

        # Update speed
        self.status_texts['speed'].set_text(f'Speed: {row["v"]:.2f} m/s')

        # Update time
        from datetime import datetime, timedelta
        time_str = str(timedelta(seconds=int(row['t'])))
        self.status_texts['time'].set_text(f'Time: {time_str}')

        # Update position
        self.status_texts['position'].set_text(
            f'Position: [{row["x"]:.2f}, {row["y"]:.2f}]')

        # Calculate current lap
        current_lap = 1
        for i, boundary in enumerate(self.multilap_data.lap_boundaries):
            if current_idx <= boundary.end_index:
                current_lap = i + 1
                break

        self.status_texts['lap'].set_text(f'Lap: {current_lap}')

        # Calculate total distance (cumulative)
        if current_idx > 0:
            dx = np.diff(self.df['x'][:current_idx + 1])
            dy = np.diff(self.df['y'][:current_idx + 1])
            total_dist = np.sum(np.sqrt(dx**2 + dy**2))
        else:
            total_dist = 0.0

        self.status_texts['total_dist'].set_text(
            f'Total distance: {total_dist:.2f}m')

        # Calculate lap distance
        lap_start_idx = (self.multilap_data.lap_boundaries[current_lap - 2]
                        .end_index + 1 if current_lap > 1 else 0)
        if current_idx > lap_start_idx:
            dx = np.diff(self.df['x'][lap_start_idx:current_idx + 1])
            dy = np.diff(self.df['y'][lap_start_idx:current_idx + 1])
            lap_dist = np.sum(np.sqrt(dx**2 + dy**2))
        else:
            lap_dist = 0.0

        self.status_texts['lap_dist'].set_text(
            f'Lap distance: {lap_dist:.2f}m')

        # Get current segment
        if self.segment_ids is not None and current_idx < len(
                self.segment_ids):
            seg_id = self.segment_ids[current_idx]
            seg_type = (self.segmentation.segments[seg_id].segment_type.name
                       if seg_id < len(self.segmentation.segments) else '--')
            self.status_texts['segment'].set_text(
                f'Segment: {seg_id} ({seg_type})')
        else:
            self.status_texts['segment'].set_text('Segment: --')

        # Debug info
        dist_to_origin = np.sqrt(row['x']**2 + row['y']**2)
        self.status_texts['debug'].set_text(
            f'Debug: idx={current_idx}, dist={dist_to_origin:.2f}m')

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
        self._create_legend()
        self.fig.canvas.draw_idle()

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
