# Web-Based IMU Path Visualizer

This document describes the web-based IMU path visualization feature for
Donkeycar.

## Overview

The web-based IMU path visualizer provides an alternative to the
matplotlib-based UI, offering browser-based interactive visualization of
vehicle trajectories. It reuses the existing Donkey tornado web server
infrastructure and the same course analysis stack as the matplotlib UI.

## Features

- **Interactive path visualization** using Plotly.js
- **Speed-colored path scatter** with colorbar legend
- **Mean course polyline** with segment boundary markers
- **Real-time controls**:
  - Time slider with play/pause
  - Lap selector dropdown
  - Segment method selector
  - Display toggles (driven path, mean course)
- **Info panels**:
  - Current position (date/time, lap, segment, lap distance, segment rank,
    speed, heading, coordinates)
  - Dataset metadata (laps, points, duration, distance, segments)
- **Responsive layout** keeps the current position panel visible while the
  plot resizes with the browser window and values stay right-aligned
- **Segment statistics** (for Tub data with configurable aggregations)
  - Stats dropdown includes all supported aggregations for configured
    stats fields

## Usage

### Basic Usage

```bash
# Default matplotlib UI (unchanged)
donkey imupath ./recording.csv
donkey imupath ./tub_directory

# Web-based UI
donkey imupath --web ./recording.csv
donkey imupath --web ./tub_directory
```

### With Options

```bash
# Web UI with lap detection method
donkey imupath --web --lap-method drift ./data.csv

# Web UI with segmentation method
donkey imupath --web --segment-method hybrid ./tub_directory

# Web UI with custom config (for segment stats)
donkey imupath --web --config ./mycar/config.py ./tub_directory
```

### Accessing the Web UI

When using `--web`, the server starts on port 8887 (or configured
`WEB_CONTROL_PORT`):

```
Web UI available at:
  http://localhost:8887/imupath
  http://<hostname>.local:8887/imupath
```

Press Ctrl+C to stop the server.

## Architecture

### Backend Components

1. **`donkeycar/web/imupath_data.py`**
   - `IMUPathDataBuilder`: Builds JSON-ready data payloads
   - `prepare_imupath_data()`: Convenience function for data preparation
   - Reuses course_analysis stack: lap detection, mean course, segmentation

2. **`donkeycar/parts/web_controller/web.py`**
   - `IMUPathHandler`: Serves the HTML page
   - `IMUPathDataAPI`: JSON API endpoint (`/api/imupath/data`)
   - Extends `LocalWebController` with `imupath_builder` storage

3. **`donkeycar/management/imupath.py`**
   - Added `--web` flag to CLI
   - `_run_web_mode()`: Initializes web server with IMU data

### Frontend Components

1. **`templates/imupath.html`**
   - HTML structure with Bootstrap styling
   - Dark theme matching Donkey UI
   - Control panel and info panels
   - Plotly.js integration

2. **`static/imupath.js`**
   - Loads data via AJAX from `/api/imupath/data`
   - Renders interactive Plotly visualization
   - Handles UI controls (slider, dropdowns, toggles)
   - Play/pause animation support

### Data Flow

```
CSV/Tub → PathData → IMUPathDataBuilder → JSON → Browser
                          ↓
                  MultiLapData
                  MeanCourse
                  Segmentation
                  SegmentAssigner
```

### JSON API

**Endpoint:** `GET /api/imupath/data`

**Query Parameters:**
- `num_laps`: Number of laps for mean course (optional)
- `segment_method`: Segmentation method (optional)
- `max_display_points`: Maximum points for display (default: 1000)

**Response:**
```json
{
  "path_points": [
    {"t": 0.0, "x": 0.0, "y": 0.0, "v": 0.5, "h": 0.0, "lap": 0, "segment": 0},
    ...
  ],
  "mean_course": [
    {"x": 0.0, "y": 0.0},
    ...
  ],
  "segments": [
    {"id": 0, "start_idx": 0, "end_idx": 50,
     "label": "Seg 0", "type": "STRAIGHT"},
    ...
  ],
  "metadata": {
    "lap_method": "y_crossing",
    "segment_method": "gradient",
    "num_laps": 3,
    "total_laps": 3,
    "total_points": 1500,
    "display_points": 1000,
    "num_segments": 5,
    "mean_course_length": 25.5,
    "total_distance": 75.0,
    "duration": 15.0,
    "is_tub_data": false,
    "available_stats": []
  },
  "rankings": {
    "available": false,
    "fields": []
  }
}
```

## Configuration

### Downsampling

The visualizer respects `IMU_VISUALIZATION_PARAMS` from the config:

```python
# In config.py
IMU_VISUALIZATION_PARAMS = {
    'max_display_points': 1000,  # Used for downsampling display
    'update_throttle_ms': 100,   # (Not used by web UI)
}
```

Points are uniformly downsampled for display while full data is maintained
for statistics.

### Segment Statistics

For Tub data, segment statistics use `FIELD_AGGREGATIONS` from the config:

```python
# In config.py
FIELD_AGGREGATIONS = [
    FieldSpec('time', lambda r: r['_timestamp_ms'], 'sum', 'time'),
    FieldSpec('gyro_z', lambda r: abs(r['imu/gyro_z']), 'sum', 'gyro_z'),
    ...
]
```

Available stats fields are shown in the UI dropdown (when Tub data loaded).

## Comparison: Matplotlib vs Web UI

| Feature | Matplotlib UI | Web UI |
|---------|---------------|--------|
| Launch | `donkey imupath` | `donkey imupath --web` |
| Platform | Desktop only | Any browser |
| Dependencies | matplotlib, tkinter | Plotly.js (CDN) |
| Interaction | Mouse, keyboard | Mouse, touch |
| Performance | Good | Good |
| Customization | Limited | CSS/JS editable |
| Remote Access | No | Yes (via network) |
| Computation | Same (course_analysis) | Same (course_analysis) |

## Implementation Notes

### Backward Compatibility

- Default behavior unchanged: `donkey imupath` still launches matplotlib UI
- `--web` flag is opt-in
- All existing CLI options work with both UIs

### Reused Components

- Course analysis stack (lap detection, mean course, segmentation)
- Tornado web server (LocalWebController)
- Config loading (load_config)
- IMU_VISUALIZATION_PARAMS (downsampling)

### Additive Design

- No modifications to existing matplotlib UI code
- New module: `donkeycar/web/`
- New endpoints: `/imupath`, `/api/imupath/data`
- New templates: `imupath.html`, `imupath.js`

## Testing

### Manual Testing

```bash
# Create test data
python donkeycar/tests/test_imupath_web_manual.py

# Test default behavior (matplotlib)
donkey imupath <test_file>

# Test web UI
donkey imupath --web <test_file>
```

### Automated Tests

```bash
# Run unit tests (requires pytest)
pytest donkeycar/tests/test_imupath_web.py
```

## Troubleshooting

### Server Won't Start

- Check port 8887 is not already in use
- Try custom port: Set `WEB_CONTROL_PORT` in config

### Data Not Loading

- Check browser console for errors
- Verify data source path is correct
- Check server logs for errors

### Visualization Issues

- Clear browser cache
- Check Plotly.js CDN is accessible
- Verify JSON data structure in `/api/imupath/data`

## Future Enhancements

- WebSocket support for real-time updates
- Export to video/GIF
- Multi-session comparison
- Custom colormap selection
- Segment ranking visualization
- Touch gesture controls for mobile

## Credits

Built on:
- Donkeycar course_analysis API
- Plotly.js for visualization
- Tornado web server
- Bootstrap for styling
