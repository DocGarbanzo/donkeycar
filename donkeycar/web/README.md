# Web-Based IMU Path Visualizer

An interactive web-based visualization tool for IMU path data using Plotly and FastAPI.

## Overview

This module provides a modern web-based alternative to the matplotlib UI for visualizing IMU path data. It includes:

- **Interactive Plotly visualization** with zoom, pan, and hover tooltips
- **Real-time controls** for navigation and analysis
- **Segment statistics** for Tub data
- **Responsive design** that works on desktop and mobile

## Usage

### Basic Usage

Start the web UI with a CSV file:
```bash
donkey imupath --web ./recording.csv
```

Start the web UI with a Tub directory:
```bash
donkey imupath --web ./data/tub_1
```

The server will start on `http://localhost:8000` by default.

### Command-Line Options

```bash
donkey imupath --web [OPTIONS] <data_source>

Options:
  --web                     Launch web-based UI (default: matplotlib UI)
  --lap-method METHOD       Lap detection method: y_crossing or drift (default: y_crossing)
  --segment-method METHOD   Segmentation method: threshold, extrema, gradient, or hybrid (default: gradient)
  --num-laps N             Number of laps for mean course (default: all)
  --config PATH            Path to config file for segment stats (default: ./config.py)
  --port PORT              Web server port (default: 8000)
```

### Examples

Use drift lap detection and hybrid segmentation:
```bash
donkey imupath --web --lap-method drift --segment-method hybrid ./data/tub_1
```

Custom port and config:
```bash
donkey imupath --web --port 8080 --config ~/mycar/config.py ./data/tub_1
```

## Features

### Visualization

- **Path scatter plot** colored by speed (Viridis colormap)
- **Mean course line** overlay (computed from selected laps)
- **Segment boundaries** with labels
- **Current position marker** that moves with time slider

### Controls

- **Time slider**: Navigate through the recorded path
- **Play/Pause**: Automatic playback of the path
- **Lap count selector**: Change number of laps used for mean course
- **Segment method selector**: Switch segmentation algorithms
- **Display toggles**: Show/hide driven path, mean course, and segments

### Information Panels

- **Current Point**: Shows time, lap, segment, velocity, heading, and position
- **Segment Statistics**: Best-per-lap performance metrics (for Tub data)
- **Metadata**: Total laps, segments, distance, duration, and point count

### Keyboard Shortcuts

- **Left/Right arrows**: Navigate frame-by-frame
- **Space**: Toggle play/pause

## API Endpoints

The FastAPI server provides REST endpoints:

### GET /
Main visualization page (HTML UI)

### GET /api/health
Health check endpoint
```json
{
  "status": "healthy",
  "data_loaded": true
}
```

### GET /api/data
Get visualization data (path points, mean course, segments, rankings, metadata)

### GET /api/stats
Get segment statistics summary with best-per-lap metrics

### POST /api/load
Load new data source
```json
{
  "data_source": "./path/to/data.csv",
  "lap_method": "y_crossing",
  "segment_method": "gradient",
  "num_laps": null
}
```

### POST /api/update_settings
Update visualization settings without reloading file

## Data Format

The web UI accepts the same data formats as the matplotlib UI:

### CSV Format
```
t, x, y, h, v
0.0, 0.0, 0.0, 0.0, 1.0
0.1, 0.1, 0.0, 0.0, 1.0
...
```

Where:
- `t`: timestamp (seconds)
- `x`: X position (meters, right direction)
- `y`: Y position (meters, forward direction)
- `h`: heading (radians)
- `v`: velocity (m/s)

### Tub Format
Standard Donkey Car Tub v2 format with IMU data fields:
- `_timestamp_ms`: Timestamp in milliseconds
- `car/pos`: Position tuple (x, y)
- `car/euler`: Euler angles (roll, pitch, yaw)
- `car/speed`: Velocity
- `car/distance`: Cumulative distance (for segment stats)

## Configuration

The web UI respects the same configuration parameters as the matplotlib UI:

### IMU_VISUALIZATION_PARAMS
```python
IMU_VISUALIZATION_PARAMS = {
    'max_display_points': 1000,  # Downsample to this many points for display
    'update_throttle_ms': 100,   # Not used in web UI (kept for compatibility)
}
```

### Segment Statistics (Tub data only)
```python
FIELD_AGGREGATIONS = [
    {
        'field': 'car/gyro',
        'index': 2,
        'output_key': 'gyro_z_agg',
        'transform': abs_transform,
        'aggregation': 'avg'
    }
]

LAP_SORTING_CRITERIA = [
    {'key': 'time'},
    {'key': 'distance'},
    {'key': 'gyro_z_agg'},
]
```

## Technical Details

### Architecture

The web UI consists of three main components:

1. **Data Preparation** (`imupath_data.py`): Reuses the existing course_analysis pipeline to generate JSON-ready data
2. **FastAPI Service** (`imupath_api.py`): REST API server with data endpoints
3. **Frontend** (HTML/JS/CSS): Plotly-based interactive visualization

### Performance

- **Downsampling**: Display points are downsampled to `max_display_points` (default: 1000) for smooth rendering
- **Full data retention**: All points are kept in memory for accurate statistics
- **Efficient updates**: Only the current position marker is updated during playback

### Browser Compatibility

The web UI works on all modern browsers:
- Chrome/Edge 90+
- Firefox 88+
- Safari 14+

## Troubleshooting

### "ModuleNotFoundError: No module named 'fastapi'"
Install the web UI dependencies:
```bash
pip install fastapi uvicorn
```

### Port already in use
Use a different port:
```bash
donkey imupath --web --port 8001 ./data.csv
```

### Segment statistics not showing
Make sure:
1. You're loading Tub data (not CSV)
2. The Tub has IMU fields (car/pos, car/distance, etc.)
3. Lap detection is finding laps (check console output)

## Comparison with Matplotlib UI

| Feature | Matplotlib UI | Web UI |
|---------|---------------|--------|
| Interactive controls | ✓ | ✓ |
| Time slider | ✓ | ✓ |
| Segment statistics | ✓ | ✓ |
| Zoom/pan | Limited | ✓ Plotly native |
| Responsive | Desktop only | Desktop + mobile |
| Remote access | No | Yes (via network) |
| Installation | Matplotlib | FastAPI + uvicorn |

## Development

### Running Tests
```bash
python -m pytest donkeycar/tests/test_web_imupath.py -v
```

### Starting the Server Programmatically
```python
from donkeycar.web.imupath_api import run_web_ui

run_web_ui(
    data_source='./data.csv',
    cfg=None,
    lap_method='y_crossing',
    segment_method='gradient',
    port=8000
)
```

## License

Same as Donkey Car - MIT License
