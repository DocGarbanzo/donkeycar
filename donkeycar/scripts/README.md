# DonkeyCar Scripts

Standalone utility scripts for data analysis and visualization.

## Available Scripts

1. **IMU Path Visualizer** - Visualize and analyze IMU path data
2. **Course Analyzer** - Multi-lap course analysis and segmentation

---

## IMU Path Visualizer

Interactive visualization tool for IMU path data with drift correction.

### Features

- **Multi-source support**: Load from CSV files or Tub directories
- **Interactive timeline**: Slider to navigate through recorded path
- **Drift correction**: Automatic loop detection and drift removal
- **Real-time display**: Shows position, speed, heading, and distance

### Usage

**From CSV:**
```bash
python imu_path_visualizer.py imu.csv
```

**From Tub:**
```bash
python imu_path_visualizer.py ~/mycar/data/tub_1_25-01-12
```

**With drift correction:**
```bash
python imu_path_visualizer.py imu.csv --drift-correction
```

### Command-Line Options

- `data_source` - Path to CSV file or Tub directory (default: `imu.csv`)
- `--drift-correction` - Enable loop drift correction
- `--min-loop-distance` - Minimum distance to travel before loop (default: 1.0m)
- `--max-distance` - Max distance from origin for loop detection (default: 0.5m)
- `--downsample-factor` - Display downsample factor (default: auto)

### Input Data Format

**CSV Format:**
- Columns: `t` (time), `x`, `y`, `h` (heading), `v` (velocity)
- Heading in degrees
- Example: `0.0,0.0,0.0,0.0,1.5`

**Tub Format:**
- Automatically extracts from Tub records:
  - `_timestamp_ms` → time
  - `car/pos` → x, y coordinates
  - `car/euler` → heading (derived from euler[2]: 90° - euler[2])
  - `car/speed` → velocity

### Controls

- **Left/Right arrows**: Navigate through timeline
- **Slider**: Jump to specific time
- **Interactive plot**: Zoom and pan

### Python API

```python
from donkeycar.scripts.imu_path_visualizer import visualize_imu_path

# From CSV
visualize_imu_path('imu.csv', correct_drift=True)

# From Tub
visualize_imu_path('~/mycar/data/tub_1_25-01-12')
```

---

## Course Analyzer

Analyzes multi-lap course data to create mean reference courses,
automatically segment them into geometric features, and enable
real-time segment detection.

### Features

- **Multi-source loading**: Load from CSV files or Tub directories
- **Multi-lap detection**: Automatically detects and separates laps
- **Mean course reconstruction**: Aligns laps, removes outliers
- **Auto-segmentation**: Identifies straights, turns, S-curves, chicanes
- **Real-time estimation**: Determines current segment from position/heading

### Usage

**From CSV:**
```bash
python course_analyzer.py laps.csv
```

**From Tub:**
```bash
python course_analyzer.py ~/mycar/data/tub_1_25-01-12
```

**With custom parameters:**
```bash
python course_analyzer.py laps.csv \
    --output-dir ./my_output \
    --lap-threshold 3.0 \
    --curvature-threshold 0.08
```

### Command-Line Options

**Input/Output:**
- `data_source` - CSV file or Tub directory (required)
- `--output-dir, -o` - Output directory (default: `./course_output`)
- `--format` - Output format: `json`, `csv`, or `both` (default: `both`)

**Lap Detection:**
- `--lap-threshold` - Distance threshold for lap detection (default: 2.0m)
- `--min-lap-points` - Minimum points per lap (default: 50)

**Mean Course:**
- `--resampling` - Resampling interval in meters (default: 0.1)
- `--outlier-threshold` - Outlier detection threshold in std (default: 2.5)
- `--smoothing-window` - Position smoothing window (default: 11)

**Segmentation:**
- `--curvature-threshold` - Straight threshold in rad/m (default: 0.05)
- `--min-segment-length` - Minimum segment length (default: 0.5m)

**Control:**
- `--no-mean-course` - Skip saving mean course
- `--no-segmentation` - Skip segmentation

### Input Data Format

**CSV Format:**
```csv
timestamp,x,y,heading
0.0,0.0,0.0,0.0
0.05,0.1,0.01,0.035
0.10,0.2,0.02,0.087
...
```
- `timestamp` - Time in seconds
- `x` - X coordinate (forward) in meters
- `y` - Y coordinate (left) in meters
- `heading` - Vehicle heading in radians

**Tub Format:**
Extracts from Tub records:
- `_timestamp_ms` → timestamp (converted to seconds)
- `car/pos` → x, y coordinates
- `car/euler` → heading (derived from euler[2]: 90° - euler[2], then converted to radians)

### Output Files

**Mean Course** (`mean_course.json` or `mean_course.csv`):
- Position coordinates (x, y)
- Heading angles (radians)
- Cumulative arc-length distance
- Processing metadata and statistics

**Segmentation** (`segmentation.json`):
- Total number of segments
- Segment counts by type
- Detailed properties for each segment:
  - Segment ID and type
  - Length, curvature statistics
  - Heading changes
  - Entry/exit headings

### Segment Types

1. **STRAIGHT** - Minimal curvature, constant heading
2. **LEFT_TURN** - Positive curvature, heading increasing
3. **RIGHT_TURN** - Negative curvature, heading decreasing
4. **S_CURVE_LR** - Left turn followed by right turn
5. **S_CURVE_RL** - Right turn followed by left turn
6. **CHICANE** - Rapid alternating turns

### Python API

```python
from donkeycar.parts.course_analysis import (
    MultiLapData, MeanCourse, CourseSegmentation, SegmentEstimator
)

# Load from CSV or Tub
data = MultiLapData()
data.load_data('~/mycar/data/tub_1_25-01-12')

# Compute mean course
mean_course = MeanCourse(data)
mean_course.compute()

# Segment course
segmentation = CourseSegmentation(mean_course)
segmentation.compute()

# Create estimator for real-time use
estimator = SegmentEstimator(segmentation)

# Estimate segment (heading in radians)
estimate = estimator.estimate(x=10.0, y=5.0, heading=0.785)
print(f"Segment: {estimate.segment_id}, "
      f"Confidence: {estimate.confidence}")
```

### Example Output

```
Detected 5 laps
Course length: 487.32 meters
Total segments: 12
  straight       : 4
  left_turn      : 3
  right_turn     : 3
  s_curve_lr     : 2
```

### Troubleshooting

**"No laps detected"**
- Increase `--lap-threshold` if course is large or GPS is noisy
- Decrease `--min-lap-points` if you have sparse data

**"Course is not smooth"**
- Increase `--smoothing-window` for more aggressive smoothing
- Increase `--outlier-threshold` to remove more outliers

**"Too many/few segments"**
- Adjust `--curvature-threshold` (higher = fewer segments)
- Adjust `--min-segment-length` (higher = fewer, longer segments)

---

## Common Workflows

### Record and Visualize IMU Path

1. **Record path data**:
   ```bash
   # On the car, run with record_path=True
   cd ~/mycar
   ./manage.py drive --record_path
   ```

2. **Visualize from Tub**:
   ```bash
   python imu_path_visualizer.py ~/mycar/data/tub_1_25-01-12 \
       --drift-correction
   ```

### Analyze Multi-Lap Course

1. **Drive multiple laps** and record to Tub

2. **Analyze course**:
   ```bash
   python course_analyzer.py ~/mycar/data/tub_1_25-01-12 \
       --output-dir ~/course_analysis
   ```

3. **Review results**:
   - `course_output/mean_course.json` - Reference trajectory
   - `course_output/segmentation.json` - Identified segments

### Export CSV for External Analysis

Both scripts can work with CSV files for use in external tools:

```python
# Export Tub to CSV
from donkeycar.parts.tub_v2 import Tub
import pandas as pd

tub = Tub('~/mycar/data/tub_1_25-01-12', read_only=True)
rows = []
for record in tub:
    t = record['_timestamp_ms'] / 1000.0
    pos = record['car/pos']
    euler = record['car/euler']
    heading = math.radians(90.0 - euler[2])  # Convert euler to heading
    speed = record['car/speed']
    rows.append({
        'timestamp': t,
        'x': pos[0],
        'y': pos[1],
        'heading': heading,
        'speed': speed
    })
tub.close()

df = pd.DataFrame(rows)
df.to_csv('exported_path.csv', index=False)
```

---

## See Also

- [Course Analysis Module](../parts/course_analysis.py) - Core analysis library
- [IMU Parts](../parts/imu.py) - IMU sensor integration
- [Tub Format](../parts/tub_v2.py) - Data storage format
