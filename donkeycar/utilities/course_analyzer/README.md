# Course Analyzer Utility

Command-line tool for analyzing multi-lap course data to create mean reference courses, automatically segment them into geometric features, and enable real-time segment detection.

## Features

- **Multi-lap data loading**: Automatically detects and separates individual laps from continuous GPS/odometry data
- **Mean course reconstruction**: Aligns laps, removes outliers, and computes average trajectory
- **Auto-segmentation**: Identifies straights, turns, S-curves, and chicanes
- **Real-time segment estimation**: Determines vehicle's current segment from position and heading

## Usage

### Basic Analysis

```bash
python analyzer.py laps.csv
```

This will:
1. Load multi-lap data from `laps.csv`
2. Compute the mean course
3. Segment the course
4. Save results to `./course_output/`

### Advanced Options

```bash
python analyzer.py laps.csv \
    --output-dir ./my_output \
    --lap-threshold 3.0 \
    --resampling 0.2 \
    --curvature-threshold 0.08
```

### Command-Line Options

**Input/Output:**
- `input_csv`: Input CSV file with multi-lap data (required)
- `--output-dir, -o`: Output directory (default: `./course_output`)
- `--format`: Output format: `json`, `csv`, or `both` (default: `both`)

**Lap Detection:**
- `--lap-threshold`: Distance threshold for lap detection in meters (default: 2.0)
- `--min-lap-points`: Minimum points per lap (default: 50)

**Mean Course:**
- `--resampling`: Resampling interval in meters (default: 0.1)
- `--outlier-threshold`: Outlier detection threshold in std deviations (default: 2.5)
- `--smoothing-window`: Position smoothing window size (default: 11)

**Segmentation:**
- `--curvature-threshold`: Straight segment curvature threshold in rad/m (default: 0.05)
- `--min-segment-length`: Minimum segment length in meters (default: 0.5)

**Control:**
- `--no-mean-course`: Skip saving mean course
- `--no-segmentation`: Skip segmentation

## Input CSV Format

The input CSV must contain the following columns:

| Column | Description | Units |
|--------|-------------|-------|
| `timestamp` | Time | seconds |
| `x` | X coordinate | meters |
| `y` | Y coordinate | meters |
| `heading` | Vehicle heading (0°=forward, positive=left) | degrees |

Example:
```csv
timestamp,x,y,heading
0.0,0.0,0.0,0.0
0.05,0.1,0.01,2.0
0.10,0.2,0.02,5.0
...
```

## Output Files

### Mean Course (`mean_course.json` or `mean_course.csv`)

Contains the reconstructed mean course with:
- `x`, `y`: Position coordinates
- `heading`: Heading angles
- `distance`: Cumulative arc-length distance
- Metadata with processing parameters and statistics

### Segmentation (`segmentation.json`)

Contains segment information:
- Total number of segments
- Segment counts by type
- Detailed properties for each segment:
  - Segment ID
  - Type (straight, left_turn, right_turn, s_curve_lr, s_curve_rl, chicane)
  - Length
  - Curvature statistics
  - Heading changes

## Python API

You can also use the analyzer programmatically:

```python
from donkeycar.parts.course_analysis import (
    MultiLapData, MeanCourse, CourseSegmentation, SegmentEstimator
)

# Load data
data = MultiLapData()
data.load_csv('laps.csv')

# Compute mean course
mean_course = MeanCourse(data)
mean_course.compute()

# Segment course
segmentation = CourseSegmentation(mean_course)
segmentation.compute()

# Create estimator
estimator = SegmentEstimator(segmentation)

# Estimate segment
estimate = estimator.estimate(x=10.0, y=5.0, heading=45.0)
print(f"Segment: {estimate.segment_id}, Confidence: {estimate.confidence}")
```

## Examples

### Example 1: Analyze a race track

```bash
python analyzer.py race_track_laps.csv --output-dir race_analysis
```

Output:
```
Detected 5 laps
Course length: 487.32 meters
Total segments: 12
  straight       : 4
  left_turn      : 3
  right_turn     : 3
  s_curve_lr     : 2
```

### Example 2: Fine-tune parameters for noisy GPS data

```bash
python analyzer.py noisy_gps.csv \
    --lap-threshold 5.0 \
    --outlier-threshold 3.0 \
    --smoothing-window 21
```

### Example 3: Quick segmentation only

```bash
python analyzer.py laps.csv \
    --no-mean-course \
    --format json
```

## Troubleshooting

**"No laps detected"**
- Increase `--lap-threshold` if your course is large or GPS is noisy
- Decrease `--min-lap-points` if you have sparse data

**"Course is not smooth"**
- Increase `--smoothing-window` for more aggressive smoothing
- Increase `--outlier-threshold` to remove more outliers

**"Too many/few segments"**
- Adjust `--curvature-threshold` (higher = fewer segments)
- Adjust `--min-segment-length` (higher = fewer, longer segments)

## See Also

- [Course Analysis Specification](../../../docs/course_analysis_segmentation_spec.md)
- [Test Suite](../../tests/test_course_analysis.py)
- [Main Module](../../parts/course_analysis.py)
