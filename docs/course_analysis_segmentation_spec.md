# Course Analysis and Segmentation Specification

## Overview

This specification describes a system for analyzing multi-lap course data to create a mean reference course, automatically segment it into distinct geometric features, and provide real-time segment detection for vehicles traversing the course.

## 1. Purpose and Goals

### Primary Goals
1. **Mean Course Reconstruction**: Create an accurate reference course from noisy multi-lap GPS/odometry data
2. **Auto-Segmentation**: Identify and classify course segments (straights, bends, s-curves, etc.)
3. **Real-Time Segment Estimation**: Determine which segment a vehicle is currently in based on position and heading

### Use Cases
- Course difficulty analysis and characterization
- Speed profile optimization for different segment types
- Telemetry analysis and lap comparison
- Autonomous racing and path planning
- Driver training and performance feedback

## 2. Input Data Format

### CSV File Structure
The input CSV file contains multi-lap course data with the following columns:

| Column | Type | Description | Units |
|--------|------|-------------|-------|
| `timestamp` | float | Time in seconds since epoch or relative time | seconds |
| `x` | float | X coordinate in local frame | meters |
| `y` | float | Y coordinate in local frame | meters |
| `heading` | float | Vehicle heading angle | degrees |

### Heading Convention
- **0°**: Straight ahead (vehicle's forward direction)
- **Positive angles (1-180°)**: Left turn (counterclockwise)
- **Negative angles (-1 to -180°) or (181-359°)**: Right turn (clockwise)
- Examples:
  - 10° = 10 degrees to the left
  - 350° = 10 degrees to the right
  - 90° = hard left turn
  - 270° = hard right turn

### Data Characteristics
- **Multiple laps**: Data contains N complete laps of the same course
- **Noisy measurements**: GPS/odometry drift, sensor noise, quantization error
- **Path variation**: Vehicle does not follow exact same trajectory each lap
- **Temporal sampling**: Data sampled at approximately constant frequency (e.g., 10-20 Hz)

### Example CSV
```csv
timestamp,x,y,heading
0.0,0.0,0.0,0.0
0.05,0.1,0.01,2.0
0.10,0.2,0.02,5.0
...
```

## 3. Mean Course Reconstruction

### 3.1 Requirements

#### R1.1: Lap Detection
- **ID**: R1.1
- **Description**: Automatically detect and separate individual laps from continuous multi-lap data
- **Algorithm**: Detect when vehicle returns near starting position (within threshold distance)
- **Parameters**:
  - `lap_detection_threshold`: Distance threshold for lap closure detection (default: 2.0 meters)
  - `min_lap_length`: Minimum number of points per lap (default: 50 points)

#### R1.2: Lap Alignment
- **ID**: R1.2
- **Description**: Align all laps to a common reference frame and temporal parameterization
- **Method**:
  - Translate all laps to common start position
  - Rotate to common start heading
  - Resample to common arc-length parameterization
- **Parameters**:
  - `resampling_interval`: Distance between resampled points (default: 0.1 meters)

#### R1.3: Outlier Rejection
- **ID**: R1.3
- **Description**: Remove outlier points that deviate significantly from the mean trajectory
- **Method**:
  - Compute point-wise statistics across laps
  - Remove points beyond N standard deviations from mean
  - Iterative process (2-3 iterations)
- **Parameters**:
  - `outlier_std_threshold`: Standard deviation multiplier for outlier detection (default: 2.5)
  - `outlier_iterations`: Number of outlier removal iterations (default: 2)

#### R1.4: Mean Course Calculation
- **ID**: R1.4
- **Description**: Compute the mean course from aligned, cleaned lap data
- **Method**: Point-wise averaging of x, y coordinates and heading angles
- **Output**: Single reference course with smoothed (x, y, heading) trajectory
- **Parameters**:
  - `heading_smoothing_window`: Window size for heading angle smoothing (default: 5 points)

#### R1.5: Course Smoothing
- **ID**: R1.5
- **Description**: Apply smoothing to reduce high-frequency noise in mean course
- **Method**:
  - Savitzky-Golay filter for position smoothing
  - Circular moving average for heading smoothing
- **Parameters**:
  - `position_smoothing_window`: Window size for position smoothing (default: 11 points)
  - `position_polynomial_order`: Polynomial order for Savitzky-Golay filter (default: 3)

### 3.2 Output Format

The mean course shall be represented as:
- **Class**: `MeanCourse`
- **Attributes**:
  - `x`: numpy array of X coordinates
  - `y`: numpy array of Y coordinates
  - `heading`: numpy array of heading angles (degrees)
  - `distance`: numpy array of cumulative arc-length distances
  - `num_laps`: Number of laps used in calculation
  - `metadata`: Dictionary with processing parameters

### 3.3 Validation Metrics

The system shall compute and report:
- **Path variance**: Standard deviation of lap positions from mean at each point
- **Heading variance**: Standard deviation of heading angles at each point
- **Coverage**: Percentage of course with data from all laps
- **Outlier count**: Number of points rejected as outliers

## 4. Auto-Segmentation

### 4.1 Requirements

#### R2.1: Segment Types
- **ID**: R2.1
- **Description**: System shall classify course into the following segment types:
  1. **Straight**: Minimal curvature, heading nearly constant
  2. **Left Turn**: Positive curvature, heading increasing
  3. **Right Turn**: Negative curvature, heading decreasing
  4. **S-Curve Left-Right**: Inflection point, left turn followed by right turn
  5. **S-Curve Right-Left**: Inflection point, right turn followed by left turn
  6. **Chicane**: Rapid alternating turns (multiple inflection points)

#### R2.2: Curvature Calculation
- **ID**: R2.2
- **Description**: Compute path curvature at each point
- **Method**:
  - Use finite differences of heading angle with respect to arc length
  - κ = dθ/ds where θ is heading and s is arc length
  - Smooth curvature using moving average
- **Parameters**:
  - `curvature_window`: Window size for curvature calculation (default: 5 points)
  - `curvature_smoothing_window`: Window size for curvature smoothing (default: 11 points)

#### R2.3: Segment Boundary Detection
- **ID**: R2.3
- **Description**: Detect boundaries between segment types
- **Method**:
  - Identify sign changes in curvature (inflection points)
  - Identify zero-crossings with threshold
  - Merge short segments below minimum length
- **Parameters**:
  - `straight_curvature_threshold`: Maximum curvature for straight classification (default: 0.05 rad/m)
  - `min_segment_length`: Minimum segment arc length (default: 0.5 meters)
  - `inflection_threshold`: Curvature threshold near zero for inflection detection (default: 0.02 rad/m)

#### R2.4: Segment Classification
- **ID**: R2.4
- **Description**: Assign segment type based on curvature characteristics
- **Classification Rules**:
  ```
  IF |mean_curvature| < straight_curvature_threshold:
      type = STRAIGHT
  ELIF mean_curvature > 0:
      type = LEFT_TURN
  ELIF mean_curvature < 0:
      type = RIGHT_TURN

  IF segment has inflection points:
      IF starts_left and ends_right:
          type = S_CURVE_LR
      ELIF starts_right and ends_left:
          type = S_CURVE_RL
      IF multiple_inflections (>2):
          type = CHICANE
  ```

#### R2.5: Segment Metrics
- **ID**: R2.5
- **Description**: Compute geometric properties for each segment
- **Metrics**:
  - `segment_id`: Integer segment identifier (0 to N-1)
  - `segment_type`: Segment classification (enum)
  - `start_index`: Start index in mean course arrays
  - `end_index`: End index in mean course arrays
  - `length`: Arc length of segment (meters)
  - `mean_curvature`: Average curvature (rad/m)
  - `max_curvature`: Maximum absolute curvature (rad/m)
  - `total_heading_change`: Total change in heading angle (degrees)
  - `entry_heading`: Heading at segment start (degrees)
  - `exit_heading`: Heading at segment exit (degrees)

### 4.2 Output Format

The segmentation shall be represented as:
- **Class**: `CourseSegmentation`
- **Attributes**:
  - `segments`: List of `Segment` objects
  - `total_segments`: Total number of segments
  - `segment_counts`: Dictionary with count per segment type
  - `mean_course`: Reference to associated MeanCourse object

- **Class**: `Segment`
- **Attributes**: As defined in R2.5

### 4.3 Validation Metrics

The system shall compute and report:
- **Segment count by type**: Number of each segment type
- **Total course coverage**: Verify all points are assigned to segments
- **Segment length statistics**: Min, max, mean, median segment lengths
- **Curvature distribution**: Histogram of curvature values

## 5. Segment Estimator

### 5.1 Requirements

#### R3.1: Real-Time Segment Detection
- **ID**: R3.1
- **Description**: Given current vehicle position (x, y) and heading, return the segment ID the vehicle is currently in
- **Method**:
  - Find nearest point on mean course
  - Use heading angle to disambiguate if position is ambiguous (e.g., overlapping segments)
  - Return segment ID and confidence score
- **Performance**: Must execute in < 10ms for real-time applications

#### R3.2: Position Matching
- **ID**: R3.2
- **Description**: Efficiently find closest point on mean course
- **Method**:
  - Use KD-tree or spatial indexing for fast nearest-neighbor search
  - Search within segments near last known position (if available)
  - Fallback to global search if needed
- **Parameters**:
  - `search_radius`: Maximum search radius from last position (default: 5.0 meters)
  - `position_tolerance`: Distance threshold for position match (default: 2.0 meters)

#### R3.3: Heading Validation
- **ID**: R3.3
- **Description**: Use heading angle to validate position match and handle ambiguous cases
- **Method**:
  - Compare vehicle heading with course heading at candidate positions
  - Select position with minimum heading difference
  - Reject matches with large heading errors
- **Parameters**:
  - `heading_tolerance`: Maximum heading difference for valid match (default: 45 degrees)
  - `heading_weight`: Weight for heading in combined distance metric (default: 0.3)

#### R3.4: Confidence Scoring
- **ID**: R3.4
- **Description**: Provide confidence score for segment estimation
- **Factors**:
  - Distance from mean course (closer = higher confidence)
  - Heading alignment (better alignment = higher confidence)
  - Segment uniqueness (no nearby ambiguous segments = higher confidence)
- **Output**: Confidence score in range [0.0, 1.0]

#### R3.5: Edge Case Handling
- **ID**: R3.5
- **Description**: Handle special cases robustly
- **Cases**:
  - Vehicle far from course (off-track): Return None or "unknown" segment
  - Vehicle in overlap region: Use heading to disambiguate
  - Vehicle near segment boundary: Return current segment with lower confidence
  - First estimation (no previous position): Perform global search

### 5.2 API Design

```python
class SegmentEstimator:
    def __init__(self, course_segmentation: CourseSegmentation):
        """Initialize estimator with segmented course"""
        pass

    def estimate(self, x: float, y: float, heading: float,
                 last_position: Optional[Tuple[float, float]] = None) -> SegmentEstimate:
        """
        Estimate current segment from vehicle state

        Args:
            x: Vehicle X position (meters)
            y: Vehicle Y position (meters)
            heading: Vehicle heading (degrees, 0=forward, positive=left)
            last_position: Optional last known position for incremental search

        Returns:
            SegmentEstimate object with segment_id, confidence, and debug info
        """
        pass

    def estimate_batch(self, positions: np.ndarray) -> List[SegmentEstimate]:
        """
        Estimate segments for batch of positions (for offline analysis)

        Args:
            positions: Nx3 array of (x, y, heading)

        Returns:
            List of SegmentEstimate objects
        """
        pass

class SegmentEstimate:
    segment_id: Optional[int]  # None if vehicle off-track
    confidence: float  # Range [0.0, 1.0]
    distance_to_course: float  # Meters
    heading_error: float  # Degrees
    course_position: Tuple[float, float]  # Matched (x,y) on mean course
    course_heading: float  # Expected heading at matched position
```

### 5.3 Performance Requirements

- **Latency**: < 10ms per estimation on typical hardware (Raspberry Pi 4)
- **Accuracy**: > 95% correct segment estimation when vehicle is within 1m of course
- **Robustness**: Handle up to 100 estimations per second continuously

## 6. Implementation Details

### 6.1 File Structure

```
donkeycar/
├── parts/
│   └── course_analysis.py          # Main implementation
├── tests/
│   └── test_course_analysis.py     # Comprehensive test suite
└── utilities/
    └── course_analyzer/
        ├── __init__.py
        ├── analyzer.py              # Command-line tool
        └── visualizer.py            # Visualization utilities
```

### 6.2 Dependencies

**Core Dependencies** (already in donkeycar):
- `numpy`: Numerical computations
- `scipy`: Signal processing (Savitzky-Golay, interpolation)
- `donkeycar.la`: Vec2 class for 2D vector operations

**Additional Dependencies**:
- `scikit-learn`: KD-tree for spatial indexing (if not already included)
- `pandas`: CSV reading/writing (optional, can use numpy)

### 6.3 Main Classes

```python
class MultiLapData:
    """Container for raw multi-lap course data"""
    def load_csv(self, filepath: str) -> None
    def get_laps(self) -> List[np.ndarray]

class MeanCourse:
    """Reconstructed mean course from multi-lap data"""
    def __init__(self, multilap_data: MultiLapData, params: dict)
    def compute(self) -> None
    def save(self, filepath: str) -> None
    def load(self, filepath: str) -> None

class CourseSegmentation:
    """Segmented course with classified segments"""
    def __init__(self, mean_course: MeanCourse, params: dict)
    def compute(self) -> None
    def get_segment(self, segment_id: int) -> Segment
    def save(self, filepath: str) -> None
    def load(self, filepath: str) -> None

class Segment:
    """Individual course segment with geometric properties"""
    # Attributes as defined in R2.5

class SegmentEstimator:
    """Real-time segment estimation from vehicle position/heading"""
    # API as defined in 5.2
```

## 7. Testing Requirements

### 7.1 Unit Tests

#### Test Coverage
- **Minimum code coverage**: 90% of non-visualization code
- **Test framework**: Python unittest (following donkeycar conventions)

#### Test Cases by Component

**MultiLapData**:
- Load valid CSV with multiple laps
- Handle missing columns
- Handle invalid data types
- Handle empty CSV
- Detect laps correctly

**MeanCourse**:
- Compute mean from 2 laps (simple case)
- Compute mean from 5+ laps
- Handle noisy data with outliers
- Handle laps with different lengths
- Validate output smoothness
- Test different parameter settings

**CourseSegmentation**:
- Segment simple straight course
- Segment course with single turn
- Segment course with S-curve
- Segment complex course with all segment types
- Handle edge cases (very short segments, etc.)
- Validate segment continuity (no gaps)

**SegmentEstimator**:
- Estimate segment for position on course
- Estimate segment for position near course
- Estimate segment for off-track position
- Handle ambiguous positions with heading
- Handle segment boundary positions
- Batch estimation performance

### 7.2 Integration Tests

- **End-to-end workflow**: Load CSV → compute mean → segment → estimate
- **Real data compatibility**: Test with actual GPS/odometry data
- **Performance benchmarks**: Verify latency requirements

### 7.3 Test Data

Create synthetic test datasets:
1. **Oval track**: Simple oval with 2 straights and 2 turns
2. **Figure-8**: Course with crossover point (tests ambiguity handling)
3. **Racetrack**: Complex course with straights, turns, S-curves, chicane
4. **Noisy data**: Same course with added Gaussian noise
5. **Outlier data**: Same course with random outlier points

## 8. Validation and Metrics

### 8.1 Mean Course Quality Metrics

- **Repeatability**: Standard deviation of position across laps
- **Smoothness**: Measure of curvature continuity
- **Closure error**: Distance between start and end of closed course

### 8.2 Segmentation Quality Metrics

- **Segment consistency**: Visual inspection that segments align with course geometry
- **Boundary precision**: Segment boundaries occur at curvature inflections
- **Classification accuracy**: Manual validation of segment types

### 8.3 Estimator Performance Metrics

- **Accuracy**: Percentage of correct segment estimates on test data
- **Latency**: Time per estimation (mean, max, 95th percentile)
- **Robustness**: Performance degradation with noise, outliers, off-track positions

## 9. Future Enhancements

The following features are out of scope for initial implementation but may be added later:

1. **Speed profiling**: Optimal speed recommendations per segment
2. **Racing line optimization**: Compute optimal racing line from mean course
3. **Multi-vehicle analysis**: Compare courses from different vehicles/drivers
4. **Online learning**: Update mean course incrementally with new laps
5. **3D course support**: Extend to 3D coordinates with elevation
6. **Visualization dashboard**: Web-based tool for interactive course analysis
7. **Track database**: Library of known courses with pre-computed segmentations

## 10. Success Criteria

The implementation shall be considered successful when:

1. ✅ All unit tests pass with >90% code coverage
2. ✅ Integration tests pass with real and synthetic data
3. ✅ Mean course computation produces smooth, reasonable results
4. ✅ Segmentation correctly identifies segment types on test courses
5. ✅ Segment estimator achieves >95% accuracy on test data
6. ✅ Estimator latency is <10ms on target hardware
7. ✅ Code follows donkeycar conventions and style
8. ✅ Documentation is complete and clear

## Appendix A: Coordinate System and Conventions

### Coordinate Frame
- **Origin**: Arbitrary local reference point
- **X-axis**: Typically East or forward direction at start
- **Y-axis**: Typically North or left direction at start
- **Units**: Meters

### Heading Angle
- **Reference**: 0° = vehicle forward direction
- **Positive rotation**: Counterclockwise (left turn)
- **Range**: [0°, 360°) or [-180°, 180°]
- **Conversion**: System shall handle both representations

### Arc Length
- **Measurement**: Cumulative distance along path from start
- **Calculation**: ∫√(dx² + dy²) using trapezoidal integration

## Appendix B: Mathematical Formulations

### Curvature Calculation
For discrete path with points (x_i, y_i) and headings θ_i:

```
κ_i = Δθ_i / Δs_i
```

Where:
- `Δθ_i = θ_{i+1} - θ_{i-1}` (central difference)
- `Δs_i = s_{i+1} - s_{i-1}` (arc length difference)

### Heading Averaging
For circular quantities (angles), use circular mean:

```
θ_mean = atan2(mean(sin(θ)), mean(cos(θ)))
```

### Position-Heading Distance Metric
Combined metric for nearest point search:

```
d_combined = √(dx² + dy²) + w_h * |Δθ|
```

Where:
- `dx, dy`: Position differences
- `Δθ`: Heading difference (wrapped to [-180°, 180°])
- `w_h`: Heading weight parameter

## Appendix C: References

1. DonkeyCar path following: `donkeycar/parts/path.py`
2. DonkeyCar geometry utilities: `donkeycar/geom.py`, `donkeycar/la.py`
3. Savitzky-Golay filter: scipy.signal.savgol_filter
4. KD-tree: scipy.spatial.KDTree or sklearn.neighbors.KDTree
