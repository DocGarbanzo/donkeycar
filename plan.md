# Plan: Synthetic Course Tests for IMU Path Analysis

## Goal

Build a comprehensive test suite using geometrically-defined synthetic courses
with realistic driving simulation. Tests validate the full pipeline: lap
detection, mean course recovery, segmentation, segment assignment, and segment
statistics.

## 1. Synthetic Course Generator: L-Shape

### Geometry

The course traces around the outside of the letter 'L' at offset distance `d`
(also used as turn radius). Counter-clockwise traversal.

Parameters:
- `h_len`: horizontal bar length (default 10.0m)
- `v_len`: vertical bar length (default 15.0m)
- `d`: offset distance / turn radius (default 1.0m)
- `points_per_meter`: point density (default 10)

The L-shape skeleton is two perpendicular lines meeting at the origin:
- Horizontal: (0,0) to (h_len, 0)
- Vertical: (0,0) to (0, v_len)

### 8 Course Segments

Starting at (0, -d) heading east:

| # | Type | From → To | Heading | Length |
|---|------|-----------|---------|--------|
| S1 | Straight | (0,-d) → (h_len,-d) | East (0) | h_len |
| A1 | Semicircle 180° L | center (h_len,0), r=d | 0 → π | πd |
| S2 | Straight | (h_len,d) → (2d,d) | West (π) | h_len-2d |
| A2 | Quarter 90° R (CW) | center (2d,2d), r=d | π → π/2 | πd/2 |
| S3 | Straight | (d,2d) → (d,v_len) | North (π/2) | v_len-2d |
| A3 | Semicircle 180° L | center (0,v_len), r=d | π/2 → -π/2 | πd |
| S4 | Straight | (-d,v_len) → (-d,0) | South (-π/2) | v_len |
| A4 | Quarter 90° L (CCW) | center (0,0), r=d | -π/2 → 0 | πd/2 |

Total length = 2·h_len + 2·v_len - 2d + 3πd

Constraints: h_len > 2d, v_len > 2d

Y-crossing (neg→pos) occurs in A1 at position (h_len+d, 0), enabling
`YCrossingLapDetector` to find laps.

### Output

Returns arrays: x, y, heading, distance + segment metadata (list of dicts with
segment_id, type, start_distance, end_distance, start_index, end_index).

## 2. Driving Simulator

### Noise Model: Ornstein-Uhlenbeck Cross-Track Error

Simple random noise creates unrealistic zig-zag paths that don't look like real
driving. The Ornstein-Uhlenbeck (OU) process models realistic driver behavior:

```
dε = -θ · ε · ds + σ · dW
```

Where:
- `ε`: cross-track error (perpendicular distance from course centerline)
- `θ`: mean-reversion rate — how aggressively the driver corrects back to the
  course centerline (higher = snaps back faster)
- `σ`: noise intensity — how much the driver wanders
- `ds`: arc-length step (parameterized by distance, not time)
- `dW`: Wiener process increment (Gaussian noise)

**Why OU works:**
1. **Smooth** — correlated noise, no sudden jumps (realistic driving)
2. **Mean-reverting** — driver naturally corrects back to centerline
3. **Zero-mean** — averaging N laps converges to the designed course
4. **Tunable** — θ controls driving style (tight vs. loose)

**Statistical properties:**
- Stationary variance: σ²/(2θ)
- Correlation length: ~1/θ (in meters of arc length)

### Simulator Interface

```python
def simulate_driving(
    course_x, course_y, course_heading, course_distance,
    num_laps=5,
    theta=5.0,       # mean-reversion rate
    sigma=0.15,      # noise intensity (~3cm std at theta=5)
    base_speed=2.0,  # m/s
    speed_variation=0.3,  # ±15% speed noise (also OU-driven)
    seed=None
) -> PathData:
```

### Per-Lap Generation

1. Generate OU cross-track error ε(s) along the course arc length
2. Apply perpendicular offset: x' = x + ε·(-sin h), y' = y + ε·(cos h)
3. Generate OU speed variation: v(s) = base_speed · (1 + speed_noise(s))
4. Compute heading from actual dx/dy of noisy path
5. Compute timestamps from cumulative distance / velocity
6. Concatenate laps into single PathData

### Why the Mean Course Won't Exactly Recover the Original

With finite laps (5-10), the OU noise doesn't perfectly cancel. The residual
error scales as σ/√(N·θ). With σ=0.15, θ=5, N=10: ~2cm residual. Tests use
appropriate tolerances (e.g., 5-10cm for shape, not exact equality).

## 3. File Structure

```
donkeycar/tests/
├── synthetic_courses.py           # NEW: Course generators + driving simulator
│   ├── class LShapeCourse         # L-shape geometry generator
│   ├── simulate_driving()         # OU-based driving simulator
│   └── create_multilap_pathdata() # Convenience: course + simulation
└── test_imu_synthetic.py          # NEW: Comprehensive test suite
```

## 4. Test Suite

### 4.1 Course Geometry Tests

```python
class TestLShapeCourseGeometry:
```

| Test | What it verifies |
|------|-----------------|
| `test_closed_loop` | Start and end points coincide (< 1mm) |
| `test_total_length` | Total arc length = 2h + 2v - 2d + 3πd |
| `test_segment_count` | Exactly 8 segments in metadata |
| `test_straight_headings` | S1=0, S2=π, S3=π/2, S4=-π/2 |
| `test_heading_continuity` | No heading jumps > threshold |
| `test_y_crossing_exists` | Path crosses y=0 from neg to pos exactly once |

### 4.2 Driving Simulation Tests

```python
class TestDrivingSimulation:
```

| Test | What it verifies |
|------|-----------------|
| `test_path_is_smooth` | Max step-to-step distance < threshold |
| `test_cross_track_bounded` | Max |ε| < 4σ/√(2θ) with high probability |
| `test_heading_matches_trajectory` | Heading ≈ atan2(dy, dx) |
| `test_timestamps_monotonic` | Timestamps strictly increasing |
| `test_different_seeds_different_paths` | Two seeds produce different paths |
| `test_mean_of_laps_approaches_course` | Mean of 10 laps within tolerance of designed course |

### 4.3 Lap Detection Tests

```python
class TestLapDetectionOnSynthetic:
```

| Test | What it verifies |
|------|-----------------|
| `test_y_crossing_detects_correct_count` | N laps simulated → N (or N-1 partial) detected |
| `test_drift_detector_finds_laps` | DriftLapDetector also works on this data |
| `test_lap_lengths_consistent` | All laps have similar point counts (within 10%) |
| `test_lap_distances_consistent` | All laps have similar total distance |

### 4.4 Mean Course Recovery Tests

```python
class TestMeanCourseRecovery:
```

| Test | What it verifies |
|------|-----------------|
| `test_mean_course_resembles_original` | Mean course within tolerance of designed course |
| `test_mean_course_is_closed` | Start/end distance < tolerance |
| `test_mean_course_length_close` | Total length within 5% of designed |
| `test_more_laps_better_recovery` | RMSE(10 laps) < RMSE(3 laps) |

### 4.5 Segmentation Tests

```python
class TestSegmentationOnSynthetic:
```

| Test | What it verifies |
|------|-----------------|
| `test_segment_count_matches_geometry` | ~8 segments detected (±2 for merging) |
| `test_straights_classified_correctly` | S1-S4 identified as STRAIGHT |
| `test_turns_classified_correctly` | A1-A4 identified as turns |
| `test_segmentation_on_recovered_course` | Works on mean course from noisy laps |
| `test_segment_boundaries_at_transitions` | Boundaries near straight↔turn transitions |

### 4.6 Segment Assignment Tests (Integration)

```python
class TestSegmentAssignmentIntegration:
```

| Test | What it verifies |
|------|-----------------|
| `test_each_lap_visits_all_segments` | Every detected lap touches all segment IDs |
| `test_sequential_segment_order` | Segments visited in order 0→1→...→N-1→0 |
| `test_segment_transitions_at_boundaries` | Transitions occur near expected boundary locations |
| `test_assignment_consistent_across_laps` | Same position maps to same segment in different laps |
| `test_no_segment_visited_twice_per_lap` | Each segment entered exactly once per lap |

### 4.7 Segment Statistics Tests

```python
class TestSegmentStatisticsOnSynthetic:
```

| Test | What it verifies |
|------|-----------------|
| `test_segment_times_sum_to_lap_time` | Per-segment times add up to lap time |
| `test_segment_distances_sum_to_lap_distance` | Per-segment distances add up |
| `test_faster_lap_has_lower_time_ranking` | Ranking reflects actual performance |
| `test_all_segments_have_statistics` | No missing segment entries |
| `test_ranking_percentiles_in_range` | All percentile values in [0, 1] |

## 5. Implementation Order

1. Create branch `claude/add-imu-synthetic-tests-BCYmc`
2. Implement `LShapeCourse` in `synthetic_courses.py`
3. Implement `simulate_driving()` in `synthetic_courses.py`
4. Implement test classes in `test_imu_synthetic.py`, running each as we go
5. Run full test suite to ensure no regressions
6. Commit and push

## 6. Design Decisions

- **OU process parameterized by arc length**, not time — noise stats are
  distance-based, independent of speed
- **L-shape chosen** for its mix of geometric features: 4 straights, 2
  semicircles, 2 quarter-circles (one CW, one CCW)
- **Tests use tolerances**, not exact equality — acknowledges finite-lap noise
- **No Tub creation needed** — tests work directly with PathData, MeanCourse,
  and CourseSegmentation objects (faster, no I/O)
- **Seed-controlled randomness** — reproducible tests via fixed random seeds
