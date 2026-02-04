# PRD: Comprehensive Testing of Segment-Based Ranking System

## 1. Problem Statement

The segment-based performance ranking system allows training on the
best-driven instance of each course segment across all laps, creating a
"synthetic perfect lap." This system involves multiple interacting
components: tub data ingestion, field aggregation with configurable
statistics, multi-criteria ranking, and training pipeline integration.

Current test coverage focuses on individual components with limited
aggregation configurations (typically just `time + distance + gyro_z`
with `avg`). There is no systematic test of the combinatorial space of
aggregation methods, transform functions, multi-field ranking
interactions, or realistic driving behavior patterns that these
measurements are designed to categorize.

### What We Want to Categorize

The ranking system classifies driving behavior along multiple
measurable dimensions:

| Dimension | Measurement | What It Tells Us |
|-----------|-------------|------------------|
| Speed | `time` (boundary) | How fast the segment was driven |
| Efficiency | `distance` (boundary) | How tight the racing line was |
| Smoothness | `car/gyro[2]`, `avg(abs)` | Jerky vs smooth cornering |
| Braking | `car/accel[0]`, `min` | How hard the driver brakes |
| Acceleration | `car/accel[0]`, `max` | How aggressively they accelerate |
| Stability | `car/gyro[0]`, `avg(abs)` | Roll stability through corners |
| Consistency | `car/speed`, `delta` | Speed variance through segment |
| Cornering G | `car/accel[1]`, `max(abs)` | Lateral force in turns |

Testing must verify that all these dimensions can be configured,
aggregated, and ranked correctly--both individually and in combination.

## 2. Goals

1. **Validate all aggregation methods** (`avg`, `sum`, `min`, `max`,
   `median`, `delta`) with realistic driving data
2. **Test multi-field ranking interactions** with 2, 3, and 4+ ranking
   criteria in various priority orderings
3. **Verify driving behavior categorization** -- that the ranking system
   actually distinguishes fast-smooth from fast-jerky, slow-stable from
   slow-unstable, etc.
4. **Use the official tub interface** exclusively (`Tub.write_record`)
   to create test data, matching how real recording code works
5. **Eliminate duplication** with existing tests by building on shared
   fixtures and testing what existing tests do NOT cover
6. **Integration over isolation** -- test the full pipeline from record
   creation through ranking output

## 3. Gap Analysis: What Existing Tests Cover vs. What's Missing

### Already Covered (Do Not Duplicate)

| Area | Existing Test File | What It Tests |
|------|-------------------|---------------|
| Lap time generation | `test_tub_statistics.py` | Single/multi session, overwrite, bins |
| Gyro index config | `test_tub_statistics.py` | index=1 vs index=2 |
| Segment structure | `test_segment_performance.py` | Rankings dict shape, percentiles |
| Segment ranking order | `test_segment_performance.py` | Fast < Medium < Slow |
| Synthetic best lap | `test_segment_performance.py` | Best segments from different laps |
| Training integration | `test_course_segmentation_integration.py` | PctMode.SEGMENT with TubDataset |
| End-to-end workflow | `test_segment_training_integration.py` | Record -> Segment -> Train |
| Backward compat | `test_lap_pct_regression.py` | PctMode.LAP still works |
| Segment assignment | `test_segment_assignment.py` | Boundary crossing, wraparound |
| Segment invariant | `test_segment_statistics_comprehensive.py` | Each lap visits each segment once |

### Missing Coverage (This PRD Addresses)

1. **Aggregation methods beyond `avg`**: No test exercises `sum`, `min`,
   `max`, `median`, or `delta` with realistic data
2. **Multi-field ranking priority**: No test verifies that ranking
   priority order (1st field = primary sort key) works correctly with
   >2 criteria
3. **Transform functions beyond `abs`**: No test uses custom transforms
   (e.g., `lambda x: x**2`, `lambda x: -x`)
4. **Driving behavior discrimination**: No test verifies that a
   fast-but-jerky segment ranks differently from a fast-and-smooth one
5. **Realistic sensor correlations**: Existing tests use uniform or
   simple profiles; real driving has correlated sensor readings (high
   gyro in turns correlates with high lateral accel)
6. **Record fields beyond `car/gyro`**: No test uses `car/accel`,
   `car/speed`, or custom fields in aggregation
7. **Multiple record-field aggregations**: Tests use at most 1 record
   field + 2 boundary fields; no test combines multiple record fields
8. **Edge cases in aggregation**: What happens with `delta` on a
   single-record segment? `median` with even count?
9. **Reverse ranking**: No test exercises `reverse=True` in sorting
10. **Tub creation via TubWriter**: Existing segment tests use
    `Tub.write_record` directly; no test uses `TubWriter.run()` which
    is the actual recording path

## 4. Test Architecture

### 4.1 Shared Test Data Generator

Create a reusable generator that produces realistic multi-lap tub data
with correlated sensor readings. This is NOT a fixture file -- it is a
module with functions that create tub data through the official
interface.

**File:** `donkeycar/tests/tub_test_data_generator.py`

```
Functions:
- create_segment_test_tub(
    tub_path,
    driving_profiles,   # List of per-segment driving characteristics
    num_laps,
    records_per_segment,
    use_tub_writer=False  # True = TubWriter.run(), False = Tub.write_record()
  ) -> Tub

- create_driving_profile(
    speed,              # m/s - determines time and distance
    smoothness,         # 0-1 - maps to gyro amplitude
    braking_force,      # 0-1 - maps to accel_x min
    lateral_g,          # 0-1 - maps to accel_y amplitude
    stability           # 0-1 - maps to gyro_x amplitude
  ) -> dict

- create_correlated_sensor_record(
    profile, segment_progress, track_geometry
  ) -> dict
    # Generates realistic correlated sensor values:
    # - gyro_z correlates with track curvature
    # - accel_y correlates with speed * curvature (centripetal)
    # - accel_x reflects braking/acceleration phases
    # - speed varies based on segment type
```

**Key design decisions:**
- All records created through `Tub.write_record()` or `TubWriter.run()`
- No images needed: use `image_array` type with tiny 1x1 arrays
- Include ALL standard tub fields: `car/gyro`, `car/accel`, `car/speed`,
  `car/pos`, `car/distance`, `car/lap`, `car/segment`
- Sensor values are physically plausible and correlated (not random)
- Driving profiles parameterize behavior, not raw sensor values

### 4.2 Test File Structure

**Single new test file:** `donkeycar/tests/test_segment_ranking_comprehensive.py`

This file imports from the shared generator and existing test utilities.
It does NOT duplicate any test from the existing files listed in
Section 3.

## 5. Test Specifications

### 5.1 Aggregation Method Verification

**Class:** `TestFieldAggregationMethods`

**Purpose:** Verify each aggregation method produces correct results
with realistic driving data.

#### Test: `test_avg_aggregation_gyro_smoothness`
- **Setup:** 3 laps, 4 segments. Segment 0 has different smoothness
  each lap (smooth=0.1 gyro, medium=0.5, jerky=0.9)
- **Config:** `FIELD_AGGREGATIONS = [time, distance, {car/gyro, index=2, avg, abs}]`
- **Assert:** Smooth lap's `gyro_z_agg` ranking < jerky lap's ranking

#### Test: `test_sum_aggregation_total_yaw`
- **Setup:** Same tub. One segment driven with many small corrections
  (sum of abs(gyro_z) is high) vs one clean turn (sum is low)
- **Config:** `{car/gyro, index=2, sum, abs}`
- **Assert:** Clean-turn lap ranks better (lower sum) than
  many-corrections lap

#### Test: `test_min_aggregation_hardest_braking`
- **Setup:** 3 laps, 4 segments. Segment 2 (a turn entry) has different
  braking profiles: hard brake (accel_x min = -8), medium (-4), light (-1)
- **Config:** `{car/accel, index=0, min, identity}`
- **Assert:** Hard-braking lap ranks worst (most negative min). With
  default ascending sort, the lap with least negative min ranks best.

#### Test: `test_max_aggregation_peak_acceleration`
- **Setup:** 3 laps. Segment 3 (turn exit) has different acceleration
  profiles: aggressive (accel_x max = 5), moderate (3), gentle (1)
- **Config:** `{car/accel, index=0, max, identity}`
- **Assert:** Rankings order correctly by peak acceleration value

#### Test: `test_median_aggregation_typical_speed`
- **Setup:** 3 laps. Segment 1 (straight) has different speed
  distributions: consistent 10m/s, mostly 10 with spike to 15,
  mostly 5 with spike to 20
- **Config:** `{car/speed, index=None, median, identity}`
- **Assert:** Median correctly ignores outlier spikes. Consistent-10
  and spike-to-15 laps have similar median; spike-to-20 lap has
  lower median (mostly 5)

#### Test: `test_delta_aggregation_speed_change`
- **Setup:** 3 laps. Segment 0 starts at 8m/s ends at 12m/s (delta=4),
  another starts at 10 ends at 10 (delta=0), another 12 to 8 (delta=-4)
- **Config:** `{car/speed, index=None, delta, identity}`
- **Assert:** Delta values correctly computed as last - first. Rankings
  ordered by delta magnitude.

#### Test: `test_delta_single_record_segment`
- **Setup:** Segment with exactly 1 record
- **Config:** `{car/speed, delta}`
- **Assert:** Delta = 0 (last - first = value - value)

#### Test: `test_median_even_count`
- **Setup:** Segment with exactly 4 records
- **Config:** `{car/speed, median}`
- **Assert:** Median uses middle element of sorted list (Python's
  integer division floor)

### 5.2 Multi-Field Ranking Priority

**Class:** `TestMultiFieldRankingPriority`

**Purpose:** Verify that the ordering of fields in FIELD_AGGREGATIONS
determines ranking priority, and that secondary criteria break ties.

#### Test: `test_primary_sort_key_dominates`
- **Setup:** 3 laps, 2 segments.
  - Segment 0: Lap 1 time=2.0 gyro=0.1, Lap 2 time=1.0 gyro=0.9,
    Lap 3 time=3.0 gyro=0.05
  - Time is primary, gyro is secondary
- **Config:** `[time, distance, {gyro, avg, abs}]`
- **Assert:** Lap 2 ranks best for segment 0 (fastest) despite worst
  gyro. Ranking: Lap2 < Lap1 < Lap3 by time.

#### Test: `test_secondary_breaks_ties`
- **Setup:** 3 laps, 2 segments. All laps have identical time for
  segment 0, but different gyro values
- **Config:** `[time, distance, {gyro, avg, abs}]`
- **Assert:** With equal times, gyro becomes the tiebreaker. Smoothest
  gyro ranks best.

#### Test: `test_four_field_ranking`
- **Setup:** 4 laps, 3 segments with distinct values for each of
  4 ranking fields
- **Config:** `[time, distance, {gyro_z, avg, abs}, {accel_x, min}]`
- **Assert:** Rankings computed for all 4 fields independently. Each
  record's `lap_pct` vector has exactly 4 elements in config order.

#### Test: `test_two_field_ranking_time_distance_only`
- **Setup:** 3 laps with different times and distances
- **Config:** `[time, distance]` -- no record fields at all
- **Assert:** Rankings work with boundary fields only. `lap_pct`
  has exactly 2 elements.

#### Test: `test_ranking_order_matches_config_order`
- **Setup:** 3 laps
- **Config A:** `[time, {gyro, avg}]` -> Config B: `[{gyro, avg}, time]`
- **Assert:** Same data produces different `lap_pct` vectors when
  field order is swapped. `lap_pct[0]` in config A corresponds to
  time; `lap_pct[0]` in config B corresponds to gyro.

### 5.3 Driving Behavior Discrimination

**Class:** `TestDrivingBehaviorDiscrimination`

**Purpose:** Verify that the ranking system can distinguish meaningful
driving behavior categories using realistic correlated sensor data.

#### Test: `test_fast_smooth_vs_fast_jerky`
- **Setup:** 2 laps, 4 segments. Both laps drive segment 0 at same
  speed (same time), but lap 1 is smooth (low gyro variance, low
  lateral accel) and lap 2 is jerky (high gyro peaks, abrupt
  corrections)
- **Config:** `[time, {gyro_z, avg, abs}, {accel_y, max, abs}]`
- **Assert:** Same time ranking, but smooth lap ranks better on gyro
  and lateral accel. Training system would prefer smooth lap for
  this segment.

#### Test: `test_slow_stable_vs_fast_unstable`
- **Setup:** 2 laps. Lap 1 is slow but stable (low gyro, consistent
  speed). Lap 2 is fast but unstable (high gyro corrections, speed
  spikes)
- **Config:** `[time, {gyro_z, avg, abs}]`
- **Assert:** Lap 2 ranks better on time, Lap 1 ranks better on gyro.
  The `lap_pct` vectors capture this tradeoff -- neither lap dominates
  on all criteria.

#### Test: `test_tight_racing_line_vs_wide_line`
- **Setup:** 2 laps, same time. Lap 1 takes tight line (shorter
  distance, higher lateral G). Lap 2 takes wide line (longer distance,
  lower lateral G)
- **Config:** `[time, distance, {accel_y, max, abs}]`
- **Assert:** Lap 1 ranks better on distance (shorter), worse on
  lateral G. The ranking captures the line-choice tradeoff.

#### Test: `test_aggressive_vs_conservative_braking`
- **Setup:** 2 laps. Segment 2 is a braking zone into a corner.
  Lap 1 brakes late and hard (high negative accel_x, short brake
  duration). Lap 2 brakes early and gentle (moderate negative accel_x,
  long brake duration)
- **Config:** `[time, {accel_x, min}]`
- **Assert:** Both achieve similar time. Lap 1 has more negative min
  accel_x (harder braking). Ranking captures braking style.

### 5.4 Transform Function Testing

**Class:** `TestTransformFunctions`

**Purpose:** Verify that custom transform functions are applied
correctly before aggregation.

#### Test: `test_abs_transform`
- **Setup:** Gyro values alternate positive/negative (left/right turns)
- **Config:** `{car/gyro, index=2, transform=abs, aggregation=avg}`
- **Assert:** Aggregated value is average of absolute values, not raw
  average (which could be ~0 for symmetric turns)

#### Test: `test_square_transform`
- **Setup:** Gyro values with outliers
- **Config:** `{car/gyro, index=2, transform=lambda x: x**2, aggregation=avg}`
- **Assert:** Squared transform amplifies outliers. Lap with occasional
  large gyro spikes ranks worse than lap with consistent moderate gyro.

#### Test: `test_identity_transform`
- **Setup:** Accel values that are negative (braking)
- **Config:** `{car/accel, index=0, transform=None, aggregation=min}`
- **Assert:** No transform applied. Min returns actual most-negative
  value.

#### Test: `test_reverse_ranking`
- **Setup:** 3 laps with different speeds
- **Config:** `{car/speed, aggregation=avg, reverse=True}`
- **Assert:** With `reverse=True`, highest average speed ranks best
  (lowest percentile). Default ascending would rank lowest speed best.

### 5.5 Official Tub Interface Testing

**Class:** `TestTubWriterIntegration`

**Purpose:** Verify that test data created through `TubWriter.run()`
(the actual recording path) produces identical ranking results to data
created through `Tub.write_record()`.

#### Test: `test_tub_writer_produces_valid_rankings`
- **Setup:** Create tub using `TubWriter` with inputs/types matching
  the real `donkey5` template. Write records using `writer.run(...)`.
  Create laptimer metadata. Run `calculate_segment_performance()`.
- **Assert:** Rankings have correct structure, values are in [0, 1],
  expected ranking order matches.

#### Test: `test_tub_writer_vs_direct_write_consistency`
- **Setup:** Create two tubs with identical data: one via `TubWriter`,
  one via `Tub.write_record()`. Run same aggregation on both.
- **Assert:** Rankings are identical (same field values, same ordering)

#### Test: `test_tub_without_images`
- **Setup:** Create tub with `image_array` field but use minimal
  1x1 pixel images (to keep tests fast)
- **Assert:** Image size doesn't affect ranking calculations. Rankings
  are identical to full-size image tubs.

### 5.6 Segment Performance with On-the-Fly Computation

**Class:** `TestOnTheFlySegmentComputation`

**Purpose:** Verify that segment rankings work when segments are
computed on-the-fly from metadata (no `car/segment` in records).

#### Test: `test_ranking_from_metadata_matches_record_field`
- **Setup:** Create tub WITH `car/segment` in records. Compute
  rankings. Then create identical tub WITHOUT `car/segment` but with
  segmentation metadata in manifest. Compute rankings.
- **Assert:** Both produce identical ranking structures and values.

#### Test: `test_ranking_with_segmentation_metadata_only`
- **Setup:** Create realistic tub with `car/pos` and `car/euler` but
  no `car/segment`. Store segmentation metadata (mean_course,
  segment_boundaries) in manifest. Run `calculate_segment_performance()`.
- **Assert:** Rankings computed successfully using on-the-fly segment
  assignment from position data.

### 5.7 Edge Cases and Error Handling

**Class:** `TestRankingEdgeCases`

#### Test: `test_single_lap_no_ranking`
- **Setup:** 1 lap only (after filtering lap 0)
- **Assert:** Single lap gets ranking 1.0 for all fields (100th
  percentile of 1)

#### Test: `test_missing_field_in_record`
- **Setup:** Records missing `car/accel` field
- **Config:** Includes `{car/accel, index=0, avg}`
- **Assert:** Missing field gracefully handled. Other fields still
  ranked. Missing field gets default ranking (0.5).

#### Test: `test_field_with_none_values`
- **Setup:** Some records have `car/speed = None`
- **Config:** `{car/speed, avg}`
- **Assert:** None values skipped in accumulation. Average computed
  from non-None values only.

#### Test: `test_all_segments_same_performance`
- **Setup:** 3 laps where all laps have identical time, distance, and
  gyro for each segment
- **Assert:** Rankings are still valid (assigned evenly). No division
  by zero or degenerate behavior.

## 6. Data Generation Design

### 6.1 Driving Profiles

Pre-defined profiles that create realistic sensor correlations:

```python
PROFILES = {
    'straight_fast': {
        'speed': 12.0,       # m/s
        'gyro_z_base': 0.02, # Near zero (straight)
        'gyro_z_noise': 0.01,
        'accel_x_base': 0.5, # Slight positive (accelerating)
        'accel_y_base': 0.0, # No lateral force
        'gyro_x_base': 0.0,  # No roll
    },
    'turn_smooth': {
        'speed': 8.0,
        'gyro_z_base': 0.8,  # Turning
        'gyro_z_noise': 0.05,# Low noise = smooth
        'accel_x_base': -0.5,# Slight braking into turn
        'accel_y_base': 2.0, # Centripetal acceleration
        'gyro_x_base': 0.1,  # Slight roll
    },
    'turn_aggressive': {
        'speed': 10.0,
        'gyro_z_base': 1.2,  # Sharp turning
        'gyro_z_noise': 0.3, # High noise = corrections
        'accel_x_base': -3.0,# Hard braking
        'accel_y_base': 4.0, # High lateral G
        'gyro_x_base': 0.3,  # More roll
    },
    'chicane': {
        'speed': 7.0,
        'gyro_z_base': 0.0,  # Alternates L/R
        'gyro_z_noise': 0.8, # Sign alternates
        'accel_x_base': -1.0,
        'accel_y_base': 0.0, # Alternates L/R
        'gyro_x_base': 0.2,
    },
}
```

### 6.2 Record Generation

Each record is generated as a function of:
- The driving profile (behavioral characteristics)
- The segment progress (0.0 to 1.0 within segment)
- Optional per-record noise

This ensures sensor values are correlated the way real sensors would
be, not independently randomized.

### 6.3 Tub Schema

Records match the real `donkey5` template schema:

```python
inputs = [
    'cam/image_array',
    'user/angle', 'user/throttle',
    'car/lap', 'car/segment',
    'car/gyro', 'car/accel', 'car/speed',
    'car/distance', 'car/pos', 'car/euler',
]
types = [
    'image_array',
    'float', 'float',
    'int', 'int',
    'vector', 'vector', 'float',
    'float', 'vector', 'vector',
]
```

Images: 1x1 pixel numpy arrays (`np.zeros((1, 1, 3), dtype=np.uint8)`)

## 7. Implementation Notes

### 7.1 Avoid Duplication

- Do NOT test basic `FieldAggregationSpec` construction (covered in
  `test_tub_statistics.py`)
- Do NOT test segment assignment boundary crossing (covered in
  `test_segment_assignment.py`)
- Do NOT test segment invariant (covered in
  `test_segment_statistics_comprehensive.py`)
- Do NOT test PctMode enum switching (covered in
  `test_lap_pct_regression.py`)
- DO test the combinations and interactions that existing tests skip

### 7.2 Test Data Correctness

Every test must verify correctness, not just validity:

```python
# BAD: Valid but meaningless
assert 0 <= ranking <= 1

# GOOD: Correct ranking order
assert ranking_smooth < ranking_jerky

# GOOD: Exact expected value
assert lap_pct == [time_rank, dist_rank, gyro_rank]
```

### 7.3 Shared Setup

Use `setUp` to create a standard multi-lap tub via the shared
generator. Individual tests can create additional tubs for specific
scenarios. Use `tempfile.mkdtemp()` and clean up in `tearDown`.

### 7.4 Laptop Metadata

Tests must create laptimer metadata for their tubs, since
`calculate_lap_performance()` and `calculate_segment_performance()`
require it. Follow the pattern in existing tests:

```python
tub.manifest.metadata[session_id] = {
    'laptimer': [
        {'lap': i, 'time': t, 'distance': d, 'valid': True}
        for i, (t, d) in enumerate(lap_data)
    ]
}
tub.manifest.write_metadata()
```

## 8. Test Matrix Summary

| Test Class | # Tests | Key Aggregation | Key Insight |
|-----------|---------|-----------------|-------------|
| AggregationMethods | 8 | avg/sum/min/max/median/delta | Each method works correctly with realistic data |
| MultiFieldPriority | 5 | 2-4 fields, ordering | Priority order determines ranking; secondary breaks ties |
| BehaviorDiscrimination | 4 | correlated multi-field | System distinguishes meaningful driving styles |
| TransformFunctions | 4 | abs/square/identity/reverse | Transforms applied before aggregation |
| TubWriterIntegration | 3 | any | Official recording interface produces valid rankings |
| OnTheFlyComputation | 2 | any | Metadata-based segment computation matches record field |
| RankingEdgeCases | 4 | various | Graceful handling of degenerate inputs |

**Total: 30 new tests**

## 9. Success Criteria

1. All 30 tests pass
2. No test duplicates assertions from existing test files
3. All test data is created through `Tub.write_record()` or
   `TubWriter.run()` -- never by directly manipulating internal state
4. Every assertion checks correctness (expected ordering or value),
   not just validity (in range)
5. Tests run in < 30 seconds total (no heavy computation or large data)
6. Sensor data in tests is physically plausible (correlated, not random)
