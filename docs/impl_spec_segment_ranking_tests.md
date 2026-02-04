# Implementation Spec: Comprehensive Segment Ranking Tests

**PRD:** `docs/prd_segment_ranking_tests.md`

This document specifies the exact implementation of each file, function,
class, and test from the PRD. It provides code-level detail sufficient to
implement without ambiguity.

## 1. File Layout

```
donkeycar/tests/
    tub_test_data_generator.py          # New: shared data generator
    test_segment_ranking_comprehensive.py  # New: all 30 tests
```

No other files are created or modified.

---

## 2. `tub_test_data_generator.py` — Shared Data Generator

### 2.1 Module Purpose

Provides reusable functions to create realistic multi-lap tub data
through the official `Tub.write_record()` and `TubWriter.run()`
interfaces. The module is NOT a test file — it contains no test classes
or test functions. It is imported by test files.

### 2.2 Constants

```python
import numpy as np

# Minimal image to satisfy image_array type without I/O overhead
TINY_IMAGE = np.zeros((1, 1, 3), dtype=np.uint8)

# Full tub schema matching donkey5 template
TUB_INPUTS = [
    'cam/image_array',
    'user/angle', 'user/throttle',
    'car/lap', 'car/segment',
    'car/gyro', 'car/accel', 'car/speed',
    'car/distance', 'car/pos', 'car/euler',
]
TUB_TYPES = [
    'image_array',
    'float', 'float',
    'int', 'int',
    'vector', 'vector', 'float',
    'float', 'vector', 'vector',
]

# Minimal schema for tests that don't need all fields
MINIMAL_INPUTS = [
    'car/lap', 'car/distance', 'car/gyro', 'car/accel', 'car/speed',
]
MINIMAL_TYPES = [
    'int', 'float', 'vector', 'vector', 'float',
]
```

### 2.3 `DrivingProfile` dataclass

```python
from dataclasses import dataclass, field
from typing import Optional, Callable, List

@dataclass
class DrivingProfile:
    """
    Parameterizes driving behavior for one segment.

    All values are physical quantities, not raw sensor readings.
    The record generator converts these into correlated sensor
    values.
    """
    speed: float = 8.0          # m/s average through segment
    gyro_z_base: float = 0.0    # rad/s yaw rate (+ = left turn)
    gyro_z_noise: float = 0.05  # rad/s noise amplitude
    accel_x_base: float = 0.0   # m/s^2 longitudinal (+ = accel)
    accel_x_profile: str = 'constant'
        # 'constant' | 'brake_then_accel' | 'accel_then_brake'
    accel_y_base: float = 0.0   # m/s^2 lateral (centripetal)
    gyro_x_base: float = 0.0    # rad/s roll rate
    speed_delta: float = 0.0    # m/s speed change across segment
        # end_speed = speed + speed_delta
```

**Design rationale:** `accel_x_profile` controls the longitudinal
acceleration shape across the segment so that `min` and `max`
aggregations produce predictable results. `speed_delta` controls the
`delta` aggregation result for speed.

### 2.4 Pre-defined Profiles

```python
STRAIGHT_FAST = DrivingProfile(
    speed=12.0,
    gyro_z_base=0.02, gyro_z_noise=0.01,
    accel_x_base=0.5, accel_y_base=0.0,
    gyro_x_base=0.0,
)

STRAIGHT_SLOW = DrivingProfile(
    speed=6.0,
    gyro_z_base=0.02, gyro_z_noise=0.01,
    accel_x_base=0.2, accel_y_base=0.0,
    gyro_x_base=0.0,
)

TURN_SMOOTH = DrivingProfile(
    speed=8.0,
    gyro_z_base=0.8, gyro_z_noise=0.05,
    accel_x_base=-0.5, accel_y_base=2.0,
    gyro_x_base=0.1,
)

TURN_AGGRESSIVE = DrivingProfile(
    speed=10.0,
    gyro_z_base=1.2, gyro_z_noise=0.3,
    accel_x_base=-3.0, accel_y_base=4.0,
    gyro_x_base=0.3,
)

TURN_JERKY = DrivingProfile(
    speed=8.0,        # Same speed as TURN_SMOOTH
    gyro_z_base=0.8, gyro_z_noise=0.4,  # Same base, much noisier
    accel_x_base=-1.0, accel_y_base=3.0,
    gyro_x_base=0.2,
)

BRAKING_HARD = DrivingProfile(
    speed=10.0,
    gyro_z_base=0.3, gyro_z_noise=0.05,
    accel_x_base=-8.0, accel_x_profile='brake_then_accel',
    accel_y_base=1.0, gyro_x_base=0.1,
    speed_delta=-4.0,
)

BRAKING_GENTLE = DrivingProfile(
    speed=10.0,
    gyro_z_base=0.3, gyro_z_noise=0.05,
    accel_x_base=-2.0, accel_x_profile='brake_then_accel',
    accel_y_base=1.0, gyro_x_base=0.1,
    speed_delta=-2.0,
)

CHICANE = DrivingProfile(
    speed=7.0,
    gyro_z_base=0.0, gyro_z_noise=0.8,
    accel_x_base=-1.0, accel_y_base=0.0,
    gyro_x_base=0.2,
)
```

### 2.5 `generate_sensor_record()`

```python
def generate_sensor_record(
    profile: DrivingProfile,
    progress: float,        # 0.0 to 1.0 within segment
    record_index: int,      # For deterministic noise seeding
) -> dict:
    """
    Generate correlated sensor values from a driving profile.

    :param profile: Driving behavior parameters
    :param progress: Position within segment (0.0 = start, 1.0 = end)
    :param record_index: Global record index for deterministic noise
    :return: dict with keys: car/gyro, car/accel, car/speed
             (NOT car/lap, car/segment, etc. — caller adds those)
    """
```

**Implementation logic:**

```
speed:
    base = profile.speed + progress * profile.speed_delta
    (linear interpolation from start to end speed)

gyro_z:
    value = profile.gyro_z_base
    noise = profile.gyro_z_noise * sin(record_index * 7.3)
    # Deterministic pseudo-noise, not random, for reproducibility
    result = value + noise

gyro_x:
    value = profile.gyro_x_base

gyro = [gyro_x, 0.0, gyro_z]
    # Index 0 = roll, index 1 = pitch (near 0), index 2 = yaw

accel_x:
    if profile == 'constant':
        value = profile.accel_x_base
    elif profile == 'brake_then_accel':
        # First half: braking (negative), second half: accelerating
        if progress < 0.5:
            value = profile.accel_x_base  # e.g., -8.0
        else:
            value = -profile.accel_x_base * 0.3  # e.g., +2.4
    elif profile == 'accel_then_brake':
        # Reverse of above
        if progress < 0.5:
            value = -profile.accel_x_base * 0.3
        else:
            value = profile.accel_x_base

accel_y:
    # Lateral acceleration correlates with speed^2 * curvature
    # Approximate: scale base by (speed / 8.0)^2
    speed_factor = (speed / 8.0) ** 2
    value = profile.accel_y_base * speed_factor

accel = [accel_x, accel_y, 0.0]
    # Index 0 = longitudinal, index 1 = lateral, index 2 = vertical
```

**Key design decisions:**
- Uses `sin(record_index * 7.3)` for deterministic pseudo-noise, not
  `random`. This means tests produce identical data on every run.
- Sensor correlations are physically motivated: lateral accel scales
  with speed squared, braking profile creates predictable min/max.
- Returns only sensor fields. Caller is responsible for `car/lap`,
  `car/segment`, `_timestamp_ms`, `car/distance`, etc.

### 2.6 `create_multilap_tub()`

```python
def create_multilap_tub(
    tub_path: str,
    segment_profiles: List[List[DrivingProfile]],
        # Outer list = laps, inner list = segments per lap
        # segment_profiles[lap_idx][segment_idx] = DrivingProfile
    records_per_segment: int = 20,
    segment_time_ms: int = 2000,
    inputs: List[str] = None,   # Default: MINIMAL_INPUTS
    types: List[str] = None,    # Default: MINIMAL_TYPES
) -> 'Tub':
    """
    Create a multi-lap tub with per-segment driving profiles.

    Records are written through Tub.write_record(). Laptimer metadata
    is generated automatically from the written data.

    :param tub_path: Directory path for the tub
    :param segment_profiles: Per-lap, per-segment driving profiles.
           All laps must have the same number of segments.
    :param records_per_segment: Number of records per segment
    :param segment_time_ms: Duration of each segment in milliseconds
    :param inputs: Tub input field names (default: MINIMAL_INPUTS)
    :param types: Tub input types (default: MINIMAL_TYPES)
    :return: The created Tub (still open, caller must close)
    """
```

**Implementation logic:**

```
1. Validate: all laps have same number of segments
2. num_laps = len(segment_profiles)
3. num_segments = len(segment_profiles[0])
4. tub = Tub(tub_path, inputs, types)
5. start_time_ms = 1000000  # Fixed start for determinism
6. cumulative_distance = 0.0
7. global_record_idx = 0

8. For each lap_idx in range(num_laps):
     For each seg_idx in range(num_segments):
       profile = segment_profiles[lap_idx][seg_idx]
       For each rec_idx in range(records_per_segment):
         progress = rec_idx / records_per_segment
         timestamp = start_time_ms + global_record_idx * (
             segment_time_ms // records_per_segment)

         sensor = generate_sensor_record(profile, progress,
                                         global_record_idx)
         distance_step = (profile.speed *
             segment_time_ms / 1000.0 / records_per_segment)
         cumulative_distance += distance_step

         record = {
             'car/lap': lap_idx,
             'car/distance': cumulative_distance,
             'car/gyro': sensor['car/gyro'],
             'car/accel': sensor['car/accel'],
             'car/speed': sensor['car/speed'],
             '_timestamp_ms': timestamp,
         }
         # Only include car/segment if in tub schema
         if 'car/segment' in inputs:
             record['car/segment'] = seg_idx
         # Only include image if in tub schema
         if 'cam/image_array' in inputs:
             record['cam/image_array'] = TINY_IMAGE

         tub.write_record(record)
         global_record_idx += 1

   # Final record to close last lap
   tub.write_record({
       'car/lap': num_laps,
       'car/distance': cumulative_distance,
       'car/gyro': [0.0, 0.0, 0.0],
       'car/accel': [0.0, 0.0, 0.0],
       'car/speed': 0.0,
       '_timestamp_ms': start_time_ms +
           global_record_idx * (segment_time_ms // records_per_segment),
       **({'car/segment': 0} if 'car/segment' in inputs else {}),
       **({'cam/image_array': TINY_IMAGE}
          if 'cam/image_array' in inputs else {}),
   })

9. Generate laptimer metadata from written records:
   session_id = tub.manifest.session_id[1]
   stats = TubStatistics(tub, field_aggregations=<boundary only>)
   stats.generate_laptimes_from_records()

10. Return tub
```

**Key design decisions:**
- Timestamps increment linearly. Each segment takes exactly
  `segment_time_ms` milliseconds, so lap time = `num_segments *
  segment_time_ms / 1000.0` seconds. This makes time-based ranking
  results predictable UNLESS the profiles vary segment times.
- Distance accumulates based on `profile.speed * segment_duration`.
  Different speeds → different distances per segment.
- Laptimer metadata is generated automatically using
  `TubStatistics.generate_laptimes_from_records()` so tests don't
  need to construct it manually.

### 2.7 `create_multilap_tub_via_writer()`

```python
def create_multilap_tub_via_writer(
    tub_path: str,
    segment_profiles: List[List[DrivingProfile]],
    records_per_segment: int = 20,
    segment_time_ms: int = 2000,
) -> 'Tub':
    """
    Same as create_multilap_tub but uses TubWriter.run() interface.

    Uses the full TUB_INPUTS/TUB_TYPES schema with images.
    Returns a closed, read-only Tub.
    """
```

**Implementation logic:**

```
1. writer = TubWriter(tub_path, TUB_INPUTS, TUB_TYPES)
2. For each record (same loop as create_multilap_tub):
     # TubWriter.run() takes positional args in input order
     writer.run(
         TINY_IMAGE,           # cam/image_array
         0.0,                  # user/angle
         0.5,                  # user/throttle
         lap_idx,              # car/lap
         seg_idx,              # car/segment
         sensor['car/gyro'],   # car/gyro
         sensor['car/accel'],  # car/accel
         sensor['car/speed'],  # car/speed
         cumulative_distance,  # car/distance
         [0.0, 0.0, 0.0],     # car/pos (placeholder)
         [0.0, 0.0, 0.0],     # car/euler (placeholder)
     )
3. writer.close()
4. Reopen as read-only Tub
5. Generate laptimer metadata
6. Return tub
```

**Note:** `TubWriter.run()` does not accept `_timestamp_ms` — it uses
`time.time()`. For the TubWriter integration tests, we do NOT need
deterministic timestamps because we only compare ranking *order*, not
exact time values. The laptimer is generated from records after writing.

### 2.8 `create_tub_with_varied_segment_times()`

```python
def create_tub_with_varied_segment_times(
    tub_path: str,
    num_laps: int,
    num_segments: int,
    lap_segment_times_ms: List[List[int]],
        # lap_segment_times_ms[lap][segment] = duration_ms
    base_profile: DrivingProfile = None,
    records_per_segment: int = 20,
    inputs: List[str] = None,
    types: List[str] = None,
) -> 'Tub':
    """
    Create tub where each segment instance has a specific duration.

    Used for tests that need precise control over time-based rankings
    (e.g., test_primary_sort_key_dominates). The profile is the same
    for all segments but the time between records varies to produce
    different segment durations.

    :param lap_segment_times_ms: Per-lap, per-segment duration in ms.
    :param base_profile: Profile for sensor values (default: TURN_SMOOTH)
    """
```

**Implementation logic:**
Same as `create_multilap_tub` but uses `lap_segment_times_ms` to
control `_timestamp_ms` spacing instead of a fixed `segment_time_ms`.
This gives precise control over the `time` boundary field in rankings.

---

## 3. `test_segment_ranking_comprehensive.py` — Test File

### 3.1 Module Header

```python
"""
Comprehensive tests for the segment-based ranking system.

Tests the combinatorial space of aggregation methods, multi-field
ranking interactions, driving behavior discrimination, and transform
functions. Complements existing test files by testing what they do
NOT cover (see PRD gap analysis in docs/prd_segment_ranking_tests.md).

All test data created through official Tub.write_record() or
TubWriter.run() interfaces. No internal state manipulation.
"""
import math
import os
import shutil
import tempfile
import unittest

import numpy as np

from donkeycar.parts.tub_v2 import Tub, TubWriter
from donkeycar.parts.tub_statistics import (
    TubStatistics, FieldAggregationSpec,
)
from donkeycar.pipeline.transformations import SortingStrategy
from donkeycar.pipeline.types import PctMode

from donkeycar.tests.tub_test_data_generator import (
    DrivingProfile,
    STRAIGHT_FAST, STRAIGHT_SLOW,
    TURN_SMOOTH, TURN_AGGRESSIVE, TURN_JERKY,
    BRAKING_HARD, BRAKING_GENTLE, CHICANE,
    MINIMAL_INPUTS, MINIMAL_TYPES,
    create_multilap_tub,
    create_multilap_tub_via_writer,
    create_tub_with_varied_segment_times,
)
```

### 3.2 Common Base Class

```python
class SegmentRankingTestBase(unittest.TestCase):
    """Base class providing tub lifecycle management."""

    def setUp(self):
        self.test_dirs = []

    def tearDown(self):
        for d in self.test_dirs:
            if os.path.exists(d):
                shutil.rmtree(d)

    def _make_tub_dir(self) -> str:
        d = tempfile.mkdtemp()
        self.test_dirs.append(d)
        return d

    def _rank_laps(self, tub, field_aggregations,
                   sorting_strategy=None, use_lap_0=True):
        """
        Convenience: generate laptimes, aggregate fields, rank.

        Returns {session_id: {lap: {key: ranking}}}
        """
        stats = TubStatistics(
            tub,
            field_aggregations=field_aggregations,
            sorting_strategy=sorting_strategy,
        )
        stats.generate_laptimes_from_records()
        stats._calculate_aggregated_fields()
        return stats.calculate_lap_performance(use_lap_0=use_lap_0)

    def _rank_segments(self, tub, field_aggregations,
                       sorting_strategy=None, use_lap_0=True):
        """
        Convenience: generate laptimes, aggregate fields,
        rank segments.

        Returns {session_id: {lap: {segment: {key: ranking}}}}
        """
        stats = TubStatistics(
            tub,
            field_aggregations=field_aggregations,
            sorting_strategy=sorting_strategy,
        )
        stats.generate_laptimes_from_records()
        return stats.calculate_segment_performance(
            use_lap_0=use_lap_0)

    def _get_session_id(self, tub):
        return tub.manifest.session_id[1]
```

### 3.3 Class: `TestFieldAggregationMethods`

All tests use `_rank_laps()` with 3 laps to verify aggregation
correctness through ranking order.

#### `test_avg_aggregation_gyro_smoothness`

```
Setup:
    profiles = [
        [TURN_SMOOTH, STRAIGHT_FAST],          # Lap 0: low avg gyro
        [TURN_AGGRESSIVE, STRAIGHT_FAST],      # Lap 1: high avg gyro
        [TURN_JERKY, STRAIGHT_FAST],           # Lap 2: medium avg gyro
    ]
    tub = create_multilap_tub(path, profiles)
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs)
    s = _get_session_id(tub)
    # TURN_SMOOTH has lowest avg(abs(gyro_z)), should rank best
    assert perf[s][0]['gyro_z_agg'] < perf[s][1]['gyro_z_agg']
    # TURN_AGGRESSIVE has highest base gyro, should rank worst
```

**Why this works:** `TURN_SMOOTH.gyro_z_base=0.8, noise=0.05` produces
avg(abs) around 0.8. `TURN_AGGRESSIVE.gyro_z_base=1.2, noise=0.3`
produces avg(abs) around 1.2. The ranking should order them accordingly.

#### `test_sum_aggregation_total_yaw`

```
Setup:
    # 3 laps, 1 segment each (simplest case for sum)
    profiles = [
        [DrivingProfile(gyro_z_base=0.1, gyro_z_noise=0.01)],  # Low sum
        [DrivingProfile(gyro_z_base=0.5, gyro_z_noise=0.01)],  # Medium
        [DrivingProfile(gyro_z_base=1.0, gyro_z_noise=0.01)],  # High sum
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='yaw_sum',
            transform=abs, aggregation='sum'),
    ]

Assert:
    # Lap with gyro_z_base=0.1 has lowest sum → best ranking
    assert perf[s][0]['yaw_sum'] < perf[s][1]['yaw_sum']
    assert perf[s][1]['yaw_sum'] < perf[s][2]['yaw_sum']
```

#### `test_min_aggregation_hardest_braking`

```
Setup:
    profiles = [
        [BRAKING_HARD],    # Lap 0: accel_x_base = -8.0
        [BRAKING_GENTLE],  # Lap 1: accel_x_base = -2.0
        [TURN_SMOOTH],     # Lap 2: accel_x_base = -0.5
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/accel', index=0, output_key='brake_min',
            transform=None, aggregation='min'),
    ]

Assert:
    # Default ascending sort: most negative min ranks first
    # BRAKING_HARD (-8.0) < BRAKING_GENTLE (-2.0) < TURN_SMOOTH (-0.5)
    assert perf[s][0]['brake_min'] < perf[s][1]['brake_min']
    assert perf[s][1]['brake_min'] < perf[s][2]['brake_min']
    # i.e., lap 0 has rank ~0.33, lap 1 ~0.67, lap 2 ~1.0
```

#### `test_max_aggregation_peak_acceleration`

```
Setup:
    profiles = [
        [DrivingProfile(accel_x_base=5.0)],   # High peak
        [DrivingProfile(accel_x_base=3.0)],   # Medium peak
        [DrivingProfile(accel_x_base=1.0)],   # Low peak
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/accel', index=0, output_key='accel_peak',
            transform=None, aggregation='max'),
    ]

Assert:
    # Ascending sort: lowest max ranks first
    assert perf[s][2]['accel_peak'] < perf[s][1]['accel_peak']
    assert perf[s][1]['accel_peak'] < perf[s][0]['accel_peak']
```

#### `test_median_aggregation_typical_speed`

```
Setup:
    # 3 laps with different speed distributions.
    # Use custom DrivingProfiles with specific speed/speed_delta to
    # control the median.
    #
    # Lap 0: consistent 10 m/s (median ≈ 10)
    # Lap 1: mostly 10, with speed_delta=5 (median ≈ 12.5)
    # Lap 2: starts at 5, speed_delta=15 → ends at 20 (median ≈ 12.5)
    #
    # Actually, median of linearly spaced values = midpoint.
    # So we need non-linear profiles. Better approach:
    #
    # Use 3 single-segment laps with different speed bases:
    profiles = [
        [DrivingProfile(speed=5.0, speed_delta=0.0)],   # Median=5
        [DrivingProfile(speed=10.0, speed_delta=0.0)],  # Median=10
        [DrivingProfile(speed=15.0, speed_delta=0.0)],  # Median=15
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/speed', output_key='speed_med',
            aggregation='median'),
    ]

Assert:
    # Ascending: lowest median speed ranks first
    assert perf[s][0]['speed_med'] < perf[s][1]['speed_med']
    assert perf[s][1]['speed_med'] < perf[s][2]['speed_med']
```

#### `test_delta_aggregation_speed_change`

```
Setup:
    profiles = [
        [DrivingProfile(speed=8.0, speed_delta=4.0)],   # 8→12, delta=+4
        [DrivingProfile(speed=10.0, speed_delta=0.0)],  # 10→10, delta=0
        [DrivingProfile(speed=12.0, speed_delta=-4.0)], # 12→8, delta=-4
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/speed', output_key='speed_delta',
            aggregation='delta'),
    ]

Assert:
    # Delta = last - first
    # Ascending sort: most negative delta ranks first
    # Lap 2 (delta=-4) < Lap 1 (delta=0) < Lap 0 (delta=+4)
    assert perf[s][2]['speed_delta'] < perf[s][1]['speed_delta']
    assert perf[s][1]['speed_delta'] < perf[s][0]['speed_delta']
```

#### `test_delta_single_record_segment`

```
Setup:
    Create tub with records_per_segment=1
    profiles = [
        [DrivingProfile(speed=10.0)],
        [DrivingProfile(speed=15.0)],
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/speed', output_key='speed_delta',
            aggregation='delta'),
    ]

Assert:
    # With 1 record, delta = value - value = 0 for both laps
    assert perf[s][0]['speed_delta'] == perf[s][1]['speed_delta']
```

**Implementation note:** `FieldAccumulator.compute()` for delta with 1
value returns `self.values[-1] - self.values[0]` = `v - v` = 0. Both
laps get identical rankings.

#### `test_median_even_count`

```
Setup:
    Create tub with records_per_segment=4
    # Lap 0: 4 speed values will be deterministic from profile
    profiles = [
        [DrivingProfile(speed=10.0, speed_delta=0.0)],
        [DrivingProfile(speed=20.0, speed_delta=0.0)],
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/speed', output_key='speed_med',
            aggregation='median'),
    ]

Assert:
    # With 4 values, median = sorted_vals[2] (index 4//2 = 2)
    # Both laps should have consistent speed, so median ≈ base speed
    # Lap 0 median < Lap 1 median
    assert perf[s][0]['speed_med'] < perf[s][1]['speed_med']
```

### 3.4 Class: `TestMultiFieldRankingPriority`

#### `test_primary_sort_key_dominates`

```
Setup:
    # 3 laps, 2 segments. Control segment times precisely.
    lap_segment_times = [
        [2000, 2000],   # Lap 0: 2.0s per segment
        [1000, 2000],   # Lap 1: 1.0s for seg 0 (fastest)
        [3000, 2000],   # Lap 2: 3.0s for seg 0 (slowest)
    ]
    # All use same profile (same gyro), so gyro ranking is uniform
    base = TURN_SMOOTH
    profiles = [[base, base]] * 3

    tub = create_tub_with_varied_segment_times(
        path, 3, 2, lap_segment_times, base)

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs)
    s = _get_session_id(tub)
    # Lap 1 is fastest → best time ranking
    assert perf[s][1]['time'] < perf[s][0]['time']
    assert perf[s][0]['time'] < perf[s][2]['time']
```

#### `test_secondary_breaks_ties`

```
Setup:
    # 3 laps, 1 segment. All same time but different gyro.
    # Use same segment_time_ms for all, vary profiles.
    profiles = [
        [DrivingProfile(speed=10.0, gyro_z_base=0.1)],  # Smoothest
        [DrivingProfile(speed=10.0, gyro_z_base=0.5)],  # Medium
        [DrivingProfile(speed=10.0, gyro_z_base=0.9)],  # Jerkiest
    ]
    # All same speed → same time and distance → tie on primary

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    # Time rankings should be roughly equal (all same speed)
    # Gyro ranking should differentiate:
    assert perf[s][0]['gyro_z_agg'] < perf[s][1]['gyro_z_agg']
    assert perf[s][1]['gyro_z_agg'] < perf[s][2]['gyro_z_agg']
```

#### `test_four_field_ranking`

```
Setup:
    # 4 laps, 2 segments. Each lap has distinct values for all 4 fields.
    profiles = [
        [DrivingProfile(speed=12, gyro_z_base=0.1, accel_x_base=-1),
         STRAIGHT_FAST],
        [DrivingProfile(speed=10, gyro_z_base=0.3, accel_x_base=-3),
         STRAIGHT_FAST],
        [DrivingProfile(speed=8, gyro_z_base=0.5, accel_x_base=-5),
         STRAIGHT_FAST],
        [DrivingProfile(speed=6, gyro_z_base=0.7, accel_x_base=-7),
         STRAIGHT_FAST],
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
        FieldAggregationSpec(
            field='car/accel', index=0, output_key='brake_min',
            aggregation='min'),
    ]

Assert:
    # Each lap has a ranking dict with all 4 keys
    for lap_idx in range(4):
        lap_rank = perf[s][lap_idx]
        assert 'time' in lap_rank
        assert 'distance' in lap_rank
        assert 'gyro_z_agg' in lap_rank
        assert 'brake_min' in lap_rank
    # Gyro ranking: lap 0 best (0.1), lap 3 worst (0.7)
    assert perf[s][0]['gyro_z_agg'] < perf[s][3]['gyro_z_agg']
    # Brake ranking: lap 0 best (-1, least negative),
    #                lap 3 worst (-7, most negative) in ascending sort
    assert perf[s][3]['brake_min'] < perf[s][0]['brake_min']
```

#### `test_two_field_ranking_time_distance_only`

```
Setup:
    profiles = [
        [DrivingProfile(speed=12.0)],
        [DrivingProfile(speed=8.0)],
        [DrivingProfile(speed=4.0)],
    ]
    # Only boundary fields, no record fields
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
    ]

Assert:
    for lap_idx in range(3):
        lap_rank = perf[s][lap_idx]
        assert len(lap_rank) == 2
        assert 'time' in lap_rank
        assert 'distance' in lap_rank
        # No gyro_z_agg or other keys
        assert 'gyro_z_agg' not in lap_rank
```

#### `test_ranking_order_matches_config_order`

```
Setup:
    profiles = [
        [DrivingProfile(speed=12.0, gyro_z_base=0.9)],  # Fast, jerky
        [DrivingProfile(speed=6.0, gyro_z_base=0.1)],   # Slow, smooth
        [DrivingProfile(speed=9.0, gyro_z_base=0.5)],   # Medium
    ]

    # Config A: time first, then gyro
    aggs_a = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]
    # Config B: gyro first, then time
    aggs_b = [
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
        FieldAggregationSpec(output_key='time'),
    ]

Approach:
    Create two identical tubs. Rank with aggs_a and aggs_b.

    Then simulate TubRecord.extend() to verify lap_pct order:
    - With aggs_a, ranking_keys = ['time', 'gyro_z_agg']
      → lap_pct[0] = time ranking, lap_pct[1] = gyro ranking
    - With aggs_b, ranking_keys = ['gyro_z_agg', 'time']
      → lap_pct[0] = gyro ranking, lap_pct[1] = time ranking

Assert:
    # The ranking values for each key are the same regardless
    # of config order (same data produces same per-field ranking)
    assert perf_a[s][0]['time'] == perf_b[s][0]['time']
    assert perf_a[s][0]['gyro_z_agg'] == perf_b[s][0]['gyro_z_agg']

    # But the lap_pct vector differs because key order differs
    keys_a = ['time', 'gyro_z_agg']
    keys_b = ['gyro_z_agg', 'time']
    pct_a = [perf_a[s][0][k] for k in keys_a]
    pct_b = [perf_b[s][0][k] for k in keys_b]
    # pct_a[0] is time ranking, pct_b[0] is gyro ranking
    # For lap 0 (fast, jerky): best time, worst gyro
    # So pct_a[0] != pct_b[0] (time vs gyro)
    assert pct_a[0] != pct_b[0]
```

### 3.5 Class: `TestDrivingBehaviorDiscrimination`

#### `test_fast_smooth_vs_fast_jerky`

```
Setup:
    # 2 laps, 2 segments.
    # Segment 0: same speed, different smoothness.
    # Segment 1: identical (control).
    smooth_seg0 = DrivingProfile(
        speed=10.0, gyro_z_base=0.8, gyro_z_noise=0.05,
        accel_y_base=2.0)
    jerky_seg0 = DrivingProfile(
        speed=10.0, gyro_z_base=0.8, gyro_z_noise=0.4,
        accel_y_base=3.5)
    neutral = STRAIGHT_FAST

    profiles = [
        [smooth_seg0, neutral],   # Lap 0: smooth
        [jerky_seg0, neutral],    # Lap 1: jerky
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
        FieldAggregationSpec(
            field='car/accel', index=1, output_key='lat_g_max',
            transform=abs, aggregation='max'),
    ]

Assert:
    # Time rankings should be equal (same speed)
    # Gyro: smooth < jerky (smooth has lower avg abs gyro noise)
    assert perf[s][0]['gyro_z_agg'] < perf[s][1]['gyro_z_agg']
    # Lateral G: smooth < jerky (lower accel_y_base)
    assert perf[s][0]['lat_g_max'] < perf[s][1]['lat_g_max']
```

**Note on assertion:** With 2 laps, rankings are 0.5 and 1.0. The
smooth lap should get 0.5 (better) for both gyro and lateral G.

#### `test_slow_stable_vs_fast_unstable`

```
Setup:
    slow_stable = DrivingProfile(
        speed=6.0, gyro_z_base=0.3, gyro_z_noise=0.02)
    fast_unstable = DrivingProfile(
        speed=12.0, gyro_z_base=0.3, gyro_z_noise=0.5)

    profiles = [
        [slow_stable],
        [fast_unstable],
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    # Fast lap ranks better on time (lower ranking value)
    assert perf[s][1]['time'] < perf[s][0]['time']
    # Slow lap ranks better on gyro (lower noise → lower avg abs)
    assert perf[s][0]['gyro_z_agg'] < perf[s][1]['gyro_z_agg']
    # Neither dominates: tradeoff captured in rankings
```

#### `test_tight_racing_line_vs_wide_line`

```
Setup:
    # Same time but different distances (tight = shorter = more
    # lateral G; wide = longer = less lateral G).
    #
    # Tight line: high speed on short distance → same time
    # Wide line: lower speed on longer distance → same time
    #
    # Since distance = speed * time and we want same time:
    # tight: speed=12, wide: speed=12 (same speed, same time,
    # but different distance must come from different paths)
    #
    # Actually: control distance via cumulative_distance in generator.
    # Tight has lower cumulative distance per segment (shorter path).
    # This is controlled by speed * segment_duration / records_per_segment.
    #
    # Simplification: tight = high speed (covers distance quickly),
    # wide = low speed (same segment time, covers less distance per
    # unit time). Wait — that's backwards. Let's think:
    #
    # Use create_tub_with_varied_segment_times to set SAME time
    # for both laps. Then use different speeds to produce different
    # distances. Higher speed = more distance in same time = wider
    # line (longer path).
    #
    # Tight line: lower speed = less distance = shorter path
    # Wide line: higher speed = more distance = longer path
    tight = DrivingProfile(speed=8.0, accel_y_base=3.0)
    wide = DrivingProfile(speed=12.0, accel_y_base=1.0)

    # Both get same segment time
    lap_segment_times = [[2000], [2000]]
    profiles = [[tight], [wide]]

    tub = create_tub_with_varied_segment_times(
        path, 2, 1, lap_segment_times, profiles=profiles)

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
        FieldAggregationSpec(
            field='car/accel', index=1, output_key='lat_g',
            transform=abs, aggregation='max'),
    ]

Assert:
    # Same time → equal time ranking
    # Tight has less distance → better distance ranking
    assert perf[s][0]['distance'] < perf[s][1]['distance']
    # Tight has higher lateral G → worse lat_g ranking (ascending)
    assert perf[s][0]['lat_g'] > perf[s][1]['lat_g']
```

#### `test_aggressive_vs_conservative_braking`

```
Setup:
    profiles = [
        [BRAKING_HARD],    # accel_x_base = -8.0
        [BRAKING_GENTLE],  # accel_x_base = -2.0
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/accel', index=0, output_key='brake_min',
            aggregation='min'),
    ]

Assert:
    # Hard braking has more negative min → ranks first ascending
    assert perf[s][0]['brake_min'] < perf[s][1]['brake_min']
```

### 3.6 Class: `TestTransformFunctions`

#### `test_abs_transform`

```
Setup:
    # Alternating left/right turns: gyro_z alternates sign.
    # Without abs: avg ≈ 0 (symmetric). With abs: avg > 0.
    left_turn = DrivingProfile(gyro_z_base=1.0, gyro_z_noise=0.01)
    right_turn = DrivingProfile(gyro_z_base=-1.0, gyro_z_noise=0.01)

    # We'll create a profile that produces alternating-sign gyro
    # Actually, easier: two laps with opposite gyro_z_base.
    # Lap 0: gyro_z_base = +0.5 → avg(abs) ≈ 0.5
    # Lap 1: gyro_z_base = -0.5 → avg(abs) ≈ 0.5 (same!)
    # Lap 2: gyro_z_base = +1.5 → avg(abs) ≈ 1.5

    profiles = [
        [DrivingProfile(gyro_z_base=0.5)],
        [DrivingProfile(gyro_z_base=-0.5)],
        [DrivingProfile(gyro_z_base=1.5)],
    ]

    aggs_with_abs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_abs',
            transform=abs, aggregation='avg'),
    ]
    aggs_without_abs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_raw',
            transform=None, aggregation='avg'),
    ]

Assert:
    perf_abs = _rank_laps(tub1, aggs_with_abs)
    perf_raw = _rank_laps(tub2, aggs_without_abs)
    # With abs: laps 0 and 1 have same ranking (both avg(abs)≈0.5)
    assert perf_abs[s][0]['gyro_abs'] == perf_abs[s][1]['gyro_abs']
    # Without abs: lap 1 has negative avg → ranks differently
    assert perf_raw[s][0]['gyro_raw'] != perf_raw[s][1]['gyro_raw']
```

#### `test_square_transform`

```
Setup:
    # Lap 0: consistent moderate gyro (0.5)
    #   → avg(x^2) = avg(0.25) = 0.25
    # Lap 1: occasional spikes (high noise)
    #   → avg(x^2) amplifies spikes quadratically
    profiles = [
        [DrivingProfile(gyro_z_base=0.5, gyro_z_noise=0.01)],
        [DrivingProfile(gyro_z_base=0.5, gyro_z_noise=0.5)],
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_sq',
            transform=lambda x: x ** 2, aggregation='avg'),
    ]

Assert:
    # Noisy lap has higher avg(gyro^2) due to spike amplification
    assert perf[s][0]['gyro_sq'] < perf[s][1]['gyro_sq']
```

#### `test_identity_transform`

```
Setup:
    profiles = [
        [DrivingProfile(accel_x_base=-8.0)],   # Most negative
        [DrivingProfile(accel_x_base=-2.0)],   # Moderate
        [DrivingProfile(accel_x_base=1.0)],    # Positive
    ]
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/accel', index=0, output_key='accel_min',
            transform=None, aggregation='min'),
    ]

Assert:
    # No transform → min of raw values
    # Ascending: most negative ranks first
    assert perf[s][0]['accel_min'] < perf[s][1]['accel_min']
    assert perf[s][1]['accel_min'] < perf[s][2]['accel_min']
```

#### `test_reverse_ranking`

```
Setup:
    profiles = [
        [DrivingProfile(speed=5.0)],
        [DrivingProfile(speed=10.0)],
        [DrivingProfile(speed=15.0)],
    ]
    field_aggs_asc = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/speed', output_key='speed_avg',
            aggregation='avg', reverse=False),
    ]
    field_aggs_desc = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/speed', output_key='speed_avg',
            aggregation='avg', reverse=True),
    ]

Assert:
    perf_asc = _rank_laps(tub1, field_aggs_asc)
    perf_desc = _rank_laps(tub2, field_aggs_desc)
    # Ascending: lowest speed (5.0) ranks best
    assert perf_asc[s][0]['speed_avg'] < perf_asc[s][2]['speed_avg']
    # Descending: highest speed (15.0) ranks best
    assert perf_desc[s][2]['speed_avg'] < perf_desc[s][0]['speed_avg']
```

### 3.7 Class: `TestTubWriterIntegration`

#### `test_tub_writer_produces_valid_rankings`

```
Setup:
    profiles = [
        [TURN_SMOOTH, STRAIGHT_FAST],
        [TURN_AGGRESSIVE, STRAIGHT_FAST],
        [TURN_JERKY, STRAIGHT_FAST],
    ]
    tub = create_multilap_tub_via_writer(path, profiles)

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs)
    s = _get_session_id(tub)
    # Rankings exist for all 3 laps
    assert len(perf[s]) == 3
    # Each lap has all 3 ranking keys
    for lap_idx in range(3):
        assert set(perf[s][lap_idx].keys()) == {
            'time', 'distance', 'gyro_z_agg'}
        # Rankings in valid range
        for key in perf[s][lap_idx]:
            r = perf[s][lap_idx][key]
            assert 0 < r <= 1.0
```

#### `test_tub_writer_vs_direct_write_consistency`

```
Setup:
    profiles = [
        [TURN_SMOOTH],
        [TURN_AGGRESSIVE],
        [TURN_JERKY],
    ]
    tub_direct = create_multilap_tub(path1, profiles)
    tub_writer = create_multilap_tub_via_writer(path2, profiles)

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf_d = _rank_laps(tub_direct, field_aggs)
    perf_w = _rank_laps(tub_writer, field_aggs)
    sd = _get_session_id(tub_direct)
    sw = _get_session_id(tub_writer)

    # Gyro rankings should be in same ORDER (not necessarily same
    # exact values, since timestamps differ)
    gyro_order_d = sorted(range(3),
                          key=lambda i: perf_d[sd][i]['gyro_z_agg'])
    gyro_order_w = sorted(range(3),
                          key=lambda i: perf_w[sw][i]['gyro_z_agg'])
    assert gyro_order_d == gyro_order_w
```

**Note:** We compare ranking ORDER, not exact values, because TubWriter
uses `time.time()` for timestamps (different from the deterministic
timestamps in `create_multilap_tub`).

#### `test_tub_without_images`

```
Setup:
    # Create tub with MINIMAL schema (no image field)
    profiles = [
        [TURN_SMOOTH],
        [TURN_AGGRESSIVE],
    ]
    tub = create_multilap_tub(path, profiles,
                              inputs=MINIMAL_INPUTS,
                              types=MINIMAL_TYPES)
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs)
    s = _get_session_id(tub)
    # Rankings computed successfully without image field
    assert len(perf[s]) == 2
    # TURN_SMOOTH ranks better on gyro
    assert perf[s][0]['gyro_z_agg'] < perf[s][1]['gyro_z_agg']
```

### 3.8 Class: `TestOnTheFlySegmentComputation`

These tests verify that segment rankings computed from metadata
(on-the-fly) match those computed from `car/segment` record field.

#### `test_ranking_from_metadata_matches_record_field`

```
Setup:
    # Create tub WITH car/segment in records
    segment_inputs = MINIMAL_INPUTS + ['car/segment']
    segment_types = MINIMAL_TYPES + ['int']
    profiles = [
        [TURN_SMOOTH, STRAIGHT_FAST],
        [TURN_AGGRESSIVE, STRAIGHT_FAST],
        [TURN_JERKY, STRAIGHT_FAST],
    ]
    tub_with_seg = create_multilap_tub(
        path1, profiles,
        inputs=segment_inputs, types=segment_types)

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf = _rank_segments(tub_with_seg, field_aggs)
    s = _get_session_id(tub_with_seg)
    # Segment rankings exist for all 3 laps, 2 segments each
    for lap in range(3):
        assert lap in perf[s]
        assert len(perf[s][lap]) == 2  # 2 segments
        for seg in range(2):
            assert seg in perf[s][lap]
```

**Note on on-the-fly computation:** The full on-the-fly test requires
segmentation metadata with mean_course and segment_boundaries, which
requires the course_analysis module. This test verifies the record-field
path works correctly. A separate test for the metadata path would need
to create realistic position data and run the segmentation pipeline —
this is already covered by `test_segment_command.py`. We don't duplicate
that here.

#### `test_segment_rankings_structure_with_multiple_fields`

```
Setup:
    Same as above but with 4 ranking fields.

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
        FieldAggregationSpec(
            field='car/accel', index=0, output_key='brake_min',
            aggregation='min'),
    ]

Assert:
    perf = _rank_segments(tub, field_aggs)
    s = _get_session_id(tub)
    # Each segment ranking dict has all 4 keys
    for lap in range(3):
        for seg in range(2):
            rank = perf[s][lap][seg]
            assert set(rank.keys()) == {
                'time', 'distance', 'gyro_z_agg', 'brake_min'}
```

### 3.9 Class: `TestRankingEdgeCases`

#### `test_single_lap_ranking`

```
Setup:
    # 2 laps (0 and 1), skip lap 0 → 1 lap for ranking
    profiles = [
        [TURN_SMOOTH],
        [TURN_AGGRESSIVE],
    ]
    tub = create_multilap_tub(path, profiles)
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs, use_lap_0=False)
    s = _get_session_id(tub)
    # Only lap 1 is ranked
    assert len(perf[s]) == 1
    assert 1 in perf[s]
    # Single lap → ranking = 1.0 (100th percentile of 1)
    assert perf[s][1]['time'] == 1.0
    assert perf[s][1]['gyro_z_agg'] == 1.0
```

#### `test_missing_field_in_record`

```
Setup:
    # Create tub WITHOUT car/accel field
    inputs = ['car/lap', 'car/distance', 'car/gyro', 'car/speed']
    types = ['int', 'float', 'vector', 'float']
    profiles = [
        [DrivingProfile(speed=10.0)],
        [DrivingProfile(speed=8.0)],
    ]
    tub = create_multilap_tub(path, profiles, inputs=inputs,
                              types=types)

    # Config references car/accel which doesn't exist in tub
    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/accel', index=0, output_key='accel_agg',
            aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs)
    s = _get_session_id(tub)
    # Time rankings should still work
    assert 'time' in perf[s][0]
    # accel_agg: FieldAggregationSpec.extract() returns None for
    # missing field → FieldAccumulator has empty values → compute()
    # returns None → _update_field_metadata sets valid=False.
    # SortingStrategy.rank_laps() handles None with float('inf')
    # → assigns default ranking 0.5
    assert 'accel_agg' in perf[s][0]
```

#### `test_field_with_none_values`

```
Setup:
    # Manually create tub where some records have car/speed = None
    inputs = ['car/lap', 'car/distance', 'car/gyro', 'car/speed']
    types = ['int', 'float', 'vector', 'float']
    tub = Tub(path, inputs, types)

    start_time_ms = 1000000
    for lap in range(2):
        for i in range(10):
            speed = 10.0 if i % 3 != 0 else None
            record = {
                'car/lap': lap,
                'car/distance': float(lap * 50 + i * 5),
                'car/gyro': [0.0, 0.0, 0.5],
                'car/speed': speed,
                '_timestamp_ms': start_time_ms + lap * 10000 + i * 1000,
            }
            tub.write_record(record)
    # Final record
    tub.write_record({
        'car/lap': 2,
        'car/distance': 100.0,
        'car/gyro': [0.0, 0.0, 0.5],
        'car/speed': 10.0,
        '_timestamp_ms': start_time_ms + 20000,
    })

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/speed', output_key='speed_avg',
            aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs)
    s = _get_session_id(tub)
    # Rankings computed despite None values
    assert 'speed_avg' in perf[s][0]
    assert 'speed_avg' in perf[s][1]
```

**Implementation note:** `FieldAggregationSpec.extract()` tries
`record[self.field]`. When the value is `None`, the `float(value)` call
wraps it — but actually `float(None)` raises `TypeError`, which is
caught by the `except (KeyError, IndexError, TypeError)` in `extract()`,
returning `None`. The accumulator's `add()` is never called for that
record. The test validates this behavior works correctly.

Wait — reviewing `Tub.write_record()` at line 46-47:
```python
if value is None or key not in self.input_types:
    continue
```
Records with `None` values skip that field entirely in the catalog.
When the record is read back, the field won't exist in the dict at all,
which means `extract()` will get a `KeyError`. This is fine — the test
still verifies graceful handling.

**Revision:** Since `write_record` skips None values, we can't actually
write None to the tub. Instead, we test with records that simply OMIT
the field (don't include it in the dict). This produces the same
behavior — `extract()` gets KeyError → returns None.

```
Revised setup:
    for i in range(10):
        record = {
            'car/lap': lap,
            'car/distance': float(lap * 50 + i * 5),
            'car/gyro': [0.0, 0.0, 0.5],
            '_timestamp_ms': start_time_ms + lap * 10000 + i * 1000,
        }
        # Only include speed for some records
        if i % 3 != 0:
            record['car/speed'] = 10.0
        tub.write_record(record)
```

**Actually**, this also won't work: the field is defined in the schema
but the value is omitted from some records. `write_record` will simply
not write the key for that record. When reading it back, the record
dict won't have `car/speed` → `extract()` gets KeyError → returns None.
This is the correct test.

**Further revision:** The `write_record` method at line 46 checks
`key not in self.input_types` — it skips keys NOT in the schema. But
if a schema key is missing from the record dict, it simply won't appear
in `contents`. So omitting `car/speed` from some records is valid.

#### `test_all_segments_same_performance`

```
Setup:
    # 3 laps, 2 segments, all identical profiles
    profile = DrivingProfile(speed=10.0, gyro_z_base=0.5)
    profiles = [[profile, profile]] * 3

    field_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(
            field='car/gyro', index=2, output_key='gyro_z_agg',
            transform=abs, aggregation='avg'),
    ]

Assert:
    perf = _rank_laps(tub, field_aggs)
    s = _get_session_id(tub)
    # All laps ranked — no crash, no division by zero
    assert len(perf[s]) == 3
    # Rankings exist and are valid
    for lap_idx in range(3):
        assert 0 < perf[s][lap_idx]['time'] <= 1.0
        assert 0 < perf[s][lap_idx]['gyro_z_agg'] <= 1.0
    # Rankings are spread evenly: {0.33, 0.67, 1.0}
    time_ranks = sorted(perf[s][i]['time'] for i in range(3))
    assert len(set(time_ranks)) == 3  # All different despite same data
```

**Why all different:** `SortingStrategy.rank_laps()` uses `sorted()`
which is stable. Even with identical values, laps get distinct rank
positions (0.33, 0.67, 1.0) based on their original index order. The
key assertion is that no crash or degenerate output occurs.

---

## 4. Implementation Order

1. **`tub_test_data_generator.py`** — implement in this order:
   a. Constants (`TINY_IMAGE`, `TUB_INPUTS`, etc.)
   b. `DrivingProfile` dataclass
   c. Pre-defined profiles
   d. `generate_sensor_record()`
   e. `create_multilap_tub()`
   f. `create_tub_with_varied_segment_times()`
   g. `create_multilap_tub_via_writer()`

2. **`test_segment_ranking_comprehensive.py`** — implement in this order:
   a. `SegmentRankingTestBase` base class
   b. `TestFieldAggregationMethods` (8 tests)
   c. `TestMultiFieldRankingPriority` (5 tests)
   d. `TestDrivingBehaviorDiscrimination` (4 tests)
   e. `TestTransformFunctions` (4 tests)
   f. `TestTubWriterIntegration` (3 tests)
   g. `TestOnTheFlySegmentComputation` (2 tests)
   h. `TestRankingEdgeCases` (4 tests)

3. **Run tests** after each class is implemented. Fix any failures
   before proceeding to next class.

## 5. Risk Register

| Risk | Impact | Mitigation |
|------|--------|------------|
| TubWriter uses `time.time()` so timestamps are non-deterministic | Exact time rankings differ between runs | Compare ranking ORDER, not values |
| `write_record` skips None values | Can't test "None in record" | Test with omitted fields instead |
| Deterministic noise `sin(i*7.3)` may produce unexpected patterns | Aggregation values don't match expectations | Verify with a small sample first |
| `create_tub_with_varied_segment_times` needs per-profile support for each lap/segment | More complex generator API | Accept profiles list-of-lists alongside times |
| FieldAccumulator.median uses `sorted_vals[len//2]` (upper median for even) | Different from Python `statistics.median` | Document this behavior, test against actual implementation |

## 6. Lines of Code Estimate

| File | Estimated Lines |
|------|----------------|
| `tub_test_data_generator.py` | ~250 |
| `test_segment_ranking_comprehensive.py` | ~650 |
| **Total** | ~900 |
