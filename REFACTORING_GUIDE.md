# Donkeycar Pipeline Refactoring - Issue #11

## Overview

This refactoring addresses issue #11 by improving the modularity and extensibility of the donkeycar data pipeline, specifically focusing on:

1. **Removing nested functions** - Extracted nested `rank_laps()` function from `TubStatistics.calculate_lap_performance()`
2. **Modular transformations** - Replaced hardcoded `abs()` with configurable transformation system
3. **Flexible sorting** - Replaced hardcoded sorting criteria with customizable `SortingStrategy`
4. **Configurable ranking keys** - Enabled custom lap performance metrics beyond time/distance/gyro
5. **Improved test coverage** - Added comprehensive tests with simulated oval track data

## Key Changes

### 1. New Transformation System

**File:** `donkeycar/pipeline/transformations.py`

The `Transformation` class provides a composable way to apply transformations to data:

```python
from donkeycar.pipeline.transformations import (
    abs_transform,
    clamp_transform,
    scale_transform,
    identity_transform
)

# Use absolute value transformation
abs_t = abs_transform()
result = abs_t(-5.0)  # Returns 5.0

# Use clamp transformation
clamp_t = clamp_transform(-1.0, 1.0)
result = clamp_t(2.0)  # Returns 1.0 (clamped)

# Compose transformations
composed = abs_t.compose(scale_transform(2.0))
result = composed(-3.0)  # Returns 6.0 (abs then scale)
```

### 2. Modular Sorting Strategy

**File:** `donkeycar/pipeline/transformations.py`

The `SortingStrategy` class allows custom sorting criteria for lap ranking:

```python
from donkeycar.pipeline.transformations import (
    SortingStrategy,
    SortingCriterion,
    default_lap_sorting_strategy,
    custom_lap_sorting_strategy
)

# Use default sorting (backward compatible)
strategy = default_lap_sorting_strategy()  # time, distance, gyro_z_agg

# Create custom sorting strategy
custom_strategy = SortingStrategy([
    SortingCriterion('time'),
    SortingCriterion('distance'),
    SortingCriterion('custom_metric', 
                    extractor=lambda d: d.get('custom_field'),
                    transformation=abs_transform(),
                    reverse=False)
])

# Use strategy to rank laps
laps = [
    {'lap': 0, 'time': 10.0, 'distance': 50.0, 'custom_field': -5.0},
    {'lap': 1, 'time': 9.0, 'distance': 48.0, 'custom_field': 3.0},
]
rankings = custom_strategy.rank_laps(laps, num_buckets=5)
```

### 3. Refactored TubStatistics

**File:** `donkeycar/parts/tub_statistics.py`

`TubStatistics` now accepts optional sorting strategy and transformation:

```python
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics
from donkeycar.pipeline.transformations import (
    SortingStrategy,
    SortingCriterion,
    clamp_transform
)

tub = Tub('/path/to/tub', read_only=True)

# Use default behavior (backward compatible)
stats = TubStatistics(tub, gyro_z_index=1)

# Use custom sorting and transformation
custom_sorting = SortingStrategy([
    SortingCriterion('time'),
    SortingCriterion('distance'),
])

stats = TubStatistics(
    tub,
    gyro_z_index=1,
    sorting_strategy=custom_sorting,
    gyro_transformation=clamp_transform(0.0, 1.0)
)

stats.generate_laptimes_from_records()
performance = stats.calculate_lap_performance(use_lap_0=True)
```

### 4. Configurable TubDataset

**File:** `donkeycar/pipeline/types.py`

`TubDataset` now supports custom ranking keys:

```python
from donkeycar.config import Config
from donkeycar.pipeline.types import TubDataset

config = Config()
config.GYRO_Z_INDEX = 1
config.USE_LAP_0 = True

# Use default ranking keys (backward compatible)
dataset = TubDataset(config, ['/path/to/tub'], add_lap_pct=True)

# Use custom ranking keys
custom_keys = ['time', 'distance']  # Exclude gyro_z_agg
dataset = TubDataset(
    config,
    ['/path/to/tub'],
    add_lap_pct=True,
    ranking_keys=custom_keys
)

records = dataset.get_records()
# Each record's lap_pct will contain only [time_ranking, distance_ranking]
```

### 5. Enhanced TubRecord.extend()

**File:** `donkeycar/pipeline/types.py`

`TubRecord.extend()` now accepts optional ranking keys:

```python
from donkeycar.pipeline.types import TubRecord
from donkeycar.config import Config

record = TubRecord(config, tub.base_path, underlying_dict)

# Use default keys (backward compatible)
record.extend(session_lap_rank)  # Uses time, distance, gyro_z_agg

# Use custom keys
record.extend(session_lap_rank, ranking_keys=['time', 'custom_metric'])
```

## Backward Compatibility

All changes maintain **100% backward compatibility**:

- Default sorting strategy uses original criteria: `time`, `distance`, `gyro_z_agg`
- Default transformation is `abs()` for gyro values
- Default ranking keys are `['time', 'distance', 'gyro_z_agg']`
- All existing tests pass without modification

## Testing

### New Test Files

- `donkeycar/tests/test_pipeline.py` - Extended with comprehensive tests:
  - `TestTubDatasetSortingAndTransformation` - Tests for current behavior
  - `TestTransformations` - Tests for new Transformation classes
  - `TestSortingStrategy` - Tests for new SortingStrategy classes
  - `TestModularTubStatistics` - Tests for refactored TubStatistics
  - `TestModularTubDataset` - Tests for refactored TubDataset

### Simulated Oval Track Data

Tests use realistic simulated data of a car driving on an oval track:

```python
def _create_oval_track_data(num_laps=3, records_per_lap=100):
    # Simulates:
    # - Lap numbers incrementing
    # - Distance accumulating
    # - Gyro Z values varying through turns
    # - Timestamps progressing
    # - Steering and throttle inputs
    pass
```

### Running Tests

```bash
# Run all pipeline tests
python -m unittest donkeycar.tests.test_pipeline

# Run specific test class
python -m unittest donkeycar.tests.test_pipeline.TestTransformations

# Run specific test
python -m unittest donkeycar.tests.test_pipeline.TestTransformations.test_abs_transformation
```

## Code Structure Improvements

### Before (Nested and Hardcoded)

```python
# Old TubStatistics.calculate_lap_performance()
def calculate_lap_performance(self, ...):
    def rank_laps(laps_filtered, num_buckets, session_lap_rank):  # Nested!
        for sort_by in ('time', 'distance', 'gyro_z_agg'):  # Hardcoded!
            laps_sorted = sorted(laps_filtered, key=itemgetter(sort_by))
            # ... ranking logic ...
    # ... use rank_laps() ...

# Old _calculate_aggregated_gyro()
val = abs(record['car/gyro'][self.gyro_z_index])  # Hardcoded abs()!
```

### After (Modular and Configurable)

```python
# New TubStatistics with dependency injection
def __init__(self, tub, gyro_z_index=1,
             sorting_strategy=None,
             gyro_transformation=None):
    self.sorting_strategy = sorting_strategy or default_lap_sorting_strategy()
    self.gyro_transformation = gyro_transformation or abs_transform()

def calculate_lap_performance(self, ...):
    # No nested function! Uses injected strategy
    rankings = self.sorting_strategy.rank_laps(laps_data, num_bins)

def _calculate_aggregated_gyro(self):
    # Uses configurable transformation
    raw_val = record['car/gyro'][self.gyro_z_index]
    val = self.gyro_transformation(raw_val)
```

## Benefits

1. **Modularity** - Transformations and sorting are separate, reusable classes
2. **Testability** - Each component can be tested independently
3. **Extensibility** - Easy to add new transformations or sorting criteria
4. **Readability** - No more nested functions, clearer separation of concerns
5. **Flexibility** - Users can customize behavior without modifying core code
6. **Maintainability** - Changes to sorting/transformation logic are localized

## Usage Examples

### Example 1: Custom Gyro Processing

Instead of hardcoded `abs()`, use a clamp transformation:

```python
from donkeycar.pipeline.transformations import clamp_transform

# Clamp gyro values to reasonable range
gyro_clamp = clamp_transform(-2.0, 2.0)
stats = TubStatistics(tub, gyro_z_index=1, gyro_transformation=gyro_clamp)
```

### Example 2: Custom Lap Ranking

Rank laps by custom metrics:

```python
from donkeycar.pipeline.transformations import (
    SortingStrategy,
    SortingCriterion
)

# Rank by speed and smoothness
custom_strategy = SortingStrategy([
    SortingCriterion('avg_speed', reverse=True),  # Higher is better
    SortingCriterion('steering_variance'),  # Lower is better
])

stats = TubStatistics(tub, sorting_strategy=custom_strategy)
```

### Example 3: Behavioral Cloning with Transformations

Apply transformations to behavioral variables:

```python
from donkeycar.pipeline.sequence import PipelineGenerator
from donkeycar.pipeline.transformations import clamp_transform

# Clamp steering to safe range
clamp_steering = clamp_transform(-1.0, 1.0)

pipeline = PipelineGenerator(
    records,
    x_transform=lambda r: clamp_steering(r.underlying['user/angle']),
    y_transform=lambda r: r.underlying['user/throttle']
)
```

## Migration Guide

No migration needed! All existing code continues to work without changes.

To opt into new features:

1. **Custom transformations:** Pass to `TubStatistics.__init__()`
2. **Custom sorting:** Pass `sorting_strategy` to `TubStatistics.__init__()`
3. **Custom ranking keys:** Pass `ranking_keys` to `TubDataset.__init__()`

## Future Enhancements

Possible future improvements building on this refactoring:

1. **More transformations:** sigmoid, softmax, polynomial, etc.
2. **Chained filters:** Compose multiple filtering strategies
3. **Dynamic ranking:** Change ranking criteria per session
4. **Visualization:** Plot transformation effects on data distribution
5. **Configuration:** Define transformations/sorting in config files

## Summary

This refactoring successfully addresses issue #11 by:

✅ Removing nested functions (rank_laps extracted to SortingStrategy)
✅ Making code more modular (Transformation, SortingStrategy classes)
✅ Adding comprehensive tests with simulated oval track data
✅ Enabling custom sorting and ordering by any tub parameter
✅ Supporting transformations (abs, clamp, etc.) for behavioral variables
✅ Maintaining 100% backward compatibility
✅ Improving code readability and maintainability

The codebase is now more flexible, extensible, and easier to test and maintain.
