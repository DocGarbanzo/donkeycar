# Tub Statistics Test Suite Documentation

## Overview

This document describes the comprehensive test suite for the `TubStatistics` class in `donkeycar/parts/tub_statistics.py`. The tests validate lap time generation, performance calculation, and gyro aggregation using simulated data representing a car driving on an oval track.

## Test File Location

`donkeycar/tests/test_tub_statistics.py`

## Test Structure

### Test Class: `TestTubStatistics`

The main test class that contains all test cases for the `TubStatistics` functionality.

#### Setup and Teardown

- `setUp()`: Creates a temporary directory for each test
- `tearDown()`: Removes the temporary directory after each test

### Helper Methods

#### `_create_oval_track_data()`

Creates simulated data for a car driving on an oval track.

**Parameters:**
- `num_laps` (int): Number of complete laps to simulate (default: 3)
- `records_per_lap` (int): Number of records per lap (default: 100)
- `lap_time_ms` (int): Time in milliseconds to complete one lap (default: 10000)
- `track_length` (float): Track length in arbitrary distance units (default: 50.0)
- `session_id` (str): Optional session ID

**Returns:**
- List of record dictionaries with:
  - `car/lap`: Current lap number
  - `car/distance`: Cumulative distance traveled
  - `car/gyro`: 3D gyro vector [x, y, z] simulating turns
  - `_timestamp_ms`: Timestamp in milliseconds
  - `_session_id`: Session identifier

**Simulation Details:**
- Gyro Z values simulate an oval track with two turns per lap
- Higher gyro values in turns (using sin wave)
- Lower gyro values on straights
- Linear timestamp progression
- Distance accumulates realistically

#### `_create_tub_with_data()`

Creates a Tub instance and writes records to it.

**Parameters:**
- `records` (list): List of record dictionaries to write

**Returns:**
- Tub instance with data written

## Test Cases

### 1. `test_generate_laptimes_single_session`

**Purpose:** Validates lap time generation for a single session.

**Validates:**
- Lap times are correctly calculated from timestamps
- Lap distances are correctly calculated
- Metadata is properly updated with laptimer information
- Each lap takes approximately 10 seconds (expected value)
- Each lap covers approximately 50 distance units (expected value)

**Test Data:** 3 laps, 100 records per lap

### 2. `test_generate_laptimes_multiple_sessions`

**Purpose:** Validates lap time generation for multiple sessions.

**Validates:**
- Lap times are tracked separately for each session
- Session boundaries are handled correctly
- Each session has its own laptimer metadata

**Test Data:** 
- Session 1: 2 laps (records 0-20)
- Session 2: 2 laps (records 21-50)

**Note:** Uses explicit `_index` and `_session_id` to simulate multiple sessions in a single tub.

### 3. `test_generate_laptimes_overwrite`

**Purpose:** Tests the overwrite functionality for lap times.

**Validates:**
- Existing lap times can be overwritten when `overwrite=True`
- Original lap times are preserved when `overwrite=False`

**Test Data:** 2 laps

### 4. `test_calculate_aggregated_gyro`

**Purpose:** Tests aggregation of gyro Z values per lap.

**Validates:**
- Gyro Z values are correctly aggregated per lap
- Average gyro values are calculated correctly
- Metadata is updated with `gyro_z_agg` values
- Gyro values are positive (using abs() function)
- Average values are reasonable for simulated data

**Test Data:** 2 laps

### 5. `test_calculate_lap_performance`

**Purpose:** Tests calculation of lap performance rankings.

**Validates:**
- Laps are ranked by time, distance, and gyro_z_agg
- Ranking values are between 0 and 1
- Each lap has all three ranking metrics

**Test Data:** 5 laps with different completion times

### 6. `test_calculate_lap_performance_with_bins`

**Purpose:** Tests lap performance calculation with binning.

**Validates:**
- Laps can be grouped into a specified number of bins
- Bin assignments are correct (e.g., 5 bins = 0.2, 0.4, 0.6, 0.8, 1.0)

**Test Data:** 10 laps

### 7. `test_calculate_lap_performance_skip_lap_0`

**Purpose:** Tests skipping lap 0 in performance calculation.

**Validates:**
- Lap 0 can be excluded from rankings (`use_lap_0=False`)
- Rankings only include laps 1 and 2
- Lap 0 is not present in the results

**Test Data:** 3 laps

### 8. `test_calculate_lap_performance_compressed`

**Purpose:** Tests compressed lap performance calculation.

**Validates:**
- Multiple sessions can be compressed into one ranking
- All laps across sessions are ranked together

**Test Data:** 2 sessions with 2 laps each

**Note:** Manually populates sessions metadata for explicit session IDs.

### 9. `test_all_lap_times`

**Purpose:** Tests retrieval of all lap times.

**Validates:**
- `all_lap_times()` returns a dictionary of session → lap → time
- All sessions and laps are included
- Times are approximately 10 seconds as expected

**Test Data:** 3 laps

### 10. `test_gyro_z_index_configuration`

**Purpose:** Tests that the `gyro_z_index` parameter is respected.

**Validates:**
- `gyro_z_index=1` uses the second element of gyro vector (sim)
- `gyro_z_index=2` uses the third element (real car)

**Test Data:** 2 laps with gyro vectors [0.1, 0.5, 0.9]

**Verification:**
- With `gyro_z_index=1`: gyro_z_agg ≈ 0.5
- With `gyro_z_index=2`: gyro_z_agg ≈ 0.9

### 11. `test_empty_tub`

**Purpose:** Tests handling of empty tub.

**Validates:**
- Empty tubs don't cause errors (or fail gracefully with expected assertion)
- No lap times are generated for empty tubs

### 12. `test_single_lap`

**Purpose:** Tests handling of a single lap.

**Validates:**
- A single complete lap generates valid lap time
- Metadata is correctly populated
- Lap number is 0

**Test Data:** 1 lap

## Key Patterns and Conventions

### Session ID Handling

The Tub implementation automatically assigns session IDs. Tests account for this by:
1. Using `tub.manifest.session_id[1]` to get the actual session ID assigned by the tub
2. For multiple sessions, writing with explicit `_index` and `_session_id`
3. Closing and reopening tubs to ensure session metadata is updated

### Oval Track Simulation

The `_create_oval_track_data()` method simulates realistic car behavior:
- **Gyro values**: Uses `sin(2 * angle)` to create two turns per lap
- **Turn intensity**: 0.1 to 1.0 range (0.1 baseline + 0.9 * turn_intensity)
- **Timestamps**: Linear progression with configurable lap time
- **Distance**: Cumulative, increases linearly

### Test Data Characteristics

- **Default lap time**: 10 seconds (10000 ms)
- **Default track length**: 50.0 distance units
- **Default records per lap**: 100
- **Gyro vector format**: [x, y, z] where z is at index 1 for sim, 2 for real car

## Running the Tests

```bash
# Run all tub statistics tests
python -m pytest donkeycar/tests/test_tub_statistics.py -v

# Run a specific test
python -m pytest donkeycar/tests/test_tub_statistics.py::TestTubStatistics::test_generate_laptimes_single_session -v

# Run with output
python -m pytest donkeycar/tests/test_tub_statistics.py -v -s
```

## Coverage

The test suite covers:
- ✅ Lap time generation from records
- ✅ Multiple session handling
- ✅ Lap time overwriting
- ✅ Gyro aggregation
- ✅ Performance ranking calculation
- ✅ Binning functionality
- ✅ Lap 0 filtering
- ✅ Session compression
- ✅ Gyro Z index configuration
- ✅ Edge cases (empty tub, single lap)

## Future Enhancements

Potential areas for additional testing:
1. Performance with very large datasets (1000+ laps)
2. Error handling for corrupted data
3. Validation of lap number sequences (detecting gaps or duplicates)
4. Testing with real tub data from actual car runs
5. Testing `_calculate_laptimer_index()` method
6. Integration tests with the full donkeycar pipeline

## Dependencies

The tests require:
- `unittest` (Python standard library)
- `tempfile` (Python standard library)
- `numpy` (for array operations, though not heavily used in current tests)
- `donkeycar.parts.tub_v2` (Tub, TubWriter)
- `donkeycar.parts.tub_statistics` (TubStatistics)

## Notes

- Tests use temporary directories that are cleaned up after each test
- Tests are independent and can be run in any order
- The test suite validates functionality without requiring actual hardware
- Simulated data provides consistent, reproducible test results
