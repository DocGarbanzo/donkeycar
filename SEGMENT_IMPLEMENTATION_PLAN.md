# Segment-Based Performance Implementation Plan

## Overview
Add segment-based behavioral parameters to training data, enabling the model to learn from the best-driven segments across all laps, creating a synthetic "perfect lap" that outperforms any single recorded lap.

## Key Concept
- **Current**: `lap_pct` = behavioral parameters [time, gyro_z, distance] measured per lap
- **New**: Same `lap_pct` field, but populated from segment-level measurements
- **Toggle**: `USE_SEGMENT_PCT` config flag switches between lap-based and segment-based
- **No model changes**: Existing `KerasSquarePlusMemoryLap` works unchanged

---

## UPDATED ARCHITECTURE (Jan 2026)

**Key Changes from Original Plan:**

1. **Flexible Field Selection**: No hardcoded fields (time, gyro_z, distance). User selects ANY tub field dynamically.

2. **Aggregation Methods**: Support multiple methods per field:
   - `delta`: last - first (for time, distance)
   - `mean_abs`: mean of absolute values
   - `sum_abs`: sum of absolute values
   - `max`: maximum value
   - `magnitude`: for vectors, Euclidean norm ||v||

3. **Vector Field Support**:
   - Detect 2D/3D vector fields automatically
   - Allow component selection (X, Y, Z) or magnitude
   - Position: 2D (x, y)
   - IMU data: 3D (x, y, z)

4. **Web UI Primary**:
   - Web interface is now the main tool
   - Matplotlib UI (`donkey imupath`) remains legacy (no segment stats)
   - Auto-open browser on startup (like Jupyter notebook)

5. **On-Demand Computation**:
   - Rankings computed on-demand based on user selection
   - Cached per (field, method, dimension) combination
   - No pre-computation of all possible aggregations

---

## Current Implementation Architecture

The segment-based performance system is **fully functional** with the following design:

### Configuration-Driven Approach
- Fields specified in `FIELD_AGGREGATIONS` list in config.py
- Each field spec includes: field name, index (for vectors), output_key, transform, aggregation
- Supports multiple aggregation types: avg, sum, min, max, median

### Pre-Computation Strategy
- All aggregation variants computed at startup
- Example: `gyro_z_agg`, `gyro_z_agg_sum`, `gyro_z_agg_min`, etc.
- Cached in memory for fast UI response

### Web UI Features
- Stats field selector dropdown (populated from computed rankings)
- Segment ranking display (0-100%)
- Auto-browser opening on server start
- Real-time updates when changing fields

### Training Integration
- `donkey segment --tub <path>` computes segment assignments
- `SEGMENT_PCT_MODE=True` enables segment-based training
- Segment rankings stored in tub manifest metadata
- Training pipeline uses segment rankings via PctMode.SEGMENT

### Aspirational Features (Not Implemented)
The "Updated Architecture (Jan 2026)" section describes advanced features that are **NOT currently implemented**:
- Dynamic field introspection from tub records
- UI-based aggregation method selection
- Vector dimension selector (X/Y/Z components)
- On-demand computation with per-query caching

These remain as **future enhancements** if needed for advanced use cases.

---

## Implementation Checklist

### Phase 1: Data Loading Infrastructure ✅ COMPLETE

- [x] **1.1 Create TubPathDataSource**
  - File: `donkeycar/course_analysis/data_loader.py`
  - COMPLETE: `TubPathDataSource(PathDataSource)` class exists
  - COMPLETE: `load(tub_path, session_id)` method implemented
  - COMPLETE: Extracts `car/pos`, `car/euler`, `car/speed`, `_timestamp_ms`
  - COMPLETE: Returns `PathData` object with all required arrays
  - COMPLETE: Handles missing fields gracefully

- [x] **1.2 Test TubPathDataSource**
  - File: `tests/test_tub_path_data_source.py`
  - COMPLETE: Comprehensive tests exist

---

### Phase 2: Segmentation Computation ✅ COMPLETE

- [x] **2.1 Add compute_segment_assignments() to TubStatistics**
  - File: `donkeycar/parts/tub_statistics.py` (lines 462-586)
  - COMPLETE: Method exists with full functionality
  - COMPLETE: Loads PathData, detects laps, builds mean course
  - COMPLETE: Segments course and assigns to all records
  - COMPLETE: Stores metadata in manifest (NOT in catalog records)
  - COMPLETE: Metadata includes segments, boundaries, mean course

- [x] **2.2 Add flexible field aggregation to TubStatistics**
  - File: `donkeycar/parts/tub_statistics.py` (lines 588-653)
  - COMPLETE: Method `calculate_segment_performance()` exists
  - COMPLETE: Uses `FieldAggregationSpec` for flexible field configuration
  - NOTE: Current implementation uses config-driven approach, not dynamic detection
  - **Field Aggregation Specification**:
    ```python
    # Each aggregation specifies:
    # - field: The tub field to aggregate (e.g., 'imu/gyr', 'car/speed', '_timestamp_ms')
    # - method: Aggregation method ('mean_abs', 'sum_abs', 'max', 'delta', 'magnitude')
    # - output_key: Name for the ranking output
    # - dimension: Optional - for vector fields, which component to use (0=x, 1=y, 2=z, None=magnitude)

    field_aggregations = [
        FieldAggregation(
            field='_timestamp_ms',
            method='delta',  # last - first = time spent in segment
            output_key='time',
            scalar=True
        ),
        FieldAggregation(
            field='imu/gyr',
            method='sum_abs',  # sum of abs values across all components
            output_key='gyro_z_agg',
            dimension=2,  # Z-axis only
            scalar=False
        ),
        FieldAggregation(
            field='car/pos',
            method='delta',  # distance traveled in segment
            output_key='distance',
            dimension=None,  # magnitude of position change
            scalar=False
        ),
    ]
    ```
  - **Aggregation Methods** (CURRENT):
    - Supports: `avg`, `sum`, `min`, `max`, `median`
    - Config-driven via `FIELD_AGGREGATIONS` in cfg_complete.py
    - Transform functions (e.g., abs) for preprocessing
  - **Return Structure**:
    ```python
    session_segment_rank[session_id][lap_num][segment_id] = {
        'time': 0.4,
        'gyro_z_agg': 0.2,
        'distance': 1.0,
    }
    ```
  - **Critical**: Structure is session -> lap -> segment -> {metric: rank}

- [x] **2.3 Test segment performance calculation**
  - File: `tests/test_segment_performance.py`
  - COMPLETE: Comprehensive test suite exists (316 lines)

---

### Phase 3: Training Pipeline Integration ✅ COMPLETE

- [x] **3.1 Modify TubDataset to support segment_pct**
  - File: `donkeycar/pipeline/types.py` (lines 249-279)
  - COMPLETE: `pct_mode` parameter exists
  - COMPLETE: Routes to segment or lap performance based on mode
  - COMPLETE: `PctMode` enum defined (NONE, LAP, SEGMENT)

- [x] **3.2 Modify TubDataset.extend() to populate lap_pct from segment_pct**
  - File: `donkeycar/pipeline/types.py` (lines 182-238)
  - COMPLETE: `TubRecord.extend()` handles both LAP and SEGMENT modes
  - COMPLETE: Extracts segment rankings from session_rank structure

- [x] **3.3 Update training.py to use pct_mode**
  - File: `donkeycar/pipeline/training.py` (lines 140-150)
  - COMPLETE: Determines pct_mode from SEGMENT_PCT_MODE config
  - COMPLETE: Passes pct_mode to TubDataset constructor

- [x] **3.4 Test training pipeline integration**
  - File: `tests/test_segment_training_integration.py`
  - COMPLETE: Integration tests exist (259 lines)

---

### Phase 4: Configuration ✅ COMPLETE

- [x] **4.1 Add configuration parameters and PctMode enum**
  - File: `donkeycar/pipeline/types.py` (lines 25-35)
  - COMPLETE: PctMode enum exists (NONE, LAP, SEGMENT)
  - File: `donkeycar/templates/cfg_complete.py` (lines 765-838)
  - COMPLETE: All configuration parameters exist
  - COMPLETE: FIELD_AGGREGATIONS and LAP_SORTING_CRITERIA configured

---

### Phase 5: Management Command ✅ COMPLETE

- [x] **5.1 Create donkey segment command**
  - File: `donkeycar/management/segment.py`
  - COMPLETE: SegmentCommand class exists
  - COMPLETE: Accepts all required arguments
  - COMPLETE: Calls compute_segment_assignments()
  - COMPLETE: Prints summary of results

- [x] **5.2 Register command in base.py**
  - File: `donkeycar/management/base.py`
  - COMPLETE: Command registered as 'segment': SegmentCommand

- [x] **5.3 Test command execution**
  - File: `tests/test_segment_command.py`
  - COMPLETE: Tests exist (230 lines)

---

### Phase 6: Web UI Integration ⚠️ MOSTLY COMPLETE

**NOTE**: Core web UI features are working. Advanced "Updated Architecture" features are deferred.

- [x] **6.1 Add auto-browser opening on server start**
  - File: `donkeycar/management/imupath.py` (lines 226-236)
  - COMPLETE: Uses `webbrowser.open()` after server starts
  - COMPLETE: Opens URL: `http://localhost:{port}/imupath`

- [ ] **6.2 Dynamic field detection (DEFERRED)**
  - Current: Uses `FIELD_AGGREGATIONS` from config.py
  - Aspirational: Auto-detect all tub fields from records
  - Reason: Config-driven approach is simpler and sufficient

- [ ] **6.3 Aggregation method selector (DEFERRED)**
  - Current: Stats field selector only (shows pre-computed variants)
  - Aspirational: Separate dropdowns for method and dimension
  - Reason: Pre-computing all variants is fast enough

- [ ] **6.3 Dimension selector (DEFERRED)**
  - Current: Use index in FIELD_AGGREGATIONS config
  - Aspirational: UI dropdown for X/Y/Z component selection
  - Reason: Config approach works for current needs

- [x] **6.4 Display segment rank in web UI**
  - File: `donkeycar/parts/web_controller/templates/imupath.html`
  - COMPLETE: "Seg Rank" display in Current Position panel
  - COMPLETE: Updates dynamically when position changes
  - COMPLETE: Shows percentile (0-100%)

- [x] **6.5 Pre-compute multiple aggregations**
  - File: `donkeycar/web/imupath_data.py` (lines 190-209)
  - COMPLETE: `_expand_field_aggregations()` creates all variants
  - COMPLETE: Computes avg, sum, min, max, median for each field
  - COMPLETE: Cached in memory for fast access

- [ ] **6.5 On-demand computation (DEFERRED)**
  - Current: All aggregations computed at startup
  - Aspirational: Compute only when user selects field+method
  - Reason: Pre-computation is fast and simplifies caching

- [x] **6.6 Legacy UI (matplotlib) - NO CHANGES**
  - COMPLETE: Matplotlib UI remains as legacy tool

---

### Phase 7: Documentation ⚠️ COULD BE ENHANCED

- [x] **7.1 CLAUDE.md documentation exists**
  - File: `donkeycar/CLAUDE.md`
  - COMPLETE: Segment-based performance section exists
  - NOTE: Could be enhanced with more details on current architecture

---

### Phase 8: Testing ✅ COMPLETE

- [x] **8.1 Integration test: End-to-end workflow**
  - File: `tests/test_segment_training_integration.py`
  - COMPLETE: Integration tests exist (259 lines)

- [x] **8.2 Regression test: Lap-based training still works**
  - File: `tests/test_lap_pct_regression.py`
  - COMPLETE: Tests exist for backward compatibility

---

## Validation Checklist ✅ ALL COMPLETE

After implementation, verify:

- [x] **Segmentation works**: `donkey segment --tub <path>` completes successfully
- [x] **Metadata is stored**: Tub contains segmentation metadata (num_segments, params)
- [x] **Training loads segment_pct**: TubDataset populates lap_pct from segment rankings (session -> lap -> segment)
- [x] **Model trains**: Existing KerasSquarePlusMemoryLap trains with segment_pct
- [x] **UI shows segments**: Web UI displays segment rankings
- [x] **Backward compatible**: pct_mode=PctMode.LAP uses original lap-based behavior
- [x] **Tests pass**: All new and existing tests pass

---

## File Summary

**New files:**
- `tests/test_tub_path_data_source.py`
- `tests/test_segment_performance.py`
- `tests/test_course_segmentation_integration.py`
- `tests/test_segment_training_integration.py`
- `donkeycar/management/segment.py`

**Modified files:**
- `donkeycar/course_analysis/data_loader.py` (add TubPathDataSource)
- `donkeycar/parts/tub_statistics.py` (add compute_segment_assignments, calculate_segment_performance)
- `donkeycar/pipeline/types.py` (modify TubDataset.__init__, extend(), TubRecord.__init__)
- `donkeycar/pipeline/training.py` (modify train() to pass use_segment_pct)
- `donkeycar/templates/cfg_complete.py` (add config parameters)
- `donkeycar/management/base.py` (register segment command)
- `donkeycar/CLAUDE.md` (documentation)

---

## Expected Behavior

**Example: 3 laps, 4 segments**

Lap 1: Segments [Fast, Slow, Medium, Fast]
Lap 2: Segments [Medium, Fast, Fast, Slow]
Lap 3: Segments [Slow, Medium, Slow, Medium]

**Data structure: session_rank[session_id][lap][segment] = [time_pct, gyro_z_pct, distance_pct]**

**Segment 0 ranking (across laps 1,2,3):**
- session_rank[session_id][1][0] = [0.33, ...]  # Lap 1, Fastest
- session_rank[session_id][2][0] = [0.67, ...]  # Lap 2, Medium
- session_rank[session_id][3][0] = [1.0, ...]   # Lap 3, Slowest

**Segment 1 ranking (across laps 1,2,3):**
- session_rank[session_id][1][1] = [1.0, ...]   # Lap 1, Slowest
- session_rank[session_id][2][1] = [0.33, ...]  # Lap 2, Fastest
- session_rank[session_id][3][1] = [0.67, ...]  # Lap 3, Medium

**Result:** Training prioritizes:
- Segment 0 from Lap 1 (lap_pct = [0.33, ...])
- Segment 1 from Lap 2 (lap_pct = [0.33, ...])
- Segment 2 from Lap 2 (lap_pct = [0.33, ...])
- Segment 3 from Lap 1 (lap_pct = [0.33, ...])

This creates a "synthetic best lap" combining the best-driven instances of each segment!

---

## Progress Tracking

**IMPLEMENTATION STATUS: COMPLETE FOR CORE FEATURES** ✅

The segment-based performance system is fully functional with:
- ✅ Data loading and segmentation
- ✅ Performance ranking computation
- ✅ Training pipeline integration
- ✅ Management commands
- ✅ Web UI visualization (config-driven)
- ✅ Comprehensive testing

**Aspirational features deferred**: Dynamic field detection, UI-based aggregation/dimension selection, on-demand computation. These can be implemented later if needed.

**To use the system:**
1. `donkey segment --tub data/` - Compute segments and store in metadata
2. `donkey imupath --web data/` - Visualize segment performance in web UI
3. Set `SEGMENT_PCT_MODE=True` in config.py
4. `python manage.py train --tub data/` - Train with segment rankings
