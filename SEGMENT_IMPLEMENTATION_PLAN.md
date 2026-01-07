# Segment-Based Performance Implementation Plan

## Overview
Add segment-based behavioral parameters to training data, enabling the model to learn from the best-driven segments across all laps, creating a synthetic "perfect lap" that outperforms any single recorded lap.

## Key Concept
- **Current**: `lap_pct` = behavioral parameters [time, gyro_z, distance] measured per lap
- **New**: Same `lap_pct` field, but populated from segment-level measurements
- **Toggle**: `USE_SEGMENT_PCT` config flag switches between lap-based and segment-based
- **No model changes**: Existing `KerasSquarePlusMemoryLap` works unchanged

---

## Implementation Checklist

### Phase 1: Data Loading Infrastructure

- [ ] **1.1 Create TubPathDataSource**
  - File: `donkeycar/course_analysis/data_loader.py`
  - Add `TubPathDataSource(PathDataSource)` class
  - Implement `load(tub_path, session_id)` method
  - Extract `car/pos`, `car/euler`, `car/speed`, `_timestamp_ms` from tub records
  - Return `PathData` object with x, y, heading, velocity, timestamp arrays
  - Handle missing fields gracefully

- [ ] **1.2 Test TubPathDataSource**
  - File: `tests/test_tub_path_data_source.py`
  - Test loading single session
  - Test loading all sessions
  - Test field extraction (position, heading, velocity)
  - Test missing field handling
  - Test timestamp conversion (ms to seconds)

---

### Phase 2: Segmentation Computation

- [ ] **2.1 Add compute_segment_assignments() to TubStatistics**
  - File: `donkeycar/parts/tub_statistics.py`
  - Add method `compute_segment_assignments(lap_detector='ycrossing', segmentation_strategy='hybrid', **kwargs)`
  - For each session:
    - Load PathData via TubPathDataSource
    - Detect laps using LapDetector
    - Build mean course from ALL laps using MeanCourseBuilder
    - Segment course using CourseSegmenter
    - Assign segments to all records using SegmentAssigner
    - Write segment ID directly to each tub record: `record['car/segment'] = segment_id`
  - Store metadata in `tub.manifest.metadata[session_id]['segmentation']`:
    ```python
    {
        'num_segments': int,
        'mean_course_params': {...},
        'segmentation_params': {...}
    }
    ```
  - Call `tub.manifest.write_metadata()`

- [ ] **2.2 Add calculate_segment_performance() to TubStatistics**
  - File: `donkeycar/parts/tub_statistics.py`
  - Add method `calculate_segment_performance(ranking_keys=('time', 'distance', 'gyro_z_agg'), field_aggregations=None)`
  - Mirror structure of `calculate_lap_performance()` exactly
  - For each lap, for each segment in that lap:
    - Collect all instances of this segment across all laps
    - Aggregate metrics (time, distance, gyro_z_agg) per segment per lap
    - Rank the instances [0.1, 0.2, ..., 1.0] for each metric
  - Return structure:
    ```python
    session_segment_rank[session_id][lap_num][segment_id] = [0.4, 0.2, 1.0]
    # 3-element list: [time_pct, gyro_z_pct, distance_pct]
    ```
  - **Critical**: Structure is session -> lap -> segment (NOT session -> segment -> lap)

- [ ] **2.3 Test segment performance calculation**
  - File: `tests/test_segment_performance.py`
  - Test segment assignment to records
  - Test per-segment metric aggregation
  - Test segment ranking across laps
  - Test correct percentile calculation
  - Verify: Segment 1 in lap 2 gets different ranking than segment 1 in lap 3

---

### Phase 3: Training Pipeline Integration

- [ ] **3.1 Modify TubDataset to support segment_pct**
  - File: `donkeycar/pipeline/types.py`
  - Add parameter `pct_mode=PctMode.LAP` to `TubDataset.__init__()` (Enum: None, Lap, Segment)
  - In `__init__()`:
    ```python
    if pct_mode == PctMode.SEGMENT:
        # Calculate segment performance (reads car/segment from records)
        stats = TubStatistics(tub)
        self.session_rank = stats.calculate_segment_performance()
    elif pct_mode == PctMode.LAP:
        # Existing lap performance code
        stats = TubStatistics(tub)
        self.session_rank = stats.calculate_lap_performance()
    else:
        self.session_rank = None
    ```

- [ ] **3.2 Modify TubDataset.extend() to populate lap_pct from segment_pct**
  - File: `donkeycar/pipeline/types.py`
  - Modify `extend(record)` method:
    ```python
    def extend(self, record):
        session_id = record['_session_id']
        lap = record['car/lap']

        if self.pct_mode == PctMode.SEGMENT:
            # Get segment ID from record (already stored in car/segment)
            segment_id = record.get('car/segment')

            # Populate lap_pct from segment ranking: session -> lap -> segment
            if session_id in self.session_rank and segment_id is not None:
                record.underlying['lap_pct'] = self.session_rank[session_id][lap][segment_id]
        elif self.pct_mode == PctMode.LAP:
            # Existing lap-based code
            if session_id in self.session_rank:
                record.underlying['lap_pct'] = self.session_rank[session_id][lap]
    ```

- [ ] **3.3 Update training.py to use pct_mode**
  - File: `donkeycar/pipeline/training.py`
  - Modify `train()` function:
    ```python
    # Determine pct_mode from config
    pct_mode = PctMode.NONE
    if cfg.LAP_QUANTIFIER is not None:
        pct_mode = PctMode.LAP
    if hasattr(cfg, 'SEGMENT_PCT_MODE') and cfg.SEGMENT_PCT_MODE:
        pct_mode = PctMode.SEGMENT

    dataset = TubDataset(
        tub_paths,
        add_lap_pct=(pct_mode != PctMode.NONE),
        pct_mode=pct_mode
    )
    ```

- [ ] **3.4 Test training pipeline integration**
  - File: `tests/test_course_segmentation_integration.py`
  - Create test tub with 3 laps, multiple segments
  - Compute segmentation (writes car/segment to records)
  - Load TubDataset with `pct_mode=PctMode.SEGMENT`
  - Verify records have `lap_pct` populated from segment rankings
  - Verify different segments in same lap have different `lap_pct` values

---

### Phase 4: Configuration

- [ ] **4.1 Add configuration parameters and PctMode enum**
  - File: `donkeycar/pipeline/types.py`
  - Add enum definition:
    ```python
    from enum import Enum

    class PctMode(Enum):
        NONE = 0
        LAP = 1
        SEGMENT = 2
    ```
  - File: `donkeycar/templates/cfg_complete.py`
  - Add configuration section:
    ```python
    #SEGMENT PERFORMANCE
    SEGMENT_PCT_MODE = False  # True = segment-based, False = lap-based
    SEGMENT_STRATEGY = 'hybrid'  # Segmentation strategy
    SEGMENT_LAP_DETECTOR = 'ycrossing'  # Lap detection strategy
    SEGMENT_MIN_LENGTH = 1.0  # Minimum segment length in meters
    SEGMENT_CURVATURE_THRESHOLD = 0.1  # Curvature threshold
    ```

---

### Phase 5: Management Command

- [ ] **5.1 Create donkey segment command**
  - File: `donkeycar/management/segment.py`
  - Create command function `segment(args)`
  - Parse arguments: --tub, --lap-detector, --strategy, --min-segment-length, --curvature-threshold, --visualize
  - Load tub from path
  - Create TubStatistics instance
  - Call `compute_segment_assignments()` with parameters (uses all laps)
  - Print summary (sessions processed, segments found per session)
  - Optionally show visualization

- [ ] **5.2 Register command in base.py**
  - File: `donkeycar/management/base.py`
  - Add import: `from .segment import segment`
  - Add to COMMANDS dict: `'segment': segment`

- [ ] **5.3 Test command execution**
  - Create test tub with multi-lap data
  - Run: `donkey segment --tub <path>`
  - Verify metadata is stored in tub
  - Verify car/segment field is written to all records

---

### Phase 6: UI Integration

- [ ] **6.1 Verify UI displays segment data**
  - Load tub with segmentation in donkey UI
  - Verify DataFrame shows `car/segment` column (if computed)
  - Verify DataFrame shows `lap_pct` columns (populated from segment ranking when SEGMENT_PCT_MODE=True)
  - Note: car/segment is stored directly in tub records, no special handling needed

---

### Phase 7: Documentation

- [ ] **7.1 Update CLAUDE.md**
  - File: `donkeycar/CLAUDE.md`
  - Add section: "Segment-Based Performance for Training"
  - Document workflow:
    1. Record multi-lap data
    2. Run `donkey segment --tub <path>` (uses all laps for mean course)
    3. Train with `SEGMENT_PCT_MODE=True`
  - Document iterative training strategy:
    - Train with segment_pct
    - Drive with trained model
    - Collect new data
    - Re-segment and retrain
    - Iterate to improve beyond initial best lap
  - Document configuration parameters (using PctMode enum)
  - Document data structure:
    - car/segment stored in tub records
    - Metadata stores num_segments and segmentation params
    - session_rank structure: session -> lap -> segment
  - Add example usage

---

### Phase 8: Testing

- [ ] **8.1 Integration test: End-to-end workflow**
  - File: `tests/test_segment_training_integration.py`
  - Create test tub with realistic multi-lap data
  - Make specific segments fastest in different laps
  - Run segmentation (writes car/segment to records)
  - Load in TubDataset with pct_mode=PctMode.SEGMENT
  - Verify records from fastest segments have lowest lap_pct values
  - Verify "synthetic best lap" effect (best segments from different laps)

- [ ] **8.2 Regression test: Lap-based training still works**
  - File: `tests/test_lap_pct_regression.py`
  - Verify existing lap-based training unchanged when pct_mode=PctMode.LAP
  - Test backward compatibility with tubs without segmentation

---

## Validation Checklist

After implementation, verify:

- [ ] **Segmentation works**: `donkey segment --tub <path>` completes successfully
- [ ] **car/segment written to records**: All records have car/segment field after segmentation
- [ ] **Metadata is stored**: Tub contains segmentation metadata (num_segments, params)
- [ ] **Training loads segment_pct**: TubDataset populates lap_pct from segment rankings (session -> lap -> segment)
- [ ] **Model trains**: Existing KerasSquarePlusMemoryLap trains with segment_pct
- [ ] **UI shows segments**: Donkey UI displays car/segment and lap_pct columns
- [ ] **Backward compatible**: pct_mode=PctMode.LAP uses original lap-based behavior
- [ ] **Tests pass**: All new and existing tests pass

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

Use this plan as a checklist. Mark items complete with [x] as you finish them.

To resume work, review completed items and continue with the next unchecked item.
