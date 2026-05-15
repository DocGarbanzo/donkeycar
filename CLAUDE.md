# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## About This Project

Donkey Car is a minimalist and modular self-driving car library for Python, designed for hobbyists and students. It enables building RC cars that drive themselves using machine learning, with support for various sensors, actuators, and ML frameworks.

## Environment Setup

The project uses a global virtual environment at `~/.venvs/donkeycar`.
Activate it before running any Python code or tests:

```zsh
source ~/.venvs/donkeycar/bin/activate
```

Do **not** use conda or create a local `.venv` in the project directory.

**Installing packages**: Always use `uv pip` targeting the global venv:
```zsh
uv pip install --python ~/.venvs/donkeycar/bin/python <package>
```
Never use bare `pip` or `pip3` — they may resolve to the system Python.

## Key Commands

### Testing and Development
- `source ~/.venvs/donkeycar/bin/activate && pytest` - Run the full test suite
- `pytest donkeycar/tests/test_specific.py` - Run a single test file
- `pytest -k "test_name"` - Run specific test by name

## Testing Guidelines

**CRITICAL**: When testing UI features or user workflows, you MUST test the complete end-to-end scenario, not isolated components.

### Integration Testing Requirements

When testing features that involve user interactions or multi-step workflows:

1. **Simulate the actual user workflow**
   - Don't test components in isolation
   - Test the complete sequence of actions users perform
   - Use the same data transformations the real code uses

2. **Use actual detected boundaries, not guessed values**
   - Get lap boundaries from lap detection, don't hardcode indices
   - Use real algorithms to find transitions, don't assume positions
   - Example: `lap1_end = len(multilap_data.laps[0]) - 1`, NOT `lap1_end = 200`

3. **Test correctness, not just validity**
   - Bad: `assert segment >= 0` (checks if valid)
   - Good: `assert segment == expected_segment` (checks if correct)
   - Bad: `assert len(segments) > 0` (meaningless)
   - Good: `assert len(unique_segments_in_lap1) > 1` (proves transitions occur)

4. **Reproduce the exact user scenario**
   - If user selects "2 laps" in UI → compute mean course from 2 laps
   - If UI filters by time → test with time filtering
   - If UI uses specific algorithms → use those same algorithms in tests

### Test Anti-Patterns to Avoid

❌ **The "Valid But Wrong" Test**
```python
# BAD: Just checks if value is in valid range
assert 0 <= segment < total_segments

# GOOD: Checks actual expected value at specific position
assert segment_at_position == expected_segment_id
```

❌ **The "Magic Number" Test**
```python
# BAD: Arbitrary index with no justification
lap1_end = 200  # Where did this come from?

# GOOD: Actual detected boundary
lap1_end = len(multilap_data.laps[0]) - 1
```

❌ **The "Isolated Component" Test**
```python
# BAD: Tests component with artificial data
mean_course = MeanCourse()
mean_course.x = [1, 2, 3]  # Synthetic, doesn't match real workflow

# GOOD: Tests full workflow with real data processing
mean_course = compute_mean_course_from_n_laps(data, num_laps=2)
```

❌ **The "Shallow Assertion" Test**
```python
# BAD: Only checks existence
assert len(segments) > 0

# GOOD: Checks actual behavior
assert segments[after_boundary] != segments[before_boundary]
```

### Required Test Workflow for Bug Fixes

When fixing a bug:

1. **First, write a test that reproduces the bug** (test must FAIL)
2. Understand WHY previous tests didn't catch the bug
3. Fix the code
4. Verify the test now PASSES
5. Add similar tests for related scenarios
6. Document what you learned in commit message

**NEVER** fix code without first having a failing test!

### Example: Testing IMU Path Visualization

**Bad Test (Component Isolation)**:
```python
def test_segment_assignment():
    # Create mean course from 1 lap only
    mean_course = create_mean_course(lap1_data)

    # Assign to 2-lap path
    segments = assign_segments(two_lap_data)

    # Check validity (meaningless!)
    assert all(s >= 0 for s in segments)
```

**Good Test (Integration)**:
```python
def test_segment_assignment_with_2_lap_mean_course():
    # Load 3-lap data
    data = load_multilap_csv(num_laps=3)

    # Simulate user selecting 2 laps in UI
    mean_course = compute_mean_course_from_n_laps(data, num_laps=2)

    # Create segmentation
    segmentation = CourseSegmentation(mean_course)
    segmentation.compute()

    # Assign to full path
    segments = segmentation.assign_segments_to_path(data.x, data.y)

    # Get ACTUAL lap 1 end from detection
    lap1_end = len(multilap_data.laps[0]) - 1

    # Check lap 1 has multiple segments (proves transitions work)
    lap1_segments = set(segments[:lap1_end + 1])
    assert len(lap1_segments) > 1, \
        f"Bug: lap 1 stuck at {lap1_segments}"
```

### Key Lessons

1. **Passing tests don't guarantee correct behavior** - they only test what you told them to test
2. **Simulate the real environment** - UI workflows, user actions, actual data transformations
3. **Don't guess boundaries** - use the actual detection algorithms
4. **Test what matters** - not "is this valid?" but "is this correct?"
5. **Integration tests catch more bugs** - unit tests are necessary but not sufficient

## Branch and Remote Management Policy

**CRITICAL Repository Remote Management:**
- The project has TWO remotes: `autorope` (upstream) and `docgarbanzo` (fork)
- **Always push to docgarbanzo** as long as there is no explicit as from the user to do differently
- **NEVER push anything to the autorope main branch without explicit user permission**
- **Pushing to docgarbanzo main branch is ALLOWED** for testing and development
- Always confirm which remote you're pushing to before executing git push

**CRITICAL Branch Management:**
- Never push to the **autorope** main branch
- Pushing to **docgarbanzo** main branch is fine for development/testing
- Always ask before:
  - Making commits to autorope branch
  - Pushing changes to autorope branch  
- Development should happen on feature branches or dev branch, but main branch work on the fork is acceptable

**Remote Configuration:**
- `autorope` - https://github.com/autorope/donkeycar.git (upstream)
- `docgarbanzo` - git@github.com:DocGarbanzo/donkeycar.git (fork)

### Package Management
- `make package` - Create source distribution
- `pip install -e .` - Install in development mode

### Application Commands
- `donkey createcar --template <template_name>` - Create new car configuration
- `donkey drive --model <model_path>` - Run car with trained model
- `donkey calibrate` - Calibrate steering/throttle
- `donkey train --tub <data_path>` - Train model on collected data

## Architecture Overview

### Parts-Based System
The core architecture uses a modular **parts-based system**:
- **Vehicle** (`vehicle.py`): Central orchestrator managing parts and data flow
- **Parts** (`parts/`): Modular components (cameras, actuators, controllers, ML models)
- **Memory** (`memory.py`): Shared data store for inter-part communication
- **Config** (`config.py`): Configuration management system

### Key Directories
- `donkeycar/parts/` - 70+ modular components (cameras, actuators, sensors, ML)
- `donkeycar/templates/` - Pre-configured car setups (basic.py, complete.py, etc.)
- `donkeycar/management/` - CLI tools and utilities
- `donkeycar/pipeline/` - Data processing for model training

### Threading Model
Parts can be threaded for concurrent execution. Recent commits show optimization work on threaded parts with higher internal refresh rates.

## Development Patterns

### Parts Interface: Threaded vs Non-Threaded

#### Non-Threaded Parts
**Use for**: Fast computations, actuator control, state management, data processing
**Interface**:
- `run(*inputs)` - Core execution method, called once per main loop iteration
- `shutdown()` - Optional cleanup method

**Characteristics**:
- Synchronous execution in main vehicle loop
- Blocking - each part runs to completion before next part
- Direct computation - results computed immediately when called

**Example**:
```python
class DriveMode:
    def run(self, mode, user_angle, user_throttle, pilot_angle, pilot_throttle):
        if mode == 'user':
            return user_angle, user_throttle
        # ... logic
```

#### Threaded Parts  
**Use for**: I/O bound operations, camera capture, sensor reading, continuous monitoring
**Interface**:
- `update()` - Continuous loop method running in background thread
- `run_threaded(*inputs)` - Returns cached state, called from main thread
- `shutdown()` - Cleanup method, sets `self.on = False`

**Characteristics**:
- Asynchronous execution in separate background threads
- Non-blocking - don't block main vehicle loop
- State caching - maintain internal state, return cached results
- Higher refresh rates - can run faster than main loop frequency

**Example**:
```python
class PiCamera(BaseCamera):
    def __init__(self):
        self.on = True
        self.frame = None
        
    def update(self):
        """Runs in background thread"""
        while self.on:
            self.run()  # capture frame
            time.sleep(0)
            
    def run_threaded(self):
        """Called from main thread"""
        return self.frame  # return cached frame
        
    def shutdown(self):
        self.on = False
```

#### Part Registration
```python
# Non-threaded
car.add(part, inputs=['input'], outputs=['output'])

# Threaded  
car.add(part, inputs=['input'], outputs=['output'], threaded=True)
```

### Configuration System
- Use `cfg_*.py` files in templates for different car configurations
- Template system allows easy switching between setups (basic, complete, simulator)
- Configuration-driven approach for different hardware setups

### ML Framework Support
Supports multiple ML frameworks:
- **Keras/TensorFlow** (`parts/keras.py`) - Primary framework
- **PyTorch** (`parts/pytorch/`) - Alternative framework
- **FastAI** (`parts/fastai.py`) - High-level framework

### Apple Silicon / Metal GPU Training

On macOS with `tensorflow-metal==1.2.0`, Adam weight updates inside a compiled
`tf.function` produce incorrect results (Metal PluggableDevice compiler bug).
The fix (Phase B) is split across `KerasInterpreter.compile()` and `.fit()`:

- The Metal GPU is **not hidden** — it is used for every forward and backward op.
- `KerasInterpreter.compile()` sets `_use_metal_train_step=True` on Metal
  (instead of `run_eagerly=True`). The model is compiled normally (not eagerly),
  so the forward/backward pass is compiled inside `tf.function` for speed.
- `KerasInterpreter.fit()` calls `_install_metal_train_step(model)` which
  monkey-patches `model.train_step` to use `tf.py_function` around
  `optimizer.apply_gradients`. `tf.py_function` executes the apply step eagerly
  even inside a compiled `tf.function`, so Adam ops on Metal are correct.
- LiteRT/TFLite inference is completely unaffected.

If a future `tensorflow-metal` release fixes the compiled Adam bug, remove the
`_install_metal_train_step` call from `KerasInterpreter.fit()` and the
`_use_metal_train_step` flag from `KerasInterpreter.compile()`, then re-run
`donkeycar/tests/test_metal_gradients.py` to confirm correctness.

### Data Management
- **Tub V2** (`parts/tub_v2.py`) - Data storage format for training data
- **Datastore** (`parts/datastore*.py`) - Data management abstractions
- Pipeline architecture for processing training data

### Tub Data Integrity (CRITICAL)

**Background:** Tub data corruption occurred when attempting to modify catalog
files directly. This section documents the correct approach to working with tub
data.

#### Tub Data Structure

A tub directory contains:
```
data/
├── manifest.json           # Metadata file (5 lines, can be modified)
├── catalog_0.catalog       # Record data (NEVER modify directly)
├── catalog_0.catalog_manifest
├── catalog_1.catalog
├── catalog_1.catalog_manifest
└── images/                 # Image files referenced by records
```

**manifest.json structure (5 lines):**
1. **Line 1:** Input field names (e.g., `["cam/image_array", "car/pos", ...]`)
2. **Line 2:** Field types (e.g., `["image_array", "vector", ...]`)
3. **Line 3:** Per-session metadata (e.g., `{"session_id": {"laptimer": [...]}}`)
4. **Line 4:** Manifest metadata (sessions info, creation time)
5. **Line 5:** Catalog paths and index info

#### NEVER Modify Catalog Files Directly

**CRITICAL:** The `catalog_*.catalog` files contain immutable sensor recordings.
These files must NEVER be modified after recording because:

1. `Tub.write_record()` filters fields through `input_types` - any field not in
   the original schema is silently dropped
2. Overwriting records strips fields like `car/pos`, `car/distance`, etc.
3. There is no schema migration - once data is lost, it cannot be recovered

**What happened (2026-01-11):** Attempting to add `car/segment` field to
existing records via `write_record()` corrupted 3500+ records, stripping all
IMU data fields.

#### Safe Ways to Add Computed Data

**DO: Store in manifest.json metadata (Line 3)**
```python
# Safe: Adds computed data to session metadata
session_dict = tub.manifest.metadata.setdefault(session_id, {})
session_dict['segmentation'] = {
    'num_segments': 5,
    'segment_boundaries': [...],
    'rankings': {...}
}
tub.manifest.write_metadata()  # Only updates manifest.json
```

**DO: Create separate analysis files**
```python
# Safe: Store analysis results in separate file
import json
with open(f'{tub_path}/segment_analysis.json', 'w') as f:
    json.dump(analysis_results, f)
```

**DON'T: Modify catalog records**
```python
# DANGEROUS: This corrupts data!
for record in tub:
    record['car/segment'] = segment_id
    tub.write_record(record)  # Strips fields not in input_types!
```

#### Safe Tub Operations

| Operation | Method | Safe? |
|-----------|--------|-------|
| Read records | `for record in tub:` | ✅ Yes |
| Add new records during recording | `TubWriter.run()` | ✅ Yes |
| Delete records | `tub.delete_records()` | ✅ Yes (marks deleted) |
| Update session metadata | `tub.manifest.metadata[session_id] = {...}` | ✅ Yes |
| Write manifest metadata | `tub.manifest.write_metadata()` | ✅ Yes |
| Overwrite existing records | `tub.write_record(record)` with `_index` | ❌ DANGEROUS |

#### Example: Laptimer Data (Safe Pattern)

The laptimer correctly stores computed lap times in metadata, not records:
```python
# From TubWriter.close() - CORRECT approach
if self.lap_timer:
    self.tub.manifest.metadata[self.tub.manifest.session_id[1]] \
        = dict(laptimer=self.lap_timer.to_list())
# Only manifest.json is modified, catalogs untouched
```

#### Segment Rankings Design

For segment-based training rankings, store in manifest metadata:
```python
session_dict['segment_rankings'] = {
    'lap_1': {'segment_0': 0.85, 'segment_1': 0.92, ...},
    'lap_2': {'segment_0': 0.78, 'segment_1': 0.88, ...},
}
tub.manifest.write_metadata()
```

The training pipeline reads rankings from metadata, not from individual records.

## Coding Guidelines

### Code Style
- **Line length**: Maximum 80 characters per line
- **Function length**: Keep functions short and focused on single responsibility
- **Function arguments**: Limit number of parameters - use configuration objects or data classes for complex parameter sets
- **No nested code**: Use early returns and extract functions instead of nested loops/if-else statements

### Code Structure
- **Object-Oriented approach**: Prefer classes and methods over procedural code
- **No code duplication**: Extract common functionality into reusable functions/classes
- **Single responsibility**: Each function/class should have one clear purpose
- **Early returns**: Exit functions early on error conditions or edge cases

### Examples

**Good - Early return, no nesting**:
```python
def process_sensor_data(self, data):
    if not data:
        return None
    
    if not self.is_calibrated:
        self.logger.warning("Sensor not calibrated")
        return None
    
    return self.transform_data(data)
```

**Bad - Nested structure**:
```python
def process_sensor_data(self, data):
    if data:
        if self.is_calibrated:
            return self.transform_data(data)
        else:
            self.logger.warning("Sensor not calibrated")
            return None
    else:
        return None
```

**Good - Extract functions to avoid long methods**:
```python
def run(self, image, steering, throttle):
    if not self._validate_inputs(image, steering, throttle):
        return None, None
    
    processed_image = self._preprocess_image(image)
    steering_output = self._calculate_steering(processed_image)
    throttle_output = self._calculate_throttle(steering_output)
    
    return steering_output, throttle_output
```

## Python Requirements

- **Python 3.13** (strict requirement, single version)
- Platform-specific dependencies for RPi, Jetson, PC, macOS
- Development dependencies include pytest, mypy for type checking

## Testing

- Uses pytest with coverage support
- Test configuration in pytest.ini with custom warning filters
- CI/CD via GitHub Actions with matrix testing (macOS, Ubuntu)
- Tests located in `tests/` directory with 40+ test files

## IMU Path Visualization and Analysis

**Command:** `donkey imupath --path <path_to_data>`

Visualizes recorded vehicle trajectories, detects laps, computes mean reference
courses, and segments courses into geometric features.

**Segment stats:** For Tub data, imupath computes segment performance on the
fly using `FIELD_AGGREGATIONS` (the single source of truth for both field
aggregation and ranking). It loads `./config.py` by default; pass `--config`
to use another config and include custom tub fields in the Segment Stats
selector.
Web UI segment stats use TubStatistics session rankings from manifest
metadata, so `donkey segment` must have stored segmentation data for the
session.
If `car/lap` is missing or constant, the web UI falls back to visual lap
detection for ranking. If `car/segment` is missing or constant, the web UI
uses visual segment IDs for ranking.
Changing lap count or segmentation method in the UI recomputes the stats.
The UI prints "Computing segment statistics..." on startup for tub sources.
Segment stats ignore trailing partial laps beyond the last boundary.
Lap labels are 0-based to match record indices.

**Segment Assignment Invariant**: Each lap visits segments sequentially exactly
once: 0 → 1 → 2 → ... → N-1 → 0 (lap complete). Segment boundaries are infinite
lines. A lap completes only after crossing all segment boundaries in order and
returning to segment 0. It is impossible for a lap to visit the same segment
multiple times.

**Lap Definition for Segment Statistics**: When computing segment performance
statistics, lap boundaries MUST be defined by segment cycle completions (N-1 →
0 transitions), NOT by Y-crossing lap detection. Y-crossing is for visualization
only. Using Y-crossing boundaries for segment statistics creates misaligned laps
that appear to visit segments multiple times, violating the segment assignment
invariant.

### Running the Web-Based IMU Path Visualizer

**CRITICAL**: The web server must be run from within a car directory (e.g.,
`~/cars/hyper`) and must point to the `data/` subdirectory containing the tub.

**Command (from terminal):**
```zsh
cd ~/cars/hyper && conda activate donkey && donkey imupath --web data
```

**Command (from Claude Code):**
```zsh
cd ~/cars/hyper && source /opt/miniconda3/etc/profile.d/conda.sh && \
  conda activate donkey && donkey imupath --web data
```

**Common mistakes to avoid:**
- ❌ Running from repo root: `donkey imupath --web ~/cars/hyper`
  (Will fail: FileNotFoundError for catalog_manifest)
- ❌ Running without specifying data directory: `donkey imupath --web .`
  (Will fail: tries to use parent directory as tub)
- ❌ Using `conda activate` directly in Claude Code
  (Will fail: conda shell not initialized)
- ✅ Correct: `cd ~/cars/hyper && donkey imupath --web data` (with env active)

**Server details:**
- Default port: `8887` (configurable via `WEB_CONTROL_PORT` in config.py)
- Access at: `http://localhost:8887/imupath`
- Or: `http://<hostname>.local:8887/imupath`

**To stop the server:**
```bash
# Find and kill the process
lsof -i :8887 | grep LISTEN | awk '{print $2}' | xargs -r kill
```

**Features:**
- Dense point cloud visualization (10,000 points default)
- Segment boundary normal lines
- Time formatting (MM:SS.S)
- Fast keyboard navigation with arrow keys (← →)
- Live segment statistics with configurable field aggregations

### Key Files

- `donkeycar/utilities/imu_visualization.py` - Main UI (matplotlib)
- `donkeycar/course_analysis/` - Modular implementation:
  - `data_loader.py` - PathData container, CSV/Tub loading
  - `lap_detection.py` - YCrossingLapDetector, DriftLapDetector
  - `mean_course.py` - MeanCourseBuilder
  - `segmentation.py` - 4 strategies: threshold, extrema, gradient, hybrid
  - `segment_assignment.py` - SegmentAssigner, SegmentEstimator
- `donkeycar/course_analysis/old/course_analysis.py` - Legacy (pending deprecation)

### Data Format

**CSV:** `t, x, y, h, v` (timestamp, position x/y in meters, heading in degrees,
velocity in m/s)

**Tub:** Extracts from `_timestamp_ms`, `car/pos`, `car/euler`, `car/speed`

### Critical Design Constraint: Two-Stage Segment Assignment

SegmentAssigner uses TWO methods:

1. **Initial detection (index 0 only):** Nearest-neighbor to find starting
   segment
2. **Crossing detection (all subsequent):** Tangent projection - detects when
   path crosses boundary lines perpendicular to course direction

**Why:** Nearest-neighbor finds where you start; tangent projection tracks
progress along the course regardless of cross-track offset.

**Single source of truth:** Use `self.segmentation.segment_boundaries` directly,
never duplicate boundary storage.

## Segment-Based Performance for Training

**Concept:** Instead of training on best complete laps, train on best-driven
instances of each segment across all laps. This creates a "synthetic perfect
lap" that outperforms any single recorded lap.

### Workflow

1. **Record multi-lap data** - Drive multiple laps with IMU enabled
2. **Compute segment assignments** - Run segmentation on the tub
   ```bash
   donkey segment --tub ./data/tub_1
   ```
3. **Train with segment performance** - Enable in config and train
   ```python
   SEGMENT_PCT_MODE = True
   ```
   ```bash
   python manage.py train --tub ./data/tub_1
   ```

### How It Works

**Data Structure:**
- Segment assignments stored in manifest metadata (NOT in catalog records)
- Metadata stores: segmentation parameters, segment boundaries, rankings
- Performance rankings computed from FIELD_AGGREGATIONS:
  `session_rank[session_id][lap_num][segment_id] = {field1_pct, field2_pct, ...}`
- The `lap_pct` vector passed to training matches FIELD_AGGREGATIONS order:
  `[time_pct, distance_pct, gyro_z_pct, ...]`

**IMPORTANT:** See "Tub Data Integrity" section - segment data is computed at
training time from manifest metadata, NOT stored in individual records.

**Single source of truth:** `FIELD_AGGREGATIONS` defines both what gets
aggregated AND how laps/segments are ranked. The order of entries determines
ranking priority (first entry is primary sort key).

**Example:** 3 laps, 4 segments per lap

Lap 1: Segments [Fast, Slow, Medium, Fast]
Lap 2: Segments [Medium, Fast, Fast, Slow]
Lap 3: Segments [Slow, Medium, Slow, Medium]

Training prioritizes:
- Segment 0 from Lap 1 (fastest instance of segment 0)
- Segment 1 from Lap 2 (fastest instance of segment 1)
- Segment 2 from Lap 2 (fastest instance of segment 2)
- Segment 3 from Lap 1 (fastest instance of segment 3)

This creates a "synthetic best lap" combining the best-driven instances of each
segment!

### Configuration

**Primary config:** `donkeycar/templates/cfg_donkey5.py` (cfg_complete.py uses
deprecated LAP_SORTING_CRITERIA for backward compatibility)

```python
# Enable segment-based training
SEGMENT_PCT_MODE = True  # True = segment-based, False = lap-based

# FIELD_AGGREGATIONS: Single source of truth for:
# 1. Which fields to aggregate per lap/segment
# 2. How to rank laps/segments (order matters!)
# 3. What goes into the lap_pct vector for training

def abs_transform(value):
    return abs(value)

FIELD_AGGREGATIONS = [
    # Primary ranking: lap/segment time
    {'output_key': 'time'},        # Boundary field (no 'field' key)
    # Secondary ranking: distance
    {'output_key': 'distance'},    # Boundary field
    # Tertiary ranking: smoothness via gyro Z-axis
    {
        'field': 'car/gyro',       # Record field
        'index': 2,
        'output_key': 'gyro_z_agg',
        'transform': abs_transform,
        'aggregation': 'avg'
    }
]

# To train using ONLY time and distance (no behavioral metrics):
# FIELD_AGGREGATIONS = [
#     {'output_key': 'time'},
#     {'output_key': 'distance'}
# ]
```

**Field types:**
- **Boundary fields**: Computed from lap/segment timing (time, distance).
  No 'field' key.
- **Record fields**: Extracted from tub records (gyro, accel, speed). Have
  'field' key.

### Iterative Training Strategy

1. Train with segment_pct on initial multi-lap data
2. Drive with trained model (will perform better in some segments)
3. Collect new data from model-driven laps
4. Re-segment combined data (original + new laps)
5. Retrain - model learns from new best segments
6. Repeat - iteratively improve beyond initial human best lap

### Implementation Details

**PctMode Enum** (`donkeycar/pipeline/types.py`):
- `PctMode.NONE` - No performance ranking
- `PctMode.LAP` - Lap-based ranking (original behavior)
- `PctMode.SEGMENT` - Segment-based ranking (new feature)

**Key Methods:**
- `TubStatistics.compute_segment_assignments()` - Computes segments, stores in
  manifest metadata (NOT in catalog records)
- `TubStatistics.calculate_segment_performance()` - Ranks segment instances
- `TubDataset.__init__(pct_mode=PctMode.SEGMENT)` - Enables segment mode
- `TubRecord.extend()` - Populates `lap_pct` from segment rankings in metadata

**Command:**
```bash
donkey segment --help
donkey segment --tub ./data/tub_1 --strategy hybrid
donkey segment --tub ./data/tub_1 --min-segment-length 1.0
```

## Remote Development Workflow

### Raspberry Pi Development Setup

When developing for Raspberry Pi deployment, use this workflow:

**Development Environment:**
- Local development machine with donkeycar repo
- Raspberry Pi accessible via SSH at `hyper.local`
- Git repo on Pi located at `~/projects/donkeycar`
- Car application directory at `~/mycar`

**Deployment Steps:**

1. **Local Development:**
   ```bash
   # Make changes to donkey5.py or other files
   git add . && git commit -m "description"
   git push
   ```

2. **Deploy to Raspberry Pi:**
   ```bash
   ssh hyper.local
   cd ~/projects/donkeycar
   git pull origin new_dev
   
   # Update car application with latest template
   cd ~/mycar
   donkey update --template donkey5
   ```

3. **Test on Pi:**
   ```bash
   # Run car application
   ./manage.py drive
   
   # Observe logging output and adjust logging.conf if needed
   # The logging.conf file should be in ~/mycar directory
   ```

4. **Iterate:** Repeat steps 1-3 until functionality works correctly

**Logging Configuration:**
- Car app uses rotating file handler + console output with timestamps
- Optional `logging.conf` in car directory for module-specific debug levels
- Uses configparser approach (not fileConfig) to preserve console/file handlers
- Example working logging.conf:
  ```ini
  [loggers]
  keys=root,actuator,transform

  [handlers]
  keys=

  [formatters]
  keys=

  [logger_root]
  level=INFO

  [logger_actuator]
  level=DEBUG
  qualname=donkeycar.parts.actuator

  [logger_transform]
  level=DEBUG
  qualname=donkeycar.parts.transform
  ```

**Key Points:**
- **CRITICAL:** Never use `handlers=` lines in logger sections - they override console/file output
- **CRITICAL:** donkey5.py uses configparser, not fileConfig(), to avoid handler disruption
- The `donkey update` command copies template files to car directory
- Changes to core library require git pull + update cycle
- Working logging shows: `2025-07-12 12:47:40,939 [INFO] donkeycar.parts.actuator __init__: RCReceiver created`
- Documentation at docs.donkeycar.com covers main branch; new_dev may differ
- To move code from the donkey car project to the pi, you have to go into ~/projects/donkeycar on the pi and do a git pull there after changing code here and pushing
