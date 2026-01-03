# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## About This Project

Donkey Car is a minimalist and modular self-driving car library for Python, designed for hobbyists and students. It enables building RC cars that drive themselves using machine learning, with support for various sensors, actuators, and ML frameworks.

## Environment Setup

**CRITICAL**: Always ensure the 'donkey' conda environment is activated before running any Python code, tests, or installations. If the environment is not active, ask the user before proceeding.

Check current environment with: `conda info --envs | grep \*`
Activate with: `conda activate donkey`

## Key Commands

### Testing and Development
- `make tests` or `pytest` - Run the full test suite
- `pytest tests/test_specific.py` - Run a single test file
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
- **NEVER push anything to the autorope main branch without explicit user permission**
- **Pushing to docgarbanzo main branch is ALLOWED** for testing and development
- Always confirm which remote you're pushing to before executing git push

**CRITICAL Branch Management:**
- Never push to the **autorope** main branch without explicit user permission
- Pushing to **docgarbanzo** main branch is fine for development/testing
- Always ask before:
  - Making commits to autorope main branch
  - Pushing changes to autorope main branch  
  - Creating pull requests to autorope main branch
- Development should happen on feature branches or new_dev branch, but main branch work on the fork is acceptable

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

### Data Management
- **Tub V2** (`parts/tub_v2.py`) - Data storage format for training data
- **Datastore** (`parts/datastore*.py`) - Data management abstractions
- Pipeline architecture for processing training data

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

- **Python 3.11+** but **< 3.12** (strict requirement)
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