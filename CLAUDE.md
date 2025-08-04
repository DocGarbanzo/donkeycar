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

