# Web-Based IMU Path Visualizer - Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         User Interface Layer                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌──────────────────┐                   ┌──────────────────┐        │
│  │  Matplotlib UI   │                   │    Web Browser    │        │
│  │  (DEFAULT)       │                   │   (--web flag)    │        │
│  │                  │                   │                   │        │
│  │  • Time slider   │                   │  • Plotly.js viz │        │
│  │  • Lap selector  │                   │  • Controls      │        │
│  │  • Segment radio │                   │  • Info panels   │        │
│  │  • Status panel  │                   │  • Dark theme    │        │
│  │  • Keyboard nav  │                   │  • Touch support │        │
│  └────────┬─────────┘                   └────────┬─────────┘        │
│           │                                      │                   │
└───────────┼──────────────────────────────────────┼───────────────────┘
            │                                      │
            │                                      │ HTTP GET/POST
            │                                      ▼
┌───────────┼──────────────────────────────────────────────────────────┐
│           │              Application Layer                            │
├───────────┼───────────────────────────────────────────────────────────┤
│           │                                      │                    │
│           │                              ┌───────▼───────────┐       │
│           │                              │ Tornado Web Server│       │
│           │                              │ LocalWebController│       │
│           │                              │                   │       │
│           │                              │  /imupath         │       │
│           │                              │  /api/imupath/data│       │
│           │                              └───────┬───────────┘       │
│           │                                      │                    │
│  ┌────────▼───────────┐                 ┌───────▼───────────┐       │
│  │ InteractiveIMU     │                 │ IMUPathDataBuilder│       │
│  │ Visualizer         │                 │                   │       │
│  │                    │                 │ • JSON generator  │       │
│  │ • _run_web_mode()  │◄────────────────┤ • Metadata        │       │
│  │   calls builder    │                 │                   │       │
│  └────────┬───────────┘                 └───────┬───────────┘       │
│           │                                      │                    │
└───────────┼──────────────────────────────────────┼───────────────────┘
            │                                      │
            │    Both use same analysis stack     │
            └──────────────┬──────────────────────┘
                           │
┌──────────────────────────▼───────────────────────────────────────────┐
│                    Course Analysis Layer                              │
├───────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │                    Data Loading (Phase 1)                       │  │
│  │  ┌──────────────────┐              ┌──────────────────┐        │  │
│  │  │ CSVPathDataSource│              │ TubPathDataSource│        │  │
│  │  └────────┬─────────┘              └────────┬─────────┘        │  │
│  │           └─────────────────┬────────────────┘                 │  │
│  │                             │                                   │  │
│  │                       ┌─────▼──────┐                           │  │
│  │                       │  PathData  │                           │  │
│  │                       │ (immutable)│                           │  │
│  │                       └─────┬──────┘                           │  │
│  └─────────────────────────────┼──────────────────────────────────┘  │
│                                │                                      │
│  ┌─────────────────────────────▼──────────────────────────────────┐  │
│  │                  Lap Detection (Phase 2)                        │  │
│  │  ┌──────────────────────┐      ┌──────────────────────┐        │  │
│  │  │ YCrossingLapDetector │      │  DriftLapDetector    │        │  │
│  │  └──────────┬───────────┘      └──────────┬───────────┘        │  │
│  │             └──────────────┬───────────────┘                    │  │
│  │                            │                                     │  │
│  │                      ┌─────▼──────┐                             │  │
│  │                      │ MultiLapData│                            │  │
│  │                      │ + boundaries│                            │  │
│  │                      └─────┬──────┘                             │  │
│  └─────────────────────────────┼──────────────────────────────────┘  │
│                                │                                      │
│  ┌─────────────────────────────▼──────────────────────────────────┐  │
│  │               Mean Course Building (Phase 3)                    │  │
│  │                      ┌─────────────────┐                        │  │
│  │                      │ MeanCourseBuilder│                       │  │
│  │                      └────────┬────────┘                        │  │
│  │                               │                                  │  │
│  │                         ┌─────▼──────┐                          │  │
│  │                         │ MeanCourse │                          │  │
│  │                         │ (x, y, ...)│                          │  │
│  │                         └─────┬──────┘                          │  │
│  └─────────────────────────────────┼──────────────────────────────┘  │
│                                    │                                  │
│  ┌─────────────────────────────────▼──────────────────────────────┐  │
│  │                Segmentation (Phase 4)                           │  │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌──────────┐ │  │
│  │  │ Threshold   │ │  Extrema    │ │  Gradient   │ │  Hybrid  │ │  │
│  │  │Segmentation │ │Segmentation │ │Segmentation │ │Segmentation│ │
│  │  └─────┬───────┘ └──────┬──────┘ └──────┬──────┘ └─────┬────┘ │  │
│  │        └────────────────┼───────────────┴──────────────┘      │  │
│  │                         │                                       │  │
│  │                   ┌─────▼─────────┐                            │  │
│  │                   │CourseSegmenter│                            │  │
│  │                   └─────┬─────────┘                            │  │
│  │                         │                                       │  │
│  │                   ┌─────▼──────────┐                           │  │
│  │                   │CourseSegmentation│                         │  │
│  │                   │  + boundaries   │                          │  │
│  │                   └─────┬──────────┘                           │  │
│  └─────────────────────────┼──────────────────────────────────────┘  │
│                            │                                          │
│  ┌─────────────────────────▼──────────────────────────────────────┐  │
│  │            Segment Assignment (Phase 5)                         │  │
│  │                   ┌────────────────┐                            │  │
│  │                   │SegmentAssigner │                            │  │
│  │                   └────────┬───────┘                            │  │
│  │                            │                                     │  │
│  │                      ┌─────▼──────┐                             │  │
│  │                      │segment_ids │                             │  │
│  │                      │  (array)   │                             │  │
│  │                      └────────────┘                             │  │
│  └─────────────────────────────────────────────────────────────────┘  │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│                        Configuration Layer                              │
├────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  config.py                                                             │
│  ├─ FIELD_AGGREGATIONS           ◄───── Segment stats (Tub only)      │
│  │  └─ time, gyro_z, distance                                         │
│  │                                                                      │
│  └─ LAP_SORTING_CRITERIA         ◄───── Ranking strategy              │
│     └─ time, gyro_z percentiles                                       │
│                                                                         │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│                         Data Flow Summary                               │
├────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  CLI: donkey imupath [--web] <source>                                 │
│                       │                                                 │
│                       ├─ NO FLAG ──► InteractiveIMUVisualizer          │
│                       │               └─► Matplotlib UI               │
│                       │                                                 │
│                       └─ --web ────► LocalWebController                │
│                                       ├─► IMUPathDataBuilder           │
│                                       │   └─► JSON payload             │
│                                       │                                 │
│                                       └─► Browser ◄─► Plotly.js        │
│                                                                         │
│  Both paths use:                                                        │
│  PathData → MultiLapData → MeanCourse → Segmentation → segment_ids   │
│                                                                         │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│                         Key Design Principles                           │
├────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  1. REUSE: Same course_analysis stack for both UIs                    │
│  2. OPT-IN: --web flag, default behavior unchanged                    │
│  3. ADDITIVE: No modifications to existing matplotlib code            │
│  4. MINIMAL: Reuse tornado server, no new dependencies                │
│  5. TESTABLE: Unit tests, manual tests, syntax validation             │
│  7. DOCUMENTED: README, testing guide, inline comments                │
│                                                                         │
└────────────────────────────────────────────────────────────────────────┘
```

## Component Responsibilities

### Backend Components

**IMUPathDataBuilder** (`donkeycar/web/imupath_data.py`)
- Initializes data processing pipeline
- Builds JSON payloads for visualization
- Manages lap/segment metadata
- Supports dynamic reconfiguration

**LocalWebController** (`donkeycar/parts/web_controller/web.py`)
- Tornado web application
- Stores imupath_builder instance
- Routes HTTP requests to handlers

**IMUPathHandler**
- Serves HTML template (`/imupath`)
- Renders imupath.html

**IMUPathDataAPI**
- JSON endpoint (`/api/imupath/data`)
- Query parameters: num_laps, segment_method
- Returns structured JSON payload

**ImuPathCommand** (`donkeycar/management/imupath.py`)
- CLI argument parsing
- `--web` flag handling
- `_run_web_mode()` for server initialization

### Frontend Components

**imupath.html**
- Bootstrap dark theme layout
- Control panel (slider, dropdowns, buttons)
- Info panels (current position, dataset info)
- Plotly.js container

**imupath.js**
- AJAX data loading
- Plotly visualization rendering
- UI event handlers
- Playback animation
- State management

### Shared Components

**PathData** (immutable container)
- timestamp, x, y, heading, velocity arrays
- Read-only to prevent modification

**MultiLapData** (lap boundary info)
- lap_boundaries array
- Per-lap access methods

**MeanCourse** (reference trajectory)
- x, y coordinates
- Length, curvature

**CourseSegmentation** (segment info)
- segments array
- Boundary indices
- Segment types

## Data Flow Example

```
1. User runs: donkey imupath --web test.csv

2. imupath.py:
   - Parses args, detects --web
   - Loads CSV → PathData
   - Creates IMUPathDataBuilder
   - Starts LocalWebController

3. Browser requests: http://localhost:8887/imupath
   - IMUPathHandler serves imupath.html

4. imupath.js loads:
   - Requests: /api/imupath/data
   - IMUPathDataAPI calls builder.build_json_payload()
   - Returns JSON with path_points, mean_course, segments

5. Plotly renders visualization:
   - Scatter plot with speed colors
   - Mean course line
   - Segment markers

6. User interacts:
   - Changes lap count dropdown
   - JS requests: /api/imupath/data?num_laps=2
   - Builder rebuilds mean course with 2 laps
   - Returns updated JSON
   - Plotly re-renders
```

## Extension Points

Want to add new features? Consider:

1. **New visualization types**: Add trace to Plotly in imupath.js
2. **New stats fields**: Add to FIELD_AGGREGATIONS in config
3. **New segment methods**: Add strategy to course_analysis
4. **WebSocket support**: Add tornado.websocket handler
5. **Export features**: Add download button + server endpoint
6. **Multi-session compare**: Add session selector + overlay logic
