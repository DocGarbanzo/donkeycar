# Web-Based IMU Path Visualizer Documentation

This feature adds an opt-in web-based IMU path visualizer to Donkeycar.

## Quick Start

```bash
# Default matplotlib UI (unchanged)
donkey imupath ./recording.csv

# New web UI
donkey imupath --web ./recording.csv
```

Then browse to: http://localhost:8887/imupath

## Documentation

All documentation has been organized in the `docs/imupath/` directory:

- **[README.md](docs/imupath/README.md)** - Feature documentation, usage guide, API reference
- **[TESTING_GUIDE.md](docs/imupath/TESTING_GUIDE.md)** - Testing procedures and checklist
- **[ARCHITECTURE.md](docs/imupath/ARCHITECTURE.md)** - System design and data flow
- **[IMPLEMENTATION_SUMMARY.md](docs/imupath/IMPLEMENTATION_SUMMARY.md)** - Quick implementation overview

## Testing

Manual test script:
```bash
python donkeycar/tests/test_imupath_web_manual.py
```

Unit tests:
```bash
pytest donkeycar/tests/test_imupath_web.py
```
