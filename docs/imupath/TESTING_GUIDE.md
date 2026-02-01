# Web-Based IMU Path Visualizer - Testing Guide

## Implementation Summary

This implementation adds a web-based IMU path visualizer to Donkeycar as
an opt-in alternative to the matplotlib UI. The feature is accessed via
the `--web` flag on the `donkey imupath` command.

## Quick Start

```bash
# Create test data
python donkeycar/tests/test_imupath_web_manual.py

# Test default matplotlib UI (unchanged)
donkey imupath /tmp/tmpXXXXXX.csv

# Test web UI
donkey imupath --web /tmp/tmpXXXXXX.csv
```

Then open browser to: http://localhost:8887/imupath

## Testing Checklist

### 1. Default Behavior (Matplotlib UI)
- [ ] `donkey imupath --help` shows all options including `--web`
- [ ] `donkey imupath <csv_file>` launches matplotlib UI
- [ ] `donkey imupath <tub_dir>` launches matplotlib UI
- [ ] No changes to existing matplotlib functionality

### 2. Web UI - Basic Functionality
- [ ] `donkey imupath --web <csv_file>` starts web server
- [ ] Server starts on port 8887 (or configured port)
- [ ] `/imupath` page loads in browser
- [ ] Console shows "Web UI available at..." messages

### 3. Web UI - Visualization
- [ ] Path scatter plot renders with speed coloring
- [ ] Colorbar shows speed legend
- [ ] Mean course polyline renders (grey)
- [ ] Segment boundary markers appear
- [ ] Segment labels visible
- [ ] Plot is interactive (zoom, pan, hover)

### 4. Web UI - Controls
- [ ] Time slider moves current position marker
- [ ] Play button starts animation
- [ ] Pause button stops animation
- [ ] Lap selector dropdown populates with lap counts
- [ ] Changing lap count reloads visualization
- [ ] Segment method dropdown works (threshold, extrema, gradient, hybrid)
- [ ] Changing segment method reloads visualization
- [ ] Show/hide driven path checkbox works
- [ ] Show/hide mean course checkbox works

### 5. Web UI - Info Panels
- [ ] Current position panel updates with slider
- [ ] Time, lap, segment display correctly
- [ ] Lap distance updates with slider
- [ ] Segment rank updates for Tub data (if available)
- [ ] Speed and heading update
- [ ] X, Y coordinates shown
- [ ] Dataset info panel shows correct metadata
- [ ] Lap count, point count, duration, distance correct

### 6. Web UI - Different Data Sources
- [ ] Works with CSV files
- [ ] Works with Tub directories
- [ ] Handles single-lap data
- [ ] Handles multi-lap data
- [ ] Stats field dropdown lists available aggregations and fields

### 7. Web UI - Configuration
- [ ] `--config` option loads custom config
- [ ] FIELD_AGGREGATIONS used for Tub stats (if available)

### 8. Error Handling
- [ ] Invalid data source shows error message
- [ ] Missing file shows appropriate error
- [ ] Server handles Ctrl+C gracefully
- [ ] Browser console shows no errors

### 9. Performance
- [ ] JSON payload size is reasonable
- [ ] Plot renders smoothly
- [ ] Time slider responds quickly
- [ ] Playback is smooth

### 10. Cross-Browser Compatibility (if possible)
- [ ] Works in Chrome/Chromium
- [ ] Works in Firefox
- [ ] Works in Safari
- [ ] Works in Edge
- [ ] Works on mobile browsers (touch controls)

## Manual Testing Instructions

### Test 1: Basic CSV Visualization

```bash
# Create test data
python donkeycar/tests/test_imupath_web_manual.py

# Start web UI
donkey imupath --web /tmp/tmpXXXXXX.csv
```

Expected:
- Server starts successfully
- Browser shows circular path with 3 laps
- Speed coloring visible (varying from ~1.5 to 2.5 m/s)
- Mean course calculated from all 3 laps
- Multiple segments detected

### Test 2: Tub Data with Stats

```bash
# Extract test tub
cd /tmp
tar -xzf /path/to/donkeycar/donkeycar/tests/tub/tub.tar.gz

# Start web UI
donkey imupath --web /tmp/tub
```

Expected:
- Tub data loads successfully
- Stats field dropdown appears (if tub has IMU data)
- Segment rankings available

### Test 3: Different Lap Methods

```bash
# Y-crossing (default)
donkey imupath --web --lap-method y_crossing /tmp/test.csv

# Drift detection
donkey imupath --web --lap-method drift /tmp/test.csv
```

Expected:
- Different lap boundaries detected
- Mean course changes accordingly

### Test 4: Different Segment Methods

Use the dropdown in the UI to test:
- Threshold segmentation
- Extrema segmentation
- Gradient segmentation (default)
- Hybrid segmentation

Expected:
- Visualization reloads with new segments
- Different segment boundaries and counts

### Test 5: Playback Animation

1. Click "Play" button
2. Watch time slider advance
3. Observe current position marker moving
4. Click "Pause" to stop
5. Verify playback loops at end

Expected:
- Smooth animation at ~20 FPS
- Current position info updates
- Playback loops continuously

## Automated Testing

```bash
# Run unit tests (requires pytest)
pytest donkeycar/tests/test_imupath_web.py -v

# Expected output:
# test_imupath_data_builder_csv PASSED
# test_imupath_json_payload PASSED
```

## Known Limitations

1. **Dependencies**: Requires internet connection for Plotly.js CDN
2. **Browser Support**: Tested on modern browsers (Chrome, Firefox)
3. **Mobile**: Touch controls work but UI not optimized for small screens
4. **Stats**: Segment rankings computation simplified vs matplotlib UI
5. **Offline**: CDN dependency means offline use requires bundling Plotly

## Troubleshooting

### Server won't start
- Check port 8887 not in use: `lsof -i :8887`
- Try different port: Set `WEB_CONTROL_PORT` in config

### Data not loading
- Check browser console (F12) for errors
- Verify `/api/imupath/data` endpoint returns JSON
- Check server logs for Python errors

### Visualization not rendering
- Ensure Plotly.js loads (check browser network tab)
- Clear browser cache
- Check browser console for JavaScript errors

### Performance issues
- Check network latency to server
- Verify JSON payload size is reasonable

## Success Criteria

The implementation is successful if:

1. ✅ Default behavior unchanged (`donkey imupath` → matplotlib)
2. ✅ `--web` flag launches tornado server
3. ✅ Web UI renders interactive visualization
4. ✅ All controls functional (slider, dropdowns, play/pause)
5. ✅ Works with both CSV and Tub data
6. ✅ Configuration options respected
7. ✅ No breaking changes to existing code
8. ✅ Documentation complete and accurate

## Feedback

When testing, please note:
- What worked well
- What didn't work
- Browser/OS used
- Any error messages
- Performance observations
- Usability issues
- Suggestions for improvements

Report issues with:
- Steps to reproduce
- Expected vs actual behavior
- Screenshots if relevant
- Browser console errors
- Server logs
