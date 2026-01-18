/**
 * Donkey IMU Path Visualizer - Web UI
 * Interactive visualization using Plotly.js
 */

// Configuration constants
const PLAYBACK_INTERVAL_MS = 50;  // 50ms interval for playback
// Convert to seconds
const PLAYBACK_TIME_INCREMENT = PLAYBACK_INTERVAL_MS / 1000.0;

// Global state
let appState = {
  data: null,
  currentTimeIndex: 0,
  isPlaying: false,
  playInterval: null,
  plotInitialized: false,
  currentPathTraceIndex: null,  // Index of current path trace for updates
  showDrivenPath: true,  // Preserve visibility state across reloads
  showMeanCourse: true,  // Preserve visibility state across reloads
  // Currently selected stats field for segment ranking
  currentStatsField: null,
  resizeTimer: null,
  plotUpdateTimer: null,  // Debounce timer for plot updates
  pendingPlotUpdate: false,  // Flag for pending plot update
};

/**
 * Format duration in seconds as MM:SS.S
 */
function formatDuration(seconds) {
  const mins = Math.floor(seconds / 60);
  const secs = (seconds % 60).toFixed(1);
  return `${mins}:${secs.padStart(4, '0')}`;
}

/**
 * Format unix timestamp as date string (YYYY-MM-DD)
 */
function formatDate(unixTimestamp) {
  const date = new Date(unixTimestamp * 1000);
  const year = date.getFullYear();
  const month = String(date.getMonth() + 1).padStart(2, '0');
  const day = String(date.getDate()).padStart(2, '0');
  return `${year}-${month}-${day}`;
}

/**
 * Format unix timestamp as time string with milliseconds (HH:MM:SS.mmm)
 */
function formatTime(unixTimestamp) {
  const date = new Date(unixTimestamp * 1000);
  const hours = String(date.getHours()).padStart(2, '0');
  const minutes = String(date.getMinutes()).padStart(2, '0');
  const seconds = String(date.getSeconds()).padStart(2, '0');
  const ms = String(date.getMilliseconds()).padStart(3, '0');
  return `${hours}:${minutes}:${seconds}.${ms}`;
}

/**
 * Find the start index for a given lap
 */
function findLapStartIndex(lapNum) {
  if (!appState.data || lapNum <= 0) return 0;
  const pathPoints = appState.data.path_points;
  for (let i = 0; i < pathPoints.length; i++) {
    if (pathPoints[i].lap === lapNum) return i;
  }
  return 0;
}

/**
 * Calculate cumulative distance from start index to end index
 */
function calculateCumulativeDistance(toIndex, fromIndex = 0) {
  if (!appState.data || toIndex <= fromIndex) return 0;
  const pathPoints = appState.data.path_points;
  let dist = 0;
  for (let i = fromIndex + 1; i <= toIndex && i < pathPoints.length; i++) {
    const p1 = pathPoints[i - 1];
    const p2 = pathPoints[i];
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    dist += Math.sqrt(dx * dx + dy * dy);
  }
  return dist;
}

/**
 * Update the current path line trace on the plot
 */
function updateCurrentPathLine(closestIdx) {
  const pathPoints = appState.data.path_points;
  const pathX = pathPoints.slice(0, closestIdx + 1).map(p => p.x);
  const pathY = pathPoints.slice(0, closestIdx + 1).map(p => p.y);
  Plotly.restyle('plot-container', { x: [pathX], y: [pathY] }, 2);
}

/**
 * Update the current position marker on the plot
 */
function updateCurrentPositionMarker(point) {
  Plotly.restyle('plot-container', { x: [[point.x]], y: [[point.y]] }, 3);
}

/**
 * Get segment ranking for the current point and selected stats field.
 */
function getSegmentRanking(point) {
  if (!appState.data || !appState.data.rankings) return null;
  const rankings = appState.data.rankings.segments;
  if (!rankings) return null;
  if (point.lap === null || point.segment === null) return null;
  if (!appState.currentStatsField) return null;
  const lapRanks = rankings[String(point.lap)];
  if (!lapRanks) return null;
  const segRanks = lapRanks[String(point.segment)];
  if (!segRanks) return null;
  const value = segRanks[appState.currentStatsField];
  if (value === undefined || value === null) return null;
  return value;
}

// Initialize when page loads
$(document).ready(function() {
  console.log('IMU Path Visualizer loading...');
  loadData();
  setupEventHandlers();
});

/**
 * Load IMU path data from server
 */
function loadData(numLaps = null, segmentMethod = null) {
  let url = '/api/imupath/data';
  const params = [];
  if (numLaps !== null) params.push(`num_laps=${numLaps}`);
  if (segmentMethod !== null) params.push(`segment_method=${segmentMethod}`);
  if (params.length > 0) url += '?' + params.join('&');
  
  $.ajax({
    url: url,
    method: 'GET',
    dataType: 'json',
    success: function(data) {
      console.log('Data loaded:', data);
      appState.data = data;
      initializeUI();
      $('#loading').hide();
      $('#main-content').show();
      renderPlot();
      applyVisibilityState();  // Restore visibility after render
      updateInfoPanels();
    },
    error: function(xhr, status, error) {
      console.error('Error loading data:', error);
      $('#loading').hide();
      $('#error').text('Error loading IMU path data: ' + error).show();
    }
  });
}

/**
 * Initialize UI controls with data
 */
function initializeUI() {
  const data = appState.data;
  const metadata = data.metadata;

  // Populate lap selector
  const lapSelect = $('#lap-selector');
  lapSelect.empty();
  for (let i = 1; i <= metadata.total_laps; i++) {
    const label = i === 1 ? '1 lap' : `${i} laps`;
    const selected = i === metadata.num_laps ? 'selected' : '';
    lapSelect.append(`<option value="${i}" ${selected}>${label}</option>`);
  }

  // Set segment method
  $('#segment-method').val(metadata.segment_method);

  // Setup time slider
  const times = data.path_points.map(p => p.t);
  const minTime = Math.min(...times);
  const maxTime = Math.max(...times);
  $('#time-slider').attr('min', minTime).attr('max', maxTime).val(minTime);

  // Setup stats field selector if available
  if (metadata.available_stats && metadata.available_stats.length > 0) {
    const statsSelect = $('#stats-field');
    statsSelect.empty();
    metadata.available_stats.forEach(stat => {
      // Format display name (replace underscores, title case)
      const displayName = stat.replace(/_/g, ' ')
        .replace(/\b\w/g, c => c.toUpperCase());
      statsSelect.append(`<option value="${stat}">${displayName}</option>`);
    });
    appState.currentStatsField = metadata.available_stats[0];
    $('#stats-control').show();
  } else {
    appState.currentStatsField = null;
    $('#stats-control').hide();
  }
}

/**
 * Setup event handlers
 */
function setupEventHandlers() {
  // Time slider
  $('#time-slider').on('input', function() {
    if (!appState.data) return;
    const time = parseFloat($(this).val());
    updateTimePosition(time);
  });
  
  // Play/Pause buttons
  $('#play-btn').click(function() {
    if (!appState.data) return;
    startPlayback();
  });
  
  $('#pause-btn').click(function() {
    stopPlayback();
  });

  // Shutdown button
  $('#shutdown-btn').click(function() {
    if (confirm('Shutdown the server?')) {
      $.ajax({
        url: '/api/imupath/shutdown',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({confirm: 'shutdown'}),
        success: function() {
          $('#shutdown-btn').text('Stopped').prop('disabled', true);
        },
        error: function() {
          alert('Failed to shutdown server');
        }
      });
    }
  });

  // Lap selector
  $('#lap-selector').change(function() {
    if (!appState.data) return;
    const numLaps = parseInt($(this).val());
    stopPlayback();
    loadData(numLaps, appState.data.metadata.segment_method);
  });
  
  // Segment method selector
  $('#segment-method').change(function() {
    if (!appState.data) return;
    const method = $(this).val();
    stopPlayback();
    loadData(appState.data.metadata.num_laps, method);
  });
  
  // Display toggles
  $('#show-path').change(function() {
    if (!appState.data) return;
    togglePathVisibility();
  });

  $('#show-mean-course').change(function() {
    toggleMeanCourseVisibility();
  });

  // Stats field selector
  $('#stats-field').change(function() {
    appState.currentStatsField = $(this).val();
    // Refresh display if we have ranking data
    if (appState.data && appState.data.rankings) {
      const time = parseFloat($('#time-slider').val());
      updateTimePosition(time);
    }
  });

  $(window).on('resize', function() {
    if (!appState.plotInitialized) return;
    if (appState.resizeTimer) {
      clearTimeout(appState.resizeTimer);
    }
    appState.resizeTimer = setTimeout(function() {
      Plotly.Plots.resize('plot-container');
    }, 100);
  });

  // Keyboard controls for left/right arrow keys
  $(document).on('keydown', function(e) {
    if (!appState.data) return;

    const pathPoints = appState.data.path_points;
    const currentIdx = appState.currentTimeIndex;

    if (e.key === 'ArrowLeft' && currentIdx > 0) {
      // Move backward one point
      e.preventDefault();
      updateByIndex(currentIdx - 1);
    } else if (e.key === 'ArrowRight' && currentIdx < pathPoints.length - 1) {
      // Move forward one point
      e.preventDefault();
      updateByIndex(currentIdx + 1);
    }
  });
}

/**
 * Render the main plot using Plotly
 */
function renderPlot() {
  const data = appState.data;
  const pathPoints = data.path_points;
  const meanCourse = data.mean_course;
  const segments = data.segments;
  
  // Extract coordinates
  const pathX = pathPoints.map(p => p.x);
  const pathY = pathPoints.map(p => p.y);
  const pathV = pathPoints.map(p => p.v);
  
  const meanX = meanCourse.map(p => p.x);
  const meanY = meanCourse.map(p => p.y);
  
  // Create traces
  const traces = [];
  
  // Driven path scatter (colored by speed)
  traces.push({
    x: pathX,
    y: pathY,
    mode: 'markers',
    type: 'scatter',
    name: 'Driven Path',
    marker: {
      size: 4,
      color: pathV,
      colorscale: 'Viridis',
      showscale: true,
      colorbar: {
        title: {
          text: 'Speed (m/s)',
          font: {
            family: 'Courier New, monospace',
            size: 11,
          },
        },
        x: 0.98,
        xanchor: 'right',
        tickfont: {
          family: 'Courier New, monospace',
          size: 11,
        },
      },
    },
    hovertemplate: (
      'X: %{x:.2f}m<br>Y: %{y:.2f}m<br>' +
      'Speed: %{marker.color:.2f}m/s<extra></extra>'
    ),
    visible: true,
  });
  
  // Mean course line
  traces.push({
    x: meanX,
    y: meanY,
    mode: 'lines',
    type: 'scatter',
    name: 'Mean Course',
    line: {
      color: '#808080',
      width: 3.5,
    },
    hovertemplate: 'X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>',
    visible: true,
  });
  
  // Current path line (from start to current position) - red line
  traces.push({
    x: [pathX[0]],
    y: [pathY[0]],
    mode: 'lines',
    type: 'scatter',
    name: 'Current Path',
    line: {
      color: '#FF6B6B',
      width: 1.5,
    },
    hoverinfo: 'skip',
  });
  appState.currentPathTraceIndex = traces.length - 1;

  // Current position marker
  traces.push({
    x: [pathX[0]],
    y: [pathY[0]],
    mode: 'markers',
    type: 'scatter',
    name: 'Current Position',
    marker: {
      size: 12,
      color: '#FF6B6B',
      symbol: 'circle',
      line: {
        color: 'white',
        width: 2,
      },
    },
    hoverinfo: 'skip',
  });

  // Add segment boundary normal lines (solid lines, not dashed)
  const segmentBoundaries = data.segment_boundaries || [];
  segmentBoundaries.forEach(boundary => {
    traces.push({
      x: [boundary.line.x1, boundary.line.x2],
      y: [boundary.line.y1, boundary.line.y2],
      mode: 'lines',
      type: 'scatter',
      name: 'Boundary',
      line: {
        color: '#808080',
        width: 2.5,
      },
      showlegend: false,
      hoverinfo: 'skip',
    });
  });
  
  // Build segment label annotations (round circles like matplotlib version)
  const annotations = createSegmentAnnotations(true);

  // Layout
  const layout = {
    title: {
      text: 'IMU Path Visualization',
      font: {
        family: 'Courier New, monospace',
        size: 11,
        color: '#e0e0e0',
      },
    },
    xaxis: {
      title: {
        text: 'X Position (m)',
        font: {
          family: 'Courier New, monospace',
          size: 11,
        },
      },
      gridcolor: '#333',
      color: '#e0e0e0',
      scaleanchor: 'y',
      scaleratio: 1,
      tickfont: {
        family: 'Courier New, monospace',
        size: 11,
      },
    },
    yaxis: {
      title: {
        text: 'Y Position (m)',
        font: {
          family: 'Courier New, monospace',
          size: 11,
        },
      },
      gridcolor: '#333',
      color: '#e0e0e0',
      tickfont: {
        family: 'Courier New, monospace',
        size: 11,
      },
    },
    plot_bgcolor: '#000',
    paper_bgcolor: '#1a1a1a',
    hovermode: 'closest',
    showlegend: true,
    legend: {
      x: 0,
      y: 1,
      bgcolor: 'rgba(0,0,0,0.5)',
      font: {
        family: 'Courier New, monospace',
        size: 11,
        color: '#e0e0e0',
      },
    },
    margin: { l: 50, r: 24, t: 50, b: 50 },
    annotations: annotations,
    font: {
      family: 'Courier New, monospace',
      size: 11,
    },
  };
  
  const config = {
    responsive: true,
    displayModeBar: true,
    modeBarButtonsToRemove: ['lasso2d', 'select2d'],
  };
  
  // Render plot
  Plotly.newPlot('plot-container', traces, layout, config);
  appState.plotInitialized = true;
}

/**
 * Update display by index (fast path for keyboard navigation)
 */
function updateByIndex(idx) {
  const pathPoints = appState.data.path_points;
  appState.currentTimeIndex = idx;
  const point = pathPoints[idx];

  // Update slider to match
  $('#time-slider').val(point.t);

  // Fast update - only marker, skip expensive operations
  updateDisplayFast(idx, point);
}

/**
 * Fast update for keyboard navigation - uses CSS overlay marker
 */
function updateDisplayFast(closestIdx, point) {
  // Update slider index display (instant)
  const totalPoints = appState.data.path_points.length;
  $('#idx-value').text(`${closestIdx}/${totalPoints - 1}`);

  // Update position panel (instant - no expensive calculations)
  $('#current-date').text(formatDate(point.t));
  $('#current-time').text(formatTime(point.t));
  $('#current-idx').text(closestIdx);
  $('#current-lap').text(point.lap !== null ? point.lap + 1 : '--');
  $('#current-segment').text(point.segment !== null ? point.segment : '--');
  $('#current-speed').text(point.v.toFixed(2));
  $('#current-heading').text((point.h * 180 / Math.PI).toFixed(1) + '°');
  $('#current-x').text(point.x.toFixed(2));
  $('#current-y').text(point.y.toFixed(2));

  // Update driven path and marker on plot
  if (appState.plotInitialized) {
    const pathPoints = appState.data.path_points;
    const pathX = pathPoints.slice(0, closestIdx + 1).map(p => p.x);
    const pathY = pathPoints.slice(0, closestIdx + 1).map(p => p.y);

    Plotly.restyle('plot-container', {
      x: [pathX, [point.x]],
      y: [pathY, [point.y]],
    }, [2, 3]);
  }

  // Skip distance calculations for speed
}

/**
 * Update time position
 *
 * Note: This function assumes path_points are sorted by monotonically
 * increasing 't' (timestamp). This is guaranteed by the data source
 * (PathData from course_analysis) which maintains temporal ordering.
 */
function updateTimePosition(time) {
  const data = appState.data;
  const pathPoints = data.path_points;

  // Find closest point to this time using binary search
  // Binary search is efficient (O(log n)) because timestamps are sorted
  let closestIdx = 0;
  if (pathPoints.length > 1) {
    let left = 0;
    let right = pathPoints.length - 1;

    // Find first index where pathPoints[idx].t >= time
    while (left < right) {
      const mid = Math.floor((left + right) / 2);
      if (pathPoints[mid].t < time) {
        left = mid + 1;
      } else {
        right = mid;
      }
    }

    // 'left' is the first index with t >= time (or 0 if all are >= time).
    // Compare this point with the previous one (if any) to get the closest.
    const idxAfter = left;
    const idxBefore = left > 0 ? left - 1 : left;

    const diffBefore = Math.abs(pathPoints[idxBefore].t - time);
    const diffAfter = Math.abs(pathPoints[idxAfter].t - time);

    closestIdx = diffBefore <= diffAfter ? idxBefore : idxAfter;
  }

  appState.currentTimeIndex = closestIdx;
  const point = pathPoints[closestIdx];

  // Update display
  updateDisplayForPoint(closestIdx, point);
}

/**
 * Update all display elements for a given point
 */
function updateDisplayForPoint(closestIdx, point) {
  const pathPoints = appState.data.path_points;

  // Update current path line and marker position on plot
  if (appState.plotInitialized) {
    // Get path up to current position
    const pathX = pathPoints.slice(0, closestIdx + 1).map(p => p.x);
    const pathY = pathPoints.slice(0, closestIdx + 1).map(p => p.y);

    // Batch update both traces at once for better performance
    Plotly.restyle('plot-container', {
      x: [pathX, [point.x]],
      y: [pathY, [point.y]],
    }, [2, 3]);
  }

  // Update slider index display
  const totalPoints = appState.data.path_points.length;
  $('#idx-value').text(`${closestIdx}/${totalPoints - 1}`);

  // Update position panel
  $('#current-date').text(formatDate(point.t));
  $('#current-time').text(formatTime(point.t));
  $('#current-idx').text(closestIdx);
  $('#current-lap').text(point.lap !== null ? point.lap + 1 : '--');
  $('#current-segment').text(point.segment !== null ? point.segment : '--');
  $('#current-speed').text(point.v.toFixed(2));
  $('#current-heading').text((point.h * 180 / Math.PI).toFixed(1) + '°');
  $('#current-x').text(point.x.toFixed(2));
  $('#current-y').text(point.y.toFixed(2));

  // Calculate and display distances
  const totalDist = calculateCumulativeDistance(closestIdx);
  $('#current-total-dist').text(totalDist.toFixed(1) + 'm');

  // Calculate lap distance
  const lapNum = point.lap !== null ? point.lap : 0;
  const lapStartIdx = findLapStartIndex(lapNum);
  const lapDist = calculateCumulativeDistance(closestIdx, lapStartIdx);
  $('#current-lap-dist').text(lapDist.toFixed(1) + 'm');

  // Update segment rank if available
  const segRank = getSegmentRanking(point);
  if (segRank === null) {
    $('#current-seg-rank').text('--');
    return;
  }
  const pct = Math.round(segRank * 100);
  $('#current-seg-rank').text(`${pct}%`);
}

/**
 * Update info panels
 */
function updateInfoPanels() {
  const metadata = appState.data.metadata;

  $('#info-laps').text(metadata.total_laps);
  $('#info-points').text(metadata.total_points);
  $('#info-duration').text(formatDuration(metadata.duration));
  $('#info-distance').text(metadata.total_distance.toFixed(1));
  $('#info-segments').text(metadata.num_segments);
  $('#info-mean-length').text(metadata.mean_course_length.toFixed(1));
}

/**
 * Start playback
 */
function startPlayback() {
  if (appState.isPlaying) return;
  
  appState.isPlaying = true;
  $('#play-btn').hide();
  $('#pause-btn').show();
  
  const slider = $('#time-slider');
  const minTime = parseFloat(slider.attr('min'));
  const maxTime = parseFloat(slider.attr('max'));
  
  // Playback at configured FPS
  appState.playInterval = setInterval(function() {
    let currentTime = parseFloat(slider.val());
    
    // Advance time using configured increment
    currentTime += PLAYBACK_TIME_INCREMENT;
    
    if (currentTime >= maxTime) {
      currentTime = minTime;  // Loop back
    }
    
    slider.val(currentTime);
    updateTimePosition(currentTime);
  }, PLAYBACK_INTERVAL_MS);
}

/**
 * Stop playback
 */
function stopPlayback() {
  if (!appState.isPlaying) return;
  
  appState.isPlaying = false;
  $('#play-btn').show();
  $('#pause-btn').hide();
  
  if (appState.playInterval) {
    clearInterval(appState.playInterval);
    appState.playInterval = null;
  }
}

/**
 * Create segment label annotations for the plot
 * @param {boolean} visible - Whether annotations should be visible
 * @returns {Array} Array of annotation objects
 */
function createSegmentAnnotations(visible) {
  if (!appState.data || !appState.data.segments) {
    return [];
  }
  return appState.data.segments.map((seg) => {
    const meanCourse = appState.data.mean_course;
    const midIdx = Math.floor((seg.start_idx + seg.end_idx) / 2);
    return {
      x: meanCourse[midIdx]?.x || 0,
      y: meanCourse[midIdx]?.y || 0,
      text: String(seg.id),
      showarrow: false,
      font: {
        family: 'Courier New, monospace',
        size: 11,
        color: 'black',
      },
      bgcolor: 'white',
      bordercolor: 'darkred',
      borderwidth: 2,
      borderpad: 4,
      opacity: 0.85,
      visible: visible,
    };
  });
}

/**
 * Toggle visibility of segment boundary lines
 * Trace layout: 0=driven path, 1=mean course, 2=current path,
 *               3=current marker, 4+=boundary lines
 * @param {boolean} visible - Whether boundaries should be visible
 */
function toggleBoundaryLines(visible) {
  if (!appState.data) return;
  const numBoundaries = (appState.data.segment_boundaries || []).length;
  for (let i = 0; i < numBoundaries; i++) {
    Plotly.restyle('plot-container', { visible: visible }, 4 + i);
  }
}

/**
 * Toggle path visibility
 */
function togglePathVisibility() {
  const visible = $('#show-path').is(':checked');
  appState.showDrivenPath = visible;
  if (appState.plotInitialized) {
    Plotly.restyle('plot-container', { visible: visible }, 0);
  }
}

/**
 * Toggle mean course visibility
 */
function toggleMeanCourseVisibility() {
  const visible = $('#show-mean-course').is(':checked');
  appState.showMeanCourse = visible;
  if (appState.plotInitialized && appState.data) {
    Plotly.restyle('plot-container', { visible: visible }, 1);
    toggleBoundaryLines(visible);
    Plotly.relayout('plot-container', {
      annotations: createSegmentAnnotations(visible)
    });
  }
}

/**
 * Apply saved visibility state after plot render
 * Syncs checkboxes and applies visibility to traces
 */
function applyVisibilityState() {
  $('#show-path').prop('checked', appState.showDrivenPath);
  $('#show-mean-course').prop('checked', appState.showMeanCourse);

  if (!appState.plotInitialized) return;

  Plotly.restyle('plot-container', { visible: appState.showDrivenPath }, 0);
  Plotly.restyle('plot-container', { visible: appState.showMeanCourse }, 1);
  toggleBoundaryLines(appState.showMeanCourse);
  Plotly.relayout('plot-container', {
    annotations: createSegmentAnnotations(appState.showMeanCourse)
  });
}
