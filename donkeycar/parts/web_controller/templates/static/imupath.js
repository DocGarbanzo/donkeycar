/**
 * Donkey IMU Path Visualizer - Web UI
 * Interactive visualization using Plotly.js
 */

// Configuration constants
const PLAYBACK_INTERVAL_MS = 50;  // 50ms interval for playback
const PLAYBACK_TIME_INCREMENT = PLAYBACK_INTERVAL_MS / 1000.0;  // Convert to seconds

// Global state
let appState = {
  data: null,
  currentTimeIndex: 0,
  isPlaying: false,
  playInterval: null,
  plotInitialized: false,
};

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
      renderPlot();
      updateInfoPanels();
      $('#loading').hide();
      $('#main-content').show();
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
    lapSelect.append(`<option value="${i}" ${i === metadata.num_laps ? 'selected' : ''}>
      ${i} lap${i > 1 ? 's' : ''}
    </option>`);
  }
  
  // Set segment method
  $('#segment-method').val(metadata.segment_method);
  
  // Setup time slider
  const times = data.path_points.map(p => p.t);
  const minTime = Math.min(...times);
  const maxTime = Math.max(...times);
  $('#time-slider').attr('min', minTime).attr('max', maxTime).val(minTime);
  $('#time-value').text(minTime.toFixed(1) + 's');
  
  // Setup stats field selector if available
  if (metadata.available_stats && metadata.available_stats.length > 0) {
    const statsSelect = $('#stats-field');
    statsSelect.empty();
    metadata.available_stats.forEach(stat => {
      statsSelect.append(`<option value="${stat}">${stat}</option>`);
    });
    $('#stats-control').show();
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
        title: 'Speed (m/s)',
        x: 1.02,
      },
    },
    hovertemplate: 'X: %{x:.2f}m<br>Y: %{y:.2f}m<br>Speed: %{marker.color:.2f}m/s<extra></extra>',
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
      width: 2,
    },
    hovertemplate: 'X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>',
    visible: true,
  });
  
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
  
  // Add segment boundary markers
  segments.forEach(seg => {
    const idx = seg.start_idx;
    if (idx < meanX.length) {
      traces.push({
        x: [meanX[idx]],
        y: [meanY[idx]],
        mode: 'markers+text',
        type: 'scatter',
        name: seg.label,
        text: [seg.label],
        textposition: 'top center',
        marker: {
          size: 8,
          color: '#808080',
          symbol: 'diamond',
        },
        showlegend: false,
        hovertemplate: `${seg.label}<br>Type: ${seg.type}<extra></extra>`,
      });
    }
  });
  
  // Layout
  const layout = {
    title: {
      text: 'IMU Path Visualization',
      font: { color: '#e0e0e0' },
    },
    xaxis: {
      title: 'X Position (m)',
      gridcolor: '#333',
      color: '#e0e0e0',
      scaleanchor: 'y',
      scaleratio: 1,
    },
    yaxis: {
      title: 'Y Position (m)',
      gridcolor: '#333',
      color: '#e0e0e0',
    },
    plot_bgcolor: '#000',
    paper_bgcolor: '#1a1a1a',
    hovermode: 'closest',
    showlegend: true,
    legend: {
      x: 0,
      y: 1,
      bgcolor: 'rgba(0,0,0,0.5)',
      font: { color: '#e0e0e0' },
    },
    margin: { l: 50, r: 50, t: 50, b: 50 },
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
  
  // Update marker position on plot
  if (appState.plotInitialized) {
    Plotly.restyle('plot-container', {
      x: [[point.x]],
      y: [[point.y]],
    }, 2);  // Index 2 is current position marker
  }
  
  // Update info display
  $('#time-value').text(point.t.toFixed(1) + 's');
  $('#current-time').text(point.t.toFixed(2));
  $('#current-lap').text(point.lap !== null ? point.lap + 1 : 'N/A');
  $('#current-segment').text(point.segment !== null ? point.segment : 'N/A');
  $('#current-speed').text(point.v.toFixed(2));
  $('#current-heading').text((point.h * 180 / Math.PI).toFixed(1));
  $('#current-x').text(point.x.toFixed(2));
  $('#current-y').text(point.y.toFixed(2));
}

/**
 * Update info panels
 */
function updateInfoPanels() {
  const metadata = appState.data.metadata;
  
  $('#info-laps').text(metadata.total_laps);
  $('#info-points').text(metadata.total_points);
  $('#info-duration').text(metadata.duration.toFixed(1));
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
 * Toggle path visibility
 */
function togglePathVisibility() {
  const visible = $('#show-path').is(':checked');
  if (appState.plotInitialized) {
    Plotly.restyle('plot-container', { visible: visible }, 0);
  }
}

/**
 * Toggle mean course visibility
 */
function toggleMeanCourseVisibility() {
  const visible = $('#show-mean-course').is(':checked');
  if (appState.plotInitialized && appState.data && appState.data.segments) {
    Plotly.restyle('plot-container', { visible: visible }, 1);
    // Also toggle segment markers (indices 3+)
    const numTraces = appState.data.segments.length;
    for (let i = 0; i < numTraces; i++) {
      Plotly.restyle('plot-container', { visible: visible }, 3 + i);
    }
  }
}
