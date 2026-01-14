// IMU Path Visualizer - JavaScript Application

// Global state
const state = {
    data: null,
    currentIndex: 0,
    playing: false,
    playInterval: null,
    plotDiv: null,
    showPath: true,
    showMean: true,
    showSegments: true
};

// Initialize application
document.addEventListener('DOMContentLoaded', async () => {
    console.log('Initializing IMU Path Visualizer...');
    
    // Get DOM elements
    state.plotDiv = document.getElementById('plot-container');
    
    // Set up event listeners
    setupEventListeners();
    
    // Load initial data
    await loadData();
    
    // Initialize plot
    updatePlot();
    
    updateStatus('Ready');
});

// Set up event listeners
function setupEventListeners() {
    // Time control
    document.getElementById('time-slider').addEventListener('input', (e) => {
        state.currentIndex = parseInt(e.target.value);
        updateCurrentPoint();
    });
    
    document.getElementById('play-btn').addEventListener('click', startPlayback);
    document.getElementById('pause-btn').addEventListener('click', pausePlayback);
    
    // Settings
    document.getElementById('lap-minus').addEventListener('click', () => {
        const input = document.getElementById('lap-count');
        const val = parseInt(input.value);
        if (val > 1) input.value = val - 1;
    });
    
    document.getElementById('lap-plus').addEventListener('click', () => {
        const input = document.getElementById('lap-count');
        const maxLaps = state.data ? state.data.metadata.num_laps : 10;
        const val = parseInt(input.value);
        if (val < maxLaps) input.value = val + 1;
    });
    
    document.getElementById('update-btn').addEventListener('click', updateVisualization);
    
    // Display toggles
    document.getElementById('show-path').addEventListener('change', (e) => {
        state.showPath = e.target.checked;
        updatePlot();
    });
    
    document.getElementById('show-mean').addEventListener('change', (e) => {
        state.showMean = e.target.checked;
        updatePlot();
    });
    
    document.getElementById('show-segments').addEventListener('change', (e) => {
        state.showSegments = e.target.checked;
        updatePlot();
    });
    
    // Keyboard navigation
    document.addEventListener('keydown', (e) => {
        if (e.key === 'ArrowLeft') {
            if (state.currentIndex > 0) {
                state.currentIndex--;
                document.getElementById('time-slider').value = state.currentIndex;
                updateCurrentPoint();
            }
        } else if (e.key === 'ArrowRight') {
            const maxIndex = state.data ? state.data.path_points.length - 1 : 0;
            if (state.currentIndex < maxIndex) {
                state.currentIndex++;
                document.getElementById('time-slider').value = state.currentIndex;
                updateCurrentPoint();
            }
        } else if (e.key === ' ') {
            e.preventDefault();
            if (state.playing) {
                pausePlayback();
            } else {
                startPlayback();
            }
        }
    });
}

// Load data from API
async function loadData() {
    try {
        updateStatus('Loading data...');
        
        const response = await fetch('/api/data');
        if (!response.ok) {
            throw new Error(`Failed to load data: ${response.statusText}`);
        }
        
        state.data = await response.json();
        console.log('Data loaded:', state.data);
        
        // Update metadata display
        updateMetadataDisplay();
        
        // Update controls
        const lapInput = document.getElementById('lap-count');
        lapInput.max = state.data.metadata.num_laps;
        lapInput.value = state.data.metadata.num_laps;
        
        const segmentSelect = document.getElementById('segment-method');
        segmentSelect.value = state.data.metadata.segment_method;
        
        // Update time slider
        const slider = document.getElementById('time-slider');
        slider.max = state.data.path_points.length - 1;
        slider.value = 0;
        state.currentIndex = 0;
        
        // Update stats field options
        updateStatsFieldOptions();
        
        // Load stats
        await loadStats();
        
        updateStatus('Data loaded successfully');
    } catch (error) {
        console.error('Error loading data:', error);
        updateStatus(`Error: ${error.message}`, true);
    }
}

// Update metadata display
function updateMetadataDisplay() {
    const meta = state.data.metadata;
    const html = `
        <span><strong>Laps:</strong> ${meta.num_laps}</span>
        <span><strong>Segments:</strong> ${meta.num_segments}</span>
        <span><strong>Distance:</strong> ${meta.total_distance.toFixed(2)}m</span>
        <span><strong>Duration:</strong> ${meta.duration.toFixed(2)}s</span>
        <span><strong>Points:</strong> ${state.data.path_points.length}</span>
    `;
    document.getElementById('metadata').innerHTML = html;
}

// Update stats field options
function updateStatsFieldOptions() {
    const select = document.getElementById('stats-field');
    select.innerHTML = '';
    
    if (state.data.metadata.available_ranking_keys.length > 0) {
        state.data.metadata.available_ranking_keys.forEach(key => {
            const option = document.createElement('option');
            option.value = key;
            option.textContent = key.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase());
            select.appendChild(option);
        });
    } else {
        const option = document.createElement('option');
        option.value = 'time_pct';
        option.textContent = 'Time Percentile';
        select.appendChild(option);
    }
}

// Load stats from API
async function loadStats() {
    try {
        const response = await fetch('/api/stats');
        if (!response.ok) {
            console.warn('Stats not available');
            return;
        }
        
        const stats = await response.json();
        console.log('Stats loaded:', stats);
        
        displayStats(stats);
    } catch (error) {
        console.error('Error loading stats:', error);
    }
}

// Display stats in panel
function displayStats(stats) {
    const container = document.getElementById('stats-display');
    
    if (!stats.best_per_lap || Object.keys(stats.best_per_lap).length === 0) {
        container.innerHTML = '<p class="info-note">No statistics available</p>';
        return;
    }
    
    const selectedField = document.getElementById('stats-field').value;
    
    let html = '<div class="stats-table">';
    
    for (const [lapKey, segments] of Object.entries(stats.best_per_lap)) {
        const lapNum = lapKey.split('_')[1];
        html += `<h4>Lap ${lapNum}</h4>`;
        html += '<table><tr><th>Seg</th><th>Value</th></tr>';
        
        for (const [segKey, metrics] of Object.entries(segments)) {
            const segNum = segKey.split('_')[1];
            const value = metrics[selectedField];
            if (value !== undefined) {
                html += `<tr><td>S${segNum}</td><td>${(value * 100).toFixed(1)}%</td></tr>`;
            }
        }
        
        html += '</table>';
    }
    
    html += '</div>';
    container.innerHTML = html;
}

// Update plot
function updatePlot() {
    if (!state.data) return;
    
    const traces = [];
    const annotations = [];
    
    // Path scatter colored by speed (Viridis)
    if (state.showPath) {
        const pathTrace = {
            x: state.data.path_points.map(p => p.x),
            y: state.data.path_points.map(p => p.y),
            mode: 'markers',
            type: 'scatter',
            marker: {
                size: 4,
                color: state.data.path_points.map(p => p.v),
                colorscale: 'Viridis',
                showscale: true,
                colorbar: {
                    title: 'Speed (m/s)',
                    x: 1.15
                }
            },
            name: 'Driven Path',
            hovertemplate: 
                'Position: (%{x:.2f}, %{y:.2f})<br>' +
                'Speed: %{marker.color:.2f} m/s<br>' +
                '<extra></extra>'
        };
        traces.push(pathTrace);
    }
    
    // Mean course line
    if (state.showMean && state.data.mean_course.length > 0) {
        const meanTrace = {
            x: state.data.mean_course.map(p => p.x),
            y: state.data.mean_course.map(p => p.y),
            mode: 'lines',
            type: 'scatter',
            line: {
                color: '#808080',
                width: 2
            },
            name: 'Mean Course',
            hoverinfo: 'skip'
        };
        traces.push(meanTrace);
    }
    
    // Segment boundaries and labels
    if (state.showSegments && state.data.segments.length > 0) {
        const segmentX = state.data.segments.map(s => s.x);
        const segmentY = state.data.segments.map(s => s.y);
        
        const segmentTrace = {
            x: segmentX,
            y: segmentY,
            mode: 'markers',
            type: 'scatter',
            marker: {
                size: 10,
                color: '#808080',
                symbol: 'diamond',
                line: {
                    color: 'white',
                    width: 1
                }
            },
            name: 'Segments',
            hovertemplate: '%{text}<extra></extra>',
            text: state.data.segments.map(s => s.label)
        };
        traces.push(segmentTrace);
        
        // Add labels as annotations
        state.data.segments.forEach(seg => {
            annotations.push({
                x: seg.x,
                y: seg.y,
                text: seg.label,
                showarrow: false,
                font: {
                    size: 10,
                    color: '#333'
                },
                bgcolor: 'rgba(255, 255, 255, 0.8)',
                borderpad: 2,
                xshift: 10,
                yshift: 10
            });
        });
    }
    
    // Current position marker
    const currentPoint = state.data.path_points[state.currentIndex];
    if (currentPoint) {
        const currentTrace = {
            x: [currentPoint.x],
            y: [currentPoint.y],
            mode: 'markers',
            type: 'scatter',
            marker: {
                size: 15,
                color: 'red',
                symbol: 'circle',
                line: {
                    color: 'white',
                    width: 2
                }
            },
            name: 'Current Position',
            hoverinfo: 'skip'
        };
        traces.push(currentTrace);
    }
    
    // Layout
    const layout = {
        title: 'IMU Path Visualization',
        xaxis: {
            title: 'X Position (m)',
            scaleanchor: 'y',
            scaleratio: 1
        },
        yaxis: {
            title: 'Y Position (m)'
        },
        hovermode: 'closest',
        showlegend: true,
        legend: {
            x: 1.2,
            y: 0.5
        },
        annotations: annotations,
        margin: {
            l: 50,
            r: 200,
            t: 50,
            b: 50
        }
    };
    
    const config = {
        responsive: true,
        displayModeBar: true,
        displaylogo: false,
        modeBarButtonsToRemove: ['lasso2d', 'select2d']
    };
    
    Plotly.newPlot(state.plotDiv, traces, layout, config);
    
    // Update current point info
    updateCurrentPoint();
}

// Update current point info
function updateCurrentPoint() {
    if (!state.data || state.currentIndex < 0 || state.currentIndex >= state.data.path_points.length) {
        return;
    }
    
    const point = state.data.path_points[state.currentIndex];
    
    document.getElementById('info-time').textContent = `${point.t.toFixed(2)}s`;
    document.getElementById('info-lap').textContent = point.lap;
    document.getElementById('info-segment').textContent = point.segment;
    document.getElementById('info-velocity').textContent = `${point.v.toFixed(2)} m/s`;
    document.getElementById('info-heading').textContent = `${(point.h * 180 / Math.PI).toFixed(1)}°`;
    document.getElementById('info-position').textContent = `(${point.x.toFixed(2)}, ${point.y.toFixed(2)})`;
    
    // Update time display
    const maxIndex = state.data.path_points.length - 1;
    document.getElementById('time-display').textContent = `${state.currentIndex} / ${maxIndex}`;
    
    // Update current position on plot
    if (state.plotDiv && state.plotDiv.data) {
        const currentTraceIndex = state.plotDiv.data.length - 1;
        Plotly.restyle(state.plotDiv, {
            x: [[point.x]],
            y: [[point.y]]
        }, [currentTraceIndex]);
    }
}

// Start playback
function startPlayback() {
    if (state.playing) return;
    
    state.playing = true;
    document.getElementById('play-btn').disabled = true;
    document.getElementById('pause-btn').disabled = false;
    
    const slider = document.getElementById('time-slider');
    const maxIndex = state.data.path_points.length - 1;
    
    state.playInterval = setInterval(() => {
        if (state.currentIndex >= maxIndex) {
            pausePlayback();
            return;
        }
        
        state.currentIndex++;
        slider.value = state.currentIndex;
        updateCurrentPoint();
    }, 50); // ~20 FPS
}

// Pause playback
function pausePlayback() {
    if (!state.playing) return;
    
    state.playing = false;
    document.getElementById('play-btn').disabled = false;
    document.getElementById('pause-btn').disabled = true;
    
    if (state.playInterval) {
        clearInterval(state.playInterval);
        state.playInterval = null;
    }
}

// Update visualization with new settings
async function updateVisualization() {
    try {
        updateStatus('Updating visualization...');
        pausePlayback();
        
        const lapCount = parseInt(document.getElementById('lap-count').value);
        const segmentMethod = document.getElementById('segment-method').value;
        
        const response = await fetch('/api/update_settings', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                num_laps: lapCount,
                segment_method: segmentMethod
            })
        });
        
        if (!response.ok) {
            throw new Error(`Failed to update settings: ${response.statusText}`);
        }
        
        // Reload data
        await loadData();
        updatePlot();
        
        updateStatus('Visualization updated');
    } catch (error) {
        console.error('Error updating visualization:', error);
        updateStatus(`Error: ${error.message}`, true);
    }
}

// Update status bar
function updateStatus(message, isError = false) {
    const statusText = document.getElementById('status-text');
    statusText.textContent = message;
    statusText.className = isError ? 'error' : '';
    
    // Clear status after 5 seconds if not error
    if (!isError) {
        setTimeout(() => {
            if (statusText.textContent === message) {
                statusText.textContent = 'Ready';
            }
        }, 5000);
    }
}
