"""
FastAPI Service for Web-Based IMU Path Visualization

This module provides a FastAPI-based web service for interactive IMU path
visualization. It serves a single-page Plotly application with REST endpoints
for data access.

Usage:
    donkey imupath --web <data_source>

The server starts on http://localhost:8000 by default.

Endpoints:
    GET  /               - Serve the visualization UI
    POST /api/load       - Load new data source
    GET  /api/data       - Get current visualization data
    GET  /api/stats      - Get segment statistics
    GET  /api/health     - Health check

The visualization UI provides:
    - Interactive Plotly scatter plot colored by speed (Viridis)
    - Mean course line overlay
    - Segment boundary markers and labels
    - Time slider for navigation
    - Lap count selector
    - Segment method selector
    - Stats field selector
    - Play/pause controls
    - Current point info panel
    - Best-per-lap statistics panel
"""

import logging
import os
from typing import Optional, Dict, Any
import uvicorn

try:
    from fastapi import FastAPI, HTTPException
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import HTMLResponse, FileResponse
    from pydantic import BaseModel
except ImportError:
    raise ImportError(
        "FastAPI is required for web UI. Install with: pip install fastapi uvicorn"
    )

from donkeycar.course_analysis import CSVPathDataSource, TubPathDataSource
from donkeycar.config import load_config
from donkeycar.web.imupath_data import IMUPathDataPreparation

logger = logging.getLogger(__name__)

# Global state (will be set by run_web_ui)
app_state = {
    'data_prep': None,
    'cfg': None,
    'data_source_path': None
}


class LoadDataRequest(BaseModel):
    """Request model for loading data."""
    data_source: str
    lap_method: str = 'y_crossing'
    segment_method: str = 'gradient'
    num_laps: Optional[int] = None


class UpdateSettingsRequest(BaseModel):
    """Request model for updating visualization settings."""
    lap_method: Optional[str] = None
    segment_method: Optional[str] = None
    num_laps: Optional[int] = None


# Create FastAPI app
app = FastAPI(
    title="IMU Path Visualizer",
    description="Interactive web-based IMU path visualization",
    version="1.0.0"
)


# Mount static files
static_dir = os.path.join(
    os.path.dirname(__file__), 'static', 'imupath'
)
if os.path.exists(static_dir):
    app.mount("/static", StaticFiles(directory=static_dir), name="static")


@app.get("/", response_class=HTMLResponse)
async def root():
    """Serve the main visualization page."""
    index_path = os.path.join(static_dir, 'index.html')
    if not os.path.exists(index_path):
        raise HTTPException(status_code=404, detail="UI not found")
    return FileResponse(index_path)


@app.get("/api/health")
async def health():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "data_loaded": app_state['data_prep'] is not None
    }


@app.get("/api/data")
async def get_data():
    """
    Get current visualization data.
    
    Returns complete data payload including path points, mean course,
    segments, rankings, and metadata.
    """
    if app_state['data_prep'] is None:
        raise HTTPException(status_code=400, detail="No data loaded")
    
    try:
        # Get max_display_points from config or use default
        max_display_points = 1000
        if app_state['cfg'] and hasattr(app_state['cfg'], 'IMU_VISUALIZATION_PARAMS'):
            max_display_points = app_state['cfg'].IMU_VISUALIZATION_PARAMS.get(
                'max_display_points', 1000
            )
        
        payload = app_state['data_prep'].get_data_payload(
            max_display_points=max_display_points
        )
        return payload
    except Exception as e:
        logger.error(f"Error generating data payload: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/stats")
async def get_stats():
    """
    Get segment statistics summary.
    
    Returns available ranking keys and best-per-lap segment statistics.
    """
    if app_state['data_prep'] is None:
        raise HTTPException(status_code=400, detail="No data loaded")
    
    try:
        # Extract best-per-lap statistics
        best_per_lap = {}
        
        if app_state['data_prep'].segment_rankings:
            num_laps = app_state['data_prep'].multilap_data.num_laps
            num_segments = app_state['data_prep'].segmentation.num_segments
            
            for lap in range(num_laps):
                for seg in range(num_segments):
                    key = f"lap_{lap}_seg_{seg}"
                    if key in app_state['data_prep'].segment_rankings:
                        metrics = app_state['data_prep'].segment_rankings[key]
                        
                        lap_key = f"lap_{lap}"
                        if lap_key not in best_per_lap:
                            best_per_lap[lap_key] = {}
                        
                        best_per_lap[lap_key][f"segment_{seg}"] = metrics
        
        return {
            'available_keys': app_state['data_prep'].available_ranking_keys,
            'best_per_lap': best_per_lap
        }
    except Exception as e:
        logger.error(f"Error generating stats: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/load")
async def load_data(request: LoadDataRequest):
    """
    Load a new data source.
    
    Args:
        request: LoadDataRequest with data source path and settings
        
    Returns:
        Success message with basic metadata
    """
    data_source = os.path.expanduser(request.data_source)
    
    if not os.path.exists(data_source):
        raise HTTPException(
            status_code=404,
            detail=f"Data source not found: {data_source}"
        )
    
    try:
        logger.info(f"Loading data from {data_source}...")
        
        # Determine data source type
        if os.path.isfile(data_source) and data_source.endswith('.csv'):
            source = CSVPathDataSource(data_source)
            tub_path = None
        elif os.path.isdir(data_source):
            source = TubPathDataSource(data_source)
            tub_path = data_source
        else:
            raise HTTPException(
                status_code=400,
                detail="Source must be CSV file or Tub directory"
            )
        
        # Load path data
        path_data = source.load()
        logger.info(f"Loaded {len(path_data.timestamp)} data points")
        
        # Prepare visualization data
        app_state['data_prep'] = IMUPathDataPreparation(
            path_data=path_data,
            cfg=app_state['cfg'],
            lap_method=request.lap_method,
            segment_method=request.segment_method,
            num_laps=request.num_laps,
            tub_path=tub_path
        )
        app_state['data_source_path'] = data_source
        
        return {
            "status": "success",
            "message": f"Loaded {len(path_data.timestamp)} data points",
            "metadata": {
                "num_laps": app_state['data_prep'].multilap_data.num_laps,
                "num_segments": app_state['data_prep'].segmentation.num_segments,
                "total_distance": float(path_data.total_distance),
                "duration": float(path_data.duration)
            }
        }
    except Exception as e:
        logger.error(f"Error loading data: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/update_settings")
async def update_settings(request: UpdateSettingsRequest):
    """
    Update visualization settings without reloading data.
    
    Args:
        request: UpdateSettingsRequest with new settings
        
    Returns:
        Success message with updated metadata
    """
    if app_state['data_prep'] is None:
        raise HTTPException(status_code=400, detail="No data loaded")
    
    if app_state['data_source_path'] is None:
        raise HTTPException(status_code=400, detail="No data source path")
    
    try:
        # Reload with new settings
        load_request = LoadDataRequest(
            data_source=app_state['data_source_path'],
            lap_method=request.lap_method or app_state['data_prep'].lap_method,
            segment_method=request.segment_method or app_state['data_prep'].segment_method,
            num_laps=request.num_laps if request.num_laps is not None else app_state['data_prep'].num_laps
        )
        return await load_data(load_request)
    except Exception as e:
        logger.error(f"Error updating settings: {e}")
        raise HTTPException(status_code=500, detail=str(e))


def run_web_ui(
    data_source: str,
    cfg: Optional[Any] = None,
    lap_method: str = 'y_crossing',
    segment_method: str = 'gradient',
    num_laps: Optional[int] = None,
    config_path: Optional[str] = None,
    host: str = "127.0.0.1",
    port: int = 8000
):
    """
    Launch the web-based IMU path visualizer.
    
    This is the main entry point for the web UI, called from the CLI.
    
    Args:
        data_source: Path to CSV file or Tub directory
        cfg: Configuration object (optional)
        lap_method: Lap detection method ('y_crossing' or 'drift')
        segment_method: Segmentation method
            ('threshold', 'extrema', 'gradient', 'hybrid')
        num_laps: Number of laps for mean course (None = all)
        config_path: Path to config file (for info display)
        host: Host to bind to (default: 127.0.0.1)
        port: Port to bind to (default: 8000)
    """
    data_source = os.path.expanduser(data_source)
    
    if not os.path.exists(data_source):
        print(f"Error: File or directory {data_source} not found.")
        return
    
    print("=" * 70)
    print("IMU Path Analysis - Web UI")
    print("=" * 70)
    print(f"Data source: {data_source}")
    print(f"Lap detection: {lap_method}")
    print(f"Segmentation: {segment_method}")
    if config_path:
        print(f"Config: {config_path}")
    print("=" * 70)
    
    # Load config
    if cfg is None and config_path:
        if os.path.exists(config_path):
            print("\nLoading config...")
            try:
                cfg = load_config(config_path)
                print(f"  Loaded config from {config_path}")
            except Exception as e:
                print(f"Warning: Failed to load config: {e}")
                print("  Using default parameters")
    
    # Store config in app state
    app_state['cfg'] = cfg
    
    # Load data
    print("\nLoading data...")
    try:
        if os.path.isfile(data_source) and data_source.endswith('.csv'):
            source = CSVPathDataSource(data_source)
            tub_path = None
        elif os.path.isdir(data_source):
            source = TubPathDataSource(data_source)
            tub_path = data_source
        else:
            print("Error: Source must be CSV file or Tub directory")
            return
        
        path_data = source.load()
        print(f"  {len(path_data.timestamp)} data points")
        print(f"  Total distance: {path_data.total_distance:.2f}m")
        print(f"  Duration: {path_data.duration:.2f}s")
    except Exception as e:
        print(f"Error loading data: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Prepare visualization data
    print("\nPreparing visualization data...")
    try:
        data_prep = IMUPathDataPreparation(
            path_data=path_data,
            cfg=cfg,
            lap_method=lap_method,
            segment_method=segment_method,
            num_laps=num_laps,
            tub_path=tub_path
        )
        app_state['data_prep'] = data_prep
        app_state['data_source_path'] = data_source
        
        print(f"  Detected {data_prep.multilap_data.num_laps} laps")
        print(f"  Segmented into {data_prep.segmentation.num_segments} segments")
        
        if tub_path and data_prep.segment_rankings:
            print(f"  ✓ Loaded segment rankings")
            print(f"  Available metrics: "
                  f"{', '.join(data_prep.available_ranking_keys)}")
        
    except Exception as e:
        print(f"Error preparing data: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Start server
    print("\n" + "=" * 70)
    print(f"Starting web server at http://{host}:{port}")
    print("Press Ctrl+C to stop")
    print("=" * 70)
    
    uvicorn.run(app, host=host, port=port, log_level="info")
