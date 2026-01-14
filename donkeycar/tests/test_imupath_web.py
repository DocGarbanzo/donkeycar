"""
Test web-based IMU path visualizer components
"""

import json
import os
import tempfile
from donkeycar.course_analysis import CSVPathDataSource
from donkeycar.web.imupath_data import IMUPathDataBuilder, prepare_imupath_data


def test_imupath_data_builder_csv():
    """Test IMUPathDataBuilder with CSV data."""
    # Create a simple CSV file
    csv_data = """t,x,y,h,v
0.0,0.0,0.0,0.0,0.5
0.1,0.1,0.0,0.0,0.5
0.2,0.2,0.1,0.1,0.6
0.3,0.3,0.2,0.2,0.6
0.4,0.4,0.3,0.3,0.7
0.5,0.4,0.4,0.5,0.7
0.6,0.3,0.5,0.7,0.6
0.7,0.2,0.6,0.9,0.6
0.8,0.1,0.6,1.2,0.5
0.9,0.0,0.5,1.5,0.5
1.0,0.0,0.4,1.57,0.5
1.1,0.0,0.3,1.57,0.5
1.2,0.0,0.2,1.57,0.5
1.3,0.0,0.1,1.57,0.5
"""
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
        f.write(csv_data)
        csv_path = f.name
    
    try:
        # Load data
        source = CSVPathDataSource(csv_path)
        path_data = source.load()
        
        # Create builder
        builder = IMUPathDataBuilder(
            path_data=path_data,
            cfg=None,
            lap_method='y_crossing',
            segment_method='gradient',
            tub_path=None
        )
        
        # Verify initialization
        assert builder.multilap_data is not None
        assert builder.mean_course is not None
        assert builder.segmentation is not None
        assert builder.segment_ids is not None
        
        print(f"Detected {builder.multilap_data.num_laps} laps")
        print(f"Mean course length: {builder.mean_course.length:.1f}m")
        print(f"Segments: {builder.segmentation.num_segments}")
        
    finally:
        os.unlink(csv_path)


def test_imupath_json_payload():
    """Test JSON payload generation."""
    # Create a simple CSV file
    csv_data = """t,x,y,h,v
0.0,0.0,0.0,0.0,0.5
0.1,0.1,0.0,0.0,0.5
0.2,0.2,0.1,0.1,0.6
0.3,0.3,0.2,0.2,0.6
0.4,0.4,0.3,0.3,0.7
"""
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
        f.write(csv_data)
        csv_path = f.name
    
    try:
        # Load data
        source = CSVPathDataSource(csv_path)
        path_data = source.load()
        
        # Build JSON payload
        data = prepare_imupath_data(
            path_data=path_data,
            cfg=None,
            lap_method='y_crossing',
            segment_method='gradient',
            tub_path=None,
            num_laps=None,
            max_display_points=100
        )
        
        # Verify structure
        assert 'path_points' in data
        assert 'mean_course' in data
        assert 'segments' in data
        assert 'metadata' in data
        assert 'rankings' in data
        
        # Verify path points
        assert len(data['path_points']) > 0
        point = data['path_points'][0]
        assert 't' in point
        assert 'x' in point
        assert 'y' in point
        assert 'v' in point
        assert 'h' in point
        assert 'lap' in point
        
        # Verify mean course
        assert len(data['mean_course']) > 0
        mean_point = data['mean_course'][0]
        assert 'x' in mean_point
        assert 'y' in mean_point
        
        # Verify metadata
        metadata = data['metadata']
        assert metadata['lap_method'] == 'y_crossing'
        assert metadata['segment_method'] == 'gradient'
        assert metadata['total_points'] == 5
        assert metadata['is_tub_data'] is False
        
        # Verify JSON serializable
        json_str = json.dumps(data)
        assert len(json_str) > 0
        
        print(f"JSON payload size: {len(json_str)} bytes")
        print(f"Path points: {len(data['path_points'])}")
        print(f"Mean course points: {len(data['mean_course'])}")
        print(f"Segments: {len(data['segments'])}")
        
    finally:
        os.unlink(csv_path)


def test_imupath_downsampling():
    """Test downsampling functionality."""
    # Create data with many points
    csv_lines = ["t,x,y,h,v"]
    for i in range(200):
        t = i * 0.01
        x = i * 0.01
        y = 0.0
        h = 0.0
        v = 0.5
        csv_lines.append(f"{t},{x},{y},{h},{v}")
    
    csv_data = "\n".join(csv_lines)
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
        f.write(csv_data)
        csv_path = f.name
    
    try:
        # Load data
        source = CSVPathDataSource(csv_path)
        path_data = source.load()
        
        # Build JSON with downsampling
        data = prepare_imupath_data(
            path_data=path_data,
            cfg=None,
            max_display_points=50
        )
        
        # Verify downsampling
        assert len(data['path_points']) <= 50
        assert data['metadata']['total_points'] == 200
        assert data['metadata']['display_points'] == len(data['path_points'])
        
        print(f"Downsampled from {data['metadata']['total_points']} "
              f"to {data['metadata']['display_points']} points")
        
    finally:
        os.unlink(csv_path)


if __name__ == '__main__':
    print("Testing IMU path data builder...")
    test_imupath_data_builder_csv()
    print("\nTesting JSON payload...")
    test_imupath_json_payload()
    print("\nTesting downsampling...")
    test_imupath_downsampling()
    print("\nAll tests passed!")
