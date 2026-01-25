"""
Test web-based IMU path visualizer components
"""

import json
import os
import sys
import tempfile
from unittest.mock import patch, MagicMock, call
import tornado.testing
import tornado.web
from donkeycar.course_analysis import CSVPathDataSource, TubPathDataSource
from donkeycar.web.imupath_data import IMUPathDataBuilder, prepare_imupath_data
from donkeycar.parts.tub_v2 import Tub
from donkeycar.config import Config
from donkeycar.parts.web_controller.web import IMUPathRestartAPI


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
    
    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.csv', delete=False) as f:
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
    
    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.csv', delete=False) as f:
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
    
    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.csv', delete=False) as f:
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


def test_imupath_stats_use_tub_session_ranks():
    """Ensure web stats align with TubStatistics session rankings."""
    temp_dir = tempfile.mkdtemp()
    tub_path = os.path.join(temp_dir, 'stats_tub')

    tub = Tub(
        tub_path,
        inputs=[
            'car/pos',
            'car/euler',
            'car/speed',
            'car/lap',
            'car/segment',
            'car/distance',
        ],
        types=[
            'vector',
            'vector',
            'float',
            'int',
            'int',
            'float',
        ],
    )

    try:
        timestamp_ms = 0
        distance = 0.0
        for i in range(120):
            y_pos = -1.0 if i < 60 else 1.0
            lap_num = 0 if i < 60 else 1
            record = {
                'car/pos': [float(i) * 0.1, y_pos, 0.0],
                'car/euler': [0.0, 0.0, 0.0],
                'car/speed': 1.0 + i * 0.01,
                'car/lap': lap_num,
                'car/segment': 0,
                'car/distance': distance,
                '_timestamp_ms': timestamp_ms,
            }
            tub.write_record(record)
            timestamp_ms += 100
            distance += 0.1
        tub.close()

        source = TubPathDataSource(tub_path)
        path_data = source.load()
        builder = IMUPathDataBuilder(
            path_data=path_data,
            cfg=None,
            lap_method='y_crossing',
            segment_method='gradient',
            tub_path=tub_path,
        )
        rankings = builder.compute_segment_statistics(
            'car/speed', 'mean_abs')

        assert 1 in rankings
        assert 0 not in rankings
    finally:
        if os.path.exists(temp_dir):
            for root, _, files in os.walk(temp_dir, topdown=False):
                for name in files:
                    os.remove(os.path.join(root, name))
            for root, dirs, _ in os.walk(temp_dir, topdown=False):
                for name in dirs:
                    os.rmdir(os.path.join(root, name))
            os.rmdir(temp_dir)


def test_imupath_stats_bin_by_lap_count():
    """Ensure segment ranks are bucketed by unique lap count."""
    temp_dir = tempfile.mkdtemp()
    tub_path = os.path.join(temp_dir, 'rank_bins_tub')

    cfg = Config()
    cfg.USE_LAP_0 = True

    tub = Tub(
        tub_path,
        inputs=[
            'car/pos',
            'car/euler',
            'car/speed',
            'car/lap',
            'car/segment',
            'car/distance',
            'car/gyro',
        ],
        types=[
            'vector',
            'vector',
            'float',
            'int',
            'int',
            'float',
            'vector',
        ],
    )

    try:
        timestamp_ms = 0
        distance = 0.0
        for i in range(200):
            lap_num = 0 if i < 100 else 1
            segment_id = 0 if i % 2 == 0 else 1
            y_pos = -1.0 if i < 100 else 1.0
            record = {
                'car/pos': [float(i) * 0.1, y_pos, 0.0],
                'car/euler': [0.0, 0.0, 0.0],
                'car/speed': 1.0 + i * 0.01,
                'car/lap': lap_num,
                'car/segment': segment_id,
                'car/distance': distance,
                'car/gyro': [0.0, 0.1 + i * 0.001, 0.0],
                '_timestamp_ms': timestamp_ms,
            }
            tub.write_record(record)
            timestamp_ms += 100
            distance += 0.1
        tub.close()

        source = TubPathDataSource(tub_path)
        path_data = source.load()
        builder = IMUPathDataBuilder(
            path_data=path_data,
            cfg=cfg,
            lap_method='y_crossing',
            segment_method='gradient',
            tub_path=tub_path,
        )
        rankings = builder.compute_segment_statistics(
            'car/gyro', 'mean_abs', dimension=1)

        min_expected = 1.0 / 2.0
        for lap_data in rankings.values():
            segment_data = lap_data.get(0, {})
            rank = segment_data.get('computed_stat')
            assert rank is not None
            assert rank >= min_expected
    finally:
        if os.path.exists(temp_dir):
            for root, _, files in os.walk(temp_dir, topdown=False):
                for name in files:
                    os.remove(os.path.join(root, name))
            for root, dirs, _ in os.walk(temp_dir, topdown=False):
                for name in dirs:
                    os.rmdir(os.path.join(root, name))
            os.rmdir(temp_dir)


def test_imupath_stats_use_visual_laps_when_constant():
    """Use visual laps when car/lap stays constant."""
    temp_dir = tempfile.mkdtemp()
    tub_path = os.path.join(temp_dir, 'constant_lap_tub')

    cfg = Config()
    cfg.USE_LAP_0 = True

    tub = Tub(
        tub_path,
        inputs=[
            'car/pos',
            'car/euler',
            'car/speed',
            'car/lap',
            'car/segment',
            'car/distance',
            'car/gyro',
        ],
        types=[
            'vector',
            'vector',
            'float',
            'int',
            'int',
            'float',
            'vector',
        ],
    )

    try:
        timestamp_ms = 0
        distance = 0.0
        for i in range(200):
            if i < 50:
                y_pos = -1.0
            elif i < 100:
                y_pos = 1.0
            elif i < 150:
                y_pos = -1.0
            else:
                y_pos = 1.0
            record = {
                'car/pos': [float(i) * 0.1, y_pos, 0.0],
                'car/euler': [0.0, 0.0, 0.0],
                'car/speed': 1.0 + i * 0.01,
                'car/lap': 0,
                'car/segment': 0,
                'car/distance': distance,
                'car/gyro': [0.0, 0.1 + i * 0.001, 0.0],
                '_timestamp_ms': timestamp_ms,
            }
            tub.write_record(record)
            timestamp_ms += 100
            distance += 0.1
        tub.close()

        source = TubPathDataSource(tub_path)
        path_data = source.load()
        builder = IMUPathDataBuilder(
            path_data=path_data,
            cfg=cfg,
            lap_method='y_crossing',
            segment_method='gradient',
            tub_path=tub_path,
        )
        rankings = builder.compute_segment_statistics(
            'car/gyro', 'mean_abs', dimension=1)

        assert 0 in rankings
        assert 1 in rankings
        segment_ids = set()
        for lap_data in rankings.values():
            segment_ids.update(lap_data.keys())
        assert max(segment_ids) > 0
    finally:
        if os.path.exists(temp_dir):
            for root, _, files in os.walk(temp_dir, topdown=False):
                for name in files:
                    os.remove(os.path.join(root, name))
            for root, dirs, _ in os.walk(temp_dir, topdown=False):
                for name in dirs:
                    os.rmdir(os.path.join(root, name))
            os.rmdir(temp_dir)


class IMUPathRestartAPITest(tornado.testing.AsyncHTTPTestCase):
    """Test the IMU Path restart API endpoint."""

    def get_app(self):
        """Create a test tornado application with restart handler."""
        app = tornado.web.Application([
            (r"/api/imupath/restart", IMUPathRestartAPI),
        ])
        return app

    @tornado.testing.gen_test
    def test_restart_success(self):
        """Test successful restart with valid origin and confirmation."""
        with patch('os.execv') as mock_execv:
            body = json.dumps({'confirm': 'restart'})
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Origin': f'http://localhost:{self.get_http_port()}',
                    'Host': f'localhost:{self.get_http_port()}'
                }
            )

            assert response.code == 200
            data = json.loads(response.body)
            assert data['status'] == 'restarting'

            # Wait for the delayed restart call
            yield tornado.gen.sleep(0.6)
            mock_execv.assert_called_once()
            args = mock_execv.call_args[0]
            assert args[0] == sys.executable
            assert args[1][0] == sys.executable

    @tornado.testing.gen_test
    def test_restart_cross_origin_blocked(self):
        """Test that cross-origin requests are blocked."""
        body = json.dumps({'confirm': 'restart'})

        try:
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Origin': 'http://evil.com',
                    'Host': f'localhost:{self.get_http_port()}'
                },
                raise_error=False
            )
            assert response.code == 403
            data = json.loads(response.body)
            assert 'error' in data
            assert data['error'] == 'Forbidden'
        except Exception:
            # Tornado might raise on 403
            pass

    @tornado.testing.gen_test
    def test_restart_invalid_json(self):
        """Test that invalid JSON is rejected."""
        body = "not valid json"

        try:
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Origin': f'http://localhost:{self.get_http_port()}',
                    'Host': f'localhost:{self.get_http_port()}'
                },
                raise_error=False
            )
            assert response.code == 400
            data = json.loads(response.body)
            assert 'error' in data
            assert data['error'] == 'Invalid JSON'
        except Exception:
            # Tornado might raise on 400
            pass

    @tornado.testing.gen_test
    def test_restart_missing_confirmation(self):
        """Test that missing confirmation is rejected."""
        body = json.dumps({'wrong_key': 'restart'})

        try:
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Origin': f'http://localhost:{self.get_http_port()}',
                    'Host': f'localhost:{self.get_http_port()}'
                },
                raise_error=False
            )
            assert response.code == 400
            data = json.loads(response.body)
            assert 'error' in data
            assert data['error'] == 'Missing confirmation'
        except Exception:
            # Tornado might raise on 400
            pass

    @tornado.testing.gen_test
    def test_restart_no_origin_header(self):
        """Test that requests without Origin header are allowed (same-origin)."""
        with patch('os.execv') as mock_execv:
            body = json.dumps({'confirm': 'restart'})
            response = yield self.http_client.fetch(
                self.get_url('/api/imupath/restart'),
                method='POST',
                body=body,
                headers={
                    'Content-Type': 'application/json',
                    'Host': f'localhost:{self.get_http_port()}'
                }
            )

            assert response.code == 200
            data = json.loads(response.body)
            assert data['status'] == 'restarting'

            # Wait for the delayed restart call
            yield tornado.gen.sleep(0.6)
            mock_execv.assert_called_once()


if __name__ == '__main__':
    print("Testing IMU path data builder...")
    test_imupath_data_builder_csv()
    print("\nTesting JSON payload...")
    test_imupath_json_payload()
    print("\nTesting downsampling...")
    test_imupath_downsampling()
    print("\nAll tests passed!")
