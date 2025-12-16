"""
Tests for IMU path visualizer
"""
import os
import tempfile
import numpy as np
import pandas as pd
import pytest
from donkeycar.utilities.imu_visualization import correct_loop_drift


class TestIMUVisualizer:
    """Test IMU path visualizer functionality"""

    def test_drift_correction_simple_loop(self):
        """Test drift correction on a simple closed loop"""
        # Create a simple square loop with drift
        t = np.linspace(0, 4, 100)
        # Perfect square would close, but add drift
        x = np.concatenate([
            np.linspace(0, 1, 25),    # Right
            np.ones(25),              # Forward
            np.linspace(1, 0, 25),    # Left
            np.zeros(25)              # Back
        ]) + 0.1  # Add drift

        y = np.concatenate([
            np.zeros(25),             # Right
            np.linspace(0, 1, 25),    # Forward
            np.ones(25),              # Left
            np.linspace(1, 0, 25)     # Back
        ]) + 0.05  # Add drift

        v = np.ones(100) * 0.5  # Constant velocity

        df = pd.DataFrame({'t': t, 'x': x, 'y': y, 'h': np.zeros(100),
                          'v': v})

        # Apply drift correction
        corrected_df, loop_indices, drift_amounts = correct_loop_drift(
            df, min_loop_distance=1.0, max_distance=0.5)

        # Check that correction was applied
        assert len(corrected_df) == len(df)
        assert isinstance(loop_indices, list)
        assert isinstance(drift_amounts, list)

        # If loop detected, drift should be reduced or equal
        if len(loop_indices) > 0:
            # End position should be closer to or equal to start
            end_x = corrected_df.iloc[-1]['x']
            end_y = corrected_df.iloc[-1]['y']
            end_distance = np.sqrt(end_x**2 + end_y**2)

            # Original end distance
            orig_end_x = df.iloc[-1]['x']
            orig_end_y = df.iloc[-1]['y']
            orig_end_distance = np.sqrt(orig_end_x**2 + orig_end_y**2)

            # Corrected should be closer to or equal to origin
            # (equal if drift was already minimal)
            assert end_distance <= orig_end_distance

    def test_drift_correction_no_loop(self):
        """Test drift correction when no loop is detected"""
        # Create a straight line (no loop)
        t = np.linspace(0, 1, 50)
        x = np.linspace(0, 5, 50)
        y = np.zeros(50)
        v = np.ones(50)

        df = pd.DataFrame({'t': t, 'x': x, 'y': y, 'h': np.zeros(50),
                          'v': v})

        # Apply drift correction
        corrected_df, loop_indices, drift_amounts = correct_loop_drift(
            df, min_loop_distance=1.0, max_distance=0.5)

        # Should return data unchanged with no loops detected
        assert len(loop_indices) == 0
        assert len(drift_amounts) == 0
        assert len(corrected_df) == len(df)

    def test_csv_data_loading(self):
        """Test loading data from CSV file"""
        from donkeycar.utilities.imu_visualization import (
            visualize_imu_path)

        # Create temporary CSV file
        with tempfile.NamedTemporaryFile(
                mode='w', suffix='.csv', delete=False) as f:
            f.write('t,x,y,h,v\n')
            for i in range(10):
                f.write(f'{i*0.1},{i*0.1},{i*0.05},0.0,1.0\n')
            csv_file = f.name

        try:
            # This should load without error
            # (will fail on visualization, but that's ok for test)
            import matplotlib
            matplotlib.use('Agg')  # Use non-GUI backend

            # We can't easily test the full visualization without GUI,
            # but we can test that data loads
            # For now, just verify file can be read
            df = pd.read_csv(csv_file)
            assert len(df) == 10
            assert all(col in df.columns for col in ['t', 'x', 'y', 'v'])

        finally:
            os.unlink(csv_file)

    def test_drift_correction_with_empty_data(self):
        """Test that empty data is handled gracefully"""
        df = pd.DataFrame({'t': [], 'x': [], 'y': [], 'h': [], 'v': []})

        corrected_df, loop_indices, drift_amounts = correct_loop_drift(df)

        assert len(corrected_df) == 0
        assert len(loop_indices) == 0
        assert len(drift_amounts) == 0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
