#!/usr/bin/env python3
"""
Manual test script for web-based IMU path visualizer.

This script creates a simple test CSV file and demonstrates how to use the
web-based IMU path visualizer.

Usage:
    python test_imupath_web_manual.py
    
Then open your browser to http://localhost:8887/imupath
"""

import tempfile


def create_test_csv():
    """Create a test CSV file with a simple circular path."""
    import math
    
    csv_lines = ["t,x,y,h,v"]
    
    # Create a circular path with 3 laps
    num_points_per_lap = 100
    radius = 5.0
    
    for lap in range(3):
        for i in range(num_points_per_lap):
            t = (lap * num_points_per_lap + i) * 0.05
            angle = (i / num_points_per_lap) * 2 * math.pi
            
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            h = angle + math.pi / 2  # Heading tangent to circle
            v = 2.0 + 0.5 * math.sin(angle * 2)  # Variable speed
            
            csv_lines.append(f"{t:.2f},{x:.3f},{y:.3f},{h:.3f},{v:.2f}")
    
    csv_data = "\n".join(csv_lines)
    
    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
        f.write(csv_data)
        csv_path = f.name
    
    return csv_path


def main():
    """Run the web-based IMU path visualizer."""
    print("=" * 70)
    print("Web-Based IMU Path Visualizer - Manual Test")
    print("=" * 70)
    
    # Create test data
    print("\nCreating test CSV file...")
    csv_path = create_test_csv()
    print(f"  Test file: {csv_path}")
    
    # Show instructions
    print("\nTo test the web visualizer:")
    print(f"  1. Run: donkey imupath --web {csv_path}")
    print("  2. Open browser to: http://localhost:8887/imupath")
    print("  3. Verify the following features:")
    print("     - Path scatter plot with speed coloring")
    print("     - Mean course polyline (grey)")
    print("     - Segment boundary markers")
    print("     - Time slider control")
    print("     - Lap selector dropdown")
    print("     - Segment method dropdown")
    print("     - Play/pause button")
    print("     - Current position info panel")
    print("     - Dataset info panel")
    print("\n" + "=" * 70)
    print("Test Setup Complete!")
    print("=" * 70)
    print(f"\nTest file will remain at: {csv_path}")
    print("You can delete it manually when done testing.")
    

if __name__ == '__main__':
    main()
