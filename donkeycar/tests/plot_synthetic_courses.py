"""Script to plot synthetic courses for visual inspection."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from donkeycar.tests.synthetic_courses import (
    LShapeCourse, UShapeCourse, simulate_driving
)


def plot_course(x, y, meta, title, filename):
    """Plot a single course with segment colors."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    colors = plt.cm.tab10(np.linspace(0, 1, len(meta)))
    for i, seg in enumerate(meta):
        s, e = seg['start_index'], seg['end_index'] + 1
        label = f"S{seg['segment_id']}: {seg['type']}"
        ax.plot(x[s:e], y[s:e], color=colors[i], linewidth=2.5,
                label=label)
    ax.plot(x[0], y[0], 'go', markersize=10, label='Start')
    ax.plot(x[-1], y[-1], 'rs', markersize=10, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.legend(fontsize=8, loc='best')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(filename, dpi=120)
    plt.close(fig)
    print(f"Saved: {filename}")


def plot_driving(course_x, course_y, path_data, course_name, filename):
    """Plot course centerline with simulated driving laps."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.plot(course_x, course_y, 'k-', linewidth=3, alpha=0.3,
            label='Course centerline')
    ax.plot(path_data.x, path_data.y, '.', markersize=0.5, alpha=0.5,
            label='Simulated driving')
    ax.plot(path_data.x[0], path_data.y[0], 'go', markersize=10,
            label='Start')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'{course_name} - Simulated Driving (5 laps)')
    ax.legend(fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(filename, dpi=120)
    plt.close(fig)
    print(f"Saved: {filename}")


if __name__ == '__main__':
    out_dir = '/tmp/synthetic_courses'
    os.makedirs(out_dir, exist_ok=True)

    # L-shape course
    l_course = LShapeCourse(h_len=10.0, v_len=15.0, d=1.0)
    lx, ly, lh, ld, lmeta = l_course.generate()
    print(f"L-shape: {len(lx)} points, "
          f"total length = {ld[-1]:.2f}m "
          f"(expected {l_course.expected_total_length():.2f}m)")
    plot_course(lx, ly, lmeta, 'L-Shape Course',
                os.path.join(out_dir, 'l_shape_course.png'))

    # L-shape simulated driving
    l_path = simulate_driving(lx, ly, lh, ld, num_laps=5, seed=42)
    print(f"L-shape driving: {len(l_path)} points, "
          f"{l_path.total_distance:.1f}m total")
    plot_driving(lx, ly, l_path, 'L-Shape',
                 os.path.join(out_dir, 'l_shape_driving.png'))

    # U-shape course
    u_course = UShapeCourse(width=6.0, v_len=12.0, d=1.0)
    ux, uy, uh, ud, umeta = u_course.generate()
    print(f"U-shape: {len(ux)} points, "
          f"total length = {ud[-1]:.2f}m "
          f"(expected {u_course.expected_total_length():.2f}m)")
    plot_course(ux, uy, umeta, 'U-Shape Course',
                os.path.join(out_dir, 'u_shape_course.png'))

    # U-shape simulated driving
    u_path = simulate_driving(ux, uy, uh, ud, num_laps=5, seed=123)
    print(f"U-shape driving: {len(u_path)} points, "
          f"{u_path.total_distance:.1f}m total")
    plot_driving(ux, uy, u_path, 'U-Shape',
                 os.path.join(out_dir, 'u_shape_driving.png'))

    print(f"\nAll plots saved to {out_dir}/")
