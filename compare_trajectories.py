#!/usr/bin/env python3
"""
Compare two recorded /drone0/self_localization/pose bags on the same overlay,
with quantitative metrics side by side (not just visual inspection).

Usage:
    python3 compare_trajectories.py <bag_a> <label_a> <bag_b> <label_b>

Example:
    python3 compare_trajectories.py jerk_check "Jerk-limited" gcopter_check "GCOPTER"
"""

import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy.serialization
from geometry_msgs.msg import PoseStamped
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

# Must match mission_trajectory.py
STRAIGHT_LEG = [4.0, 0.0, 1.0]
RADIUS = 2.0


def read_bag(path, topic='/drone0/self_localization/pose'):
    storage_options = StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    t, x, y, z = [], [], [], []
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name != topic:
            continue
        msg = rclpy.serialization.deserialize_message(data, PoseStamped)
        t.append(timestamp * 1e-9)
        x.append(msg.pose.position.x)
        y.append(msg.pose.position.y)
        z.append(msg.pose.position.z)
    return t, x, y, z


def metrics(label, t, x, y, z):
    result = {'label': label}
    if len(t) < 2:
        print(f'{label}: not enough data')
        return result

    duration = t[-1] - t[0]
    result['duration'] = duration
    result['samples'] = len(t)
    result['rate'] = len(t) / duration

    speeds = []
    accels = []
    prev_speed = None
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0:
            continue
        dist = math.dist((x[i], y[i], z[i]), (x[i - 1], y[i - 1], z[i - 1]))
        speed = dist / dt
        speeds.append(speed)
        if prev_speed is not None:
            accels.append(abs(speed - prev_speed) / dt)
        prev_speed = speed

    result['speed_max'] = max(speeds) if speeds else 0.0
    result['speed_mean'] = sum(speeds) / len(speeds) if speeds else 0.0
    result['speed_std'] = (
        (sum((s - result['speed_mean']) ** 2 for s in speeds) / len(speeds)) ** 0.5
        if speeds else 0.0
    )
    result['accel_max'] = max(accels) if accels else 0.0
    result['accel_mean'] = sum(accels) / len(accels) if accels else 0.0

    radii = [math.hypot(px, py) for px, py in zip(x, y)]
    circle_samples = [r for r in radii if 1.0 < r < 3.0]
    if circle_samples:
        errors = [abs(r - RADIUS) for r in circle_samples]
        result['radial_error_max'] = max(errors)
        result['radial_error_mean'] = sum(errors) / len(errors)
        result['radial_error_n'] = len(circle_samples)

    result['start'] = (x[0], y[0], z[0])
    result['end'] = (x[-1], y[-1], z[-1])
    result['xy'] = list(zip(x, y))
    return result


def print_comparison(m_a, m_b):
    print(f"\n{'Metric':<28} {m_a['label']:>18} {m_b['label']:>18}")
    print('-' * 66)
    rows = [
        ('Duration (s)', 'duration', '{:.1f}'),
        ('Samples', 'samples', '{:d}'),
        ('Avg rate (Hz)', 'rate', '{:.1f}'),
        ('Speed max (m/s)', 'speed_max', '{:.2f}'),
        ('Speed mean (m/s)', 'speed_mean', '{:.2f}'),
        ('Speed std dev (m/s)', 'speed_std', '{:.2f}'),
        ('Accel proxy max (m/s^2)', 'accel_max', '{:.2f}'),
        ('Accel proxy mean (m/s^2)', 'accel_mean', '{:.2f}'),
        ('Radial error max (m)', 'radial_error_max', '{:.2f}'),
        ('Radial error mean (m)', 'radial_error_mean', '{:.2f}'),
    ]
    for name, key, fmt in rows:
        va = m_a.get(key)
        vb = m_b.get(key)
        sa = fmt.format(va) if va is not None else 'n/a'
        sb = fmt.format(vb) if vb is not None else 'n/a'
        print(f'{name:<28} {sa:>18} {sb:>18}')

    print()
    print(f"{m_a['label']} start->end: "
          f"({m_a['start'][0]:.2f},{m_a['start'][1]:.2f}) -> "
          f"({m_a['end'][0]:.2f},{m_a['end'][1]:.2f})")
    print(f"{m_b['label']} start->end: "
          f"({m_b['start'][0]:.2f},{m_b['start'][1]:.2f}) -> "
          f"({m_b['end'][0]:.2f},{m_b['end'][1]:.2f})")

    print()
    print('Read: lower accel-proxy and lower radial-error mean/max = smoother, '
          'more accurate tracking. This does NOT tell you which one is the '
          '"better" trajectory in an absolute sense -- only which tracked the '
          'SAME commanded path more closely/smoothly on this run.')


def plot_overlay(m_a, m_b, out_path='trajectory_comparison.png'):
    fig, ax = plt.subplots(figsize=(8, 8))

    if 'xy' in m_a:
        xa, ya = zip(*m_a['xy'])
        ax.plot(xa, ya, '-o', markersize=2, linewidth=1.2,
                label=m_a['label'], color='tab:blue')
    if 'xy' in m_b:
        xb, yb = zip(*m_b['xy'])
        ax.plot(xb, yb, '-o', markersize=2, linewidth=1.2,
                label=m_b['label'], color='tab:red')

    circle_x = [RADIUS * math.cos(2 * math.pi * i / 200) for i in range(201)]
    circle_y = [RADIUS * math.sin(2 * math.pi * i / 200) for i in range(201)]
    ax.plot(circle_x, circle_y, '--', color='gray', linewidth=1,
            label='Commanded circle (r=2m)')
    ax.plot(STRAIGHT_LEG[0], STRAIGHT_LEG[1], 'gs', markersize=10,
            label='Straight-leg target')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Trajectory generator comparison: same waypoints, same speed')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    fig.savefig(out_path, dpi=150)
    print(f'\nSaved overlay plot to {out_path}')


if __name__ == '__main__':
    if len(sys.argv) != 5:
        print('Usage: python3 compare_trajectories.py <bag_a> <label_a> <bag_b> <label_b>')
        sys.exit(1)

    bag_a, label_a, bag_b, label_b = sys.argv[1:5]

    t_a, x_a, y_a, z_a = read_bag(bag_a)
    t_b, x_b, y_b, z_b = read_bag(bag_b)

    m_a = metrics(label_a, t_a, x_a, y_a, z_a)
    m_b = metrics(label_b, t_b, x_b, y_b, z_b)

    print_comparison(m_a, m_b)
    plot_overlay(m_a, m_b)
