#!/usr/bin/env python3
"""
Analyze a recorded /drone0/self_localization/pose bag against the trajectory
requested by mission_trajectory.py (straight leg + closed helix).

Run on the machine with ROS 2 sourced:
    source /opt/ros/humble/setup.bash
    source ~/ardupilot_as2_ws/install/setup.bash
    python3 analyze_trajectory.py mission_check
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
HEIGHT = 1.0
AMPLITUDE = 0.5
NUM_WAYPOINTS = 8


def build_helix():
    pts = []
    for i in range(NUM_WAYPOINTS + 1):
        angle = 2.0 * math.pi * i / NUM_WAYPOINTS
        pts.append((
            RADIUS * math.cos(angle),
            RADIUS * math.sin(angle),
        ))
    return pts


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


def analyze(t, x, y, z):
    print(f'Total samples: {len(t)}')
    if len(t) < 2:
        print('Not enough data to analyze.')
        return

    duration = t[-1] - t[0]
    print(f'Duration: {duration:.1f} s, avg rate: {len(t) / duration:.1f} Hz')

    speeds = []
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0:
            continue
        dist = math.dist((x[i], y[i], z[i]), (x[i - 1], y[i - 1], z[i - 1]))
        speeds.append(dist / dt)
    if speeds:
        print(f'Speed: max={max(speeds):.2f} m/s, '
              f'mean={sum(speeds) / len(speeds):.2f} m/s')

    radii = [math.hypot(px, py) for px, py in zip(x, y)]
    circle_samples = [r for r in radii if 1.0 < r < 3.0]
    if circle_samples:
        errors = [abs(r - RADIUS) for r in circle_samples]
        print(f'Radial tracking error vs {RADIUS} m commanded radius: '
              f'max={max(errors):.2f} m, mean={sum(errors) / len(errors):.2f} m '
              f'(n={len(circle_samples)} samples)')

    print(f'Start position: ({x[0]:.2f}, {y[0]:.2f}, {z[0]:.2f})')
    print(f'End position:   ({x[-1]:.2f}, {y[-1]:.2f}, {z[-1]:.2f})')


def plot(x, y, out_path='trajectory_plot.png'):
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(x, y, '-o', markersize=2, linewidth=1, label='Flown path', color='tab:blue')

    circle_x = [RADIUS * math.cos(2 * math.pi * i / 200) for i in range(201)]
    circle_y = [RADIUS * math.sin(2 * math.pi * i / 200) for i in range(201)]
    ax.plot(circle_x, circle_y, '--', color='tab:orange', label='Commanded circle (r=2m)')

    ax.plot(STRAIGHT_LEG[0], STRAIGHT_LEG[1], 'gs', markersize=10, label='Straight-leg target')
    ax.plot(x[0], y[0], 'k^', markersize=10, label='Start')
    ax.plot(x[-1], y[-1], 'kv', markersize=10, label='End (landed)')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Flown path vs commanded trajectory')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    fig.savefig(out_path, dpi=150)
    print(f'Saved plot to {out_path}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: python3 analyze_trajectory.py <bag_path>')
        sys.exit(1)

    t, x, y, z = read_bag(sys.argv[1])
    analyze(t, x, y, z)
    if x:
        plot(x, y)
