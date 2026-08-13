#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Smooth trajectory mission for a single ArduPilot drone.

The mission exercises the full trajectory path of the stack:

    TrajectoryGeneratorBehavior -> motion_reference/trajectory
      -> as2_motion_controller (bypass) -> actuator_command/trajectory
      -> as2_platform_mavlink -> mavros/setpoint_raw/local
      -> SET_POSITION_TARGET_LOCAL_NED (position + velocity + acceleration)
      -> ArduPilot GUIDED

Stage 1 is a single straight leg (smooth acceleration, constant velocity,
smooth deceleration) and stage 2 is a closed 3D trajectory flown as one
continuous path.
"""

__authors__ = 'Mohamed Elmahlawy'
__license__ = 'BSD-3-Clause'

import argparse
import math
from time import sleep

from as2_python_api.drone_interface import DroneInterface
import rclpy

TAKE_OFF_HEIGHT = 1.0  # m, must match platform takeoff.height in config/config.yaml
TAKE_OFF_SPEED = 1.0  # m/s
SPEED = 1.0  # m/s
LAND_SPEED = 0.5  # m/s
SLEEP_TIME = 1.0  # s between behaviors

# Stage 1: one straight leg, accelerate / cruise / decelerate.
STRAIGHT_LEG = [4.0, 0.0, TAKE_OFF_HEIGHT]

# Stage 2: closed 3D path flown as a single trajectory.
RADIUS = 2.0
HEIGHT_AMPLITUDE = 0.5
NUM_WAYPOINTS = 8


def build_helix(radius: float, height: float, amplitude: float, samples: int) -> list:
    """
    Build a closed 3D path with a sinusoidal height profile.

    :param radius: horizontal radius of the path (m)
    :param height: mean height of the path (m)
    :param amplitude: height oscillation amplitude (m)
    :param samples: number of waypoints
    :return: list of [x, y, z] waypoints
    """
    path = []
    for index in range(samples):
        angle = 2.0 * math.pi * index / samples
        path.append([
            radius * math.cos(angle),
            radius * math.sin(angle),
            height + amplitude * math.sin(2.0 * angle),
        ])
    return path


def drone_start(drone_interface: DroneInterface) -> bool:
    """
    Arm the drone, take control of it and take off.

    :param drone_interface: DroneInterface object
    :return: True if the drone took off
    """
    print('Offboard (ArduPilot GUIDED)')
    if not drone_interface.offboard():
        print('Could not enter GUIDED mode')
        return False

    print('Arm')
    if not drone_interface.arm():
        print('Could not arm the vehicle, check the ArduPilot pre-arm checks')
        return False

    print('Take off')
    return drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)


def drone_run(drone_interface: DroneInterface) -> bool:
    """
    Fly the two trajectory stages.

    :param drone_interface: DroneInterface object
    :return: True if both stages succeeded
    """
    print(f'Stage 1: straight leg to {STRAIGHT_LEG} at {SPEED} m/s')
    if not drone_interface.go_to.go_to_point(STRAIGHT_LEG, speed=SPEED):
        print('Straight leg failed')
        return False
    sleep(SLEEP_TIME)

    path = build_helix(RADIUS, TAKE_OFF_HEIGHT, HEIGHT_AMPLITUDE, NUM_WAYPOINTS)
    print(f'Stage 2: closed 3D trajectory through {len(path)} waypoints')
    if not drone_interface.follow_path.follow_path_with_path_facing(path, speed=SPEED):
        print('3D trajectory failed')
        return False
    sleep(SLEEP_TIME)
    return True


def drone_end(drone_interface: DroneInterface) -> bool:
    """
    Land the drone and release the offboard control.

    :param drone_interface: DroneInterface object
    :return: True if the drone landed
    """
    print('Land')
    if not drone_interface.land(speed=LAND_SPEED):
        print('Land failed')
        return False

    print('Manual')
    return drone_interface.manual()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Smooth trajectory mission')
    parser.add_argument('-n', '--namespace', type=str, default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose', action='store_true', default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time', action='store_true', default=False,
                        help='Use simulation time')

    args = parser.parse_args()

    rclpy.init()

    uav = DroneInterface(
        drone_id=args.namespace,
        use_sim_time=args.use_sim_time,
        verbose=args.verbose)

    success = drone_start(uav)
    if success:
        success = drone_run(uav)
    success = drone_end(uav) and success

    uav.shutdown()
    rclpy.shutdown()
    print(f'Mission finished, success: {success}')
    exit(0 if success else 1)