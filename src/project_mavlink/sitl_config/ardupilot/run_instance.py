#!/usr/bin/env python3

"""Launch one ArduPilot SITL instance described in the simulation config file."""

__authors__ = 'Mohamed Elmahlawy'
__license__ = 'BSD-3-Clause'

import argparse
import os
import shlex
from pathlib import Path
from time import sleep

import yaml


def read_config(config_file: Path) -> dict:
    """
    Read the simulation configuration file.

    :param config_file: path to the YAML simulation configuration file
    :return: parsed configuration
    """
    if not config_file.exists():
        raise FileNotFoundError(f'File {config_file} not found')
    with open(config_file, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def find_drone(config: dict, namespace: str) -> dict:
    """
    Find the entry of a drone namespace.

    :param config: parsed simulation configuration
    :param namespace: drone namespace
    :return: drone entry
    """
    for index, drone in enumerate(config.get('drones', [])):
        if drone['namespace'] == namespace:
            drone['index'] = index
            return drone
    raise KeyError(f'No drone found with namespace: {namespace}')


def build_command(config: dict, drone: dict, extra_args: str) -> str:
    """
    Build the sim_vehicle.py command line of a drone.

    :param config: parsed simulation configuration
    :param drone: drone entry
    :param extra_args: extra arguments appended to sim_vehicle.py
    :return: command to execute
    """
    command = [
        'sim_vehicle.py',
        '-v', str(config['vehicle']),
        '-f', str(config['frame']),
        '--model', 'JSON',
        '-I', str(drone['instance']),
        '--sysid', str(drone['sysid']),
        '--out', str(drone['mavros_out']),
        '--console',
    ]
    if extra_args:
        command.extend(shlex.split(extra_args))
    return ' '.join(command)


def main():
    """Entrypoint."""
    parser = argparse.ArgumentParser(
        description='Run an ArduPilot SITL instance from a YAML simulation file.')
    parser.add_argument('-p', '--config_file', type=str, required=True,
                        help='Path to the YAML simulation configuration file.')
    parser.add_argument('-n', '--namespace', type=str, required=True,
                        help='Namespace of the drone.')
    parser.add_argument('-e', '--extra_args', type=str, default='',
                        help='Extra arguments forwarded to sim_vehicle.py.')

    args = parser.parse_args()

    config = read_config(Path(args.config_file))
    drone = find_drone(config, args.namespace)

    # Gazebo must be up before the first instance connects to the FDM port,
    # and the instances must not race for the shared MAVProxy resources.
    sleep(5.0 + 2.0 * drone['index'])

    command = build_command(config, drone, args.extra_args)
    print(f'Executing command: {command}')
    raise SystemExit(os.system(command))


if __name__ == '__main__':
    main()