# AS2_ardupilot — Aerostack2 + ArduPilot Integration

Bringing [Aerostack2](https://github.com/aerostack2/aerostack2) — an open ROS 2
framework for autonomous drones — to ArduPilot Copter, via MAVROS. Aerostack2
has, to date, only ever been used with PX4; this project adds working
ArduPilot support and integrates GCOPTER (MINCO-based smooth trajectory
generation) on top of it.

**Status: paused, handed off.** The core flight platform is built, tested,
and working — verified against real recorded flight data. Tuning and the
planned obstacle-avoidance phase (JPS3D) were not started. See
`HANDOFF.md` for the exact current state, what's proven, what's open, and
where to pick this up.

## What this actually is

```
mission script (Python)
  -> Aerostack2 behaviors (takeoff, go-to, follow-path)
  -> TrajectoryGeneratorBehavior (GCOPTER plugin)
  -> as2_motion_controller (bypass mode)
  -> as2_platform_mavlink   <- built/rewritten in this project
  -> MAVROS
  -> ArduPilot Copter (SITL)
  -> Gazebo (physics simulation)
```

Only `as2_platform_mavlink` was substantially rewritten. Everything else is
existing Aerostack2 / MAVROS infrastructure, used largely as-is. Two small,
general (non-ArduPilot-specific) bugs were also found and fixed in
Aerostack2's own core (`as2_core/src/aerial_platform.cpp`).

## Repository layout

```
src/
  aerostack2/            Aerostack2 framework (upstream, with 2 small local patches)
  as2_platform_mavlink/  ArduPilot platform support (rewritten in this project)
  project_mavlink/       Mission scripts, launch files, SITL configuration
```

## Prerequisites

- Ubuntu 22.04 / ROS 2 Humble
- ArduPilot + `ardupilot_gazebo` (Gazebo Harmonic) built and working standalone
  first — confirm you can fly a manual ArduPilot Copter SITL + Gazebo session
  before layering this on top.
- MAVROS (`ros-humble-mavros`, `ros-humble-mavros-extras`) with the
  GeographicLib datasets installed.

## Build

```bash
cd ~/ardupilot_as2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The GCOPTER plugin (`as2_behaviors_trajectory_generation`) fetches its
backend library (`gcopter_trajectory_generator_lib`) automatically via
CMake `FetchContent` on first build — this needs network access to
`github.com`/`codeload.github.com` and will make the first build of that
package noticeably slower. This is expected, not a hang.

## Running it

**Always sourced, every terminal:**
```bash
source /opt/ros/humble/setup.bash
source ~/ardupilot_as2_ws/install/setup.bash
```

**1. Gazebo — headless.** See `HANDOFF.md` for why headless is currently
required, not optional.
```bash
cd ~/ardupilot_gazebo/worlds
gz sim -v4 -r -s iris_runway.sdf
```
Check the simulation is actually running close to real-time before doing
anything else:
```bash
gz topic -e -t /world/iris_runway/stats -n 5
```
`real_time_factor` should be consistently close to 1.0.

**2. ArduPilot SITL:**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
  --out=udp:127.0.0.1:14551
```

**3. Aerostack2 + MAVROS:**
```bash
cd ~/ardupilot_as2_ws/src/project_mavlink
./launch_as2.bash -n drone0
```

**4. Confirm it's actually connected and running the right plugin before
trusting anything downstream:**
```bash
ros2 topic echo --once /drone0/mavros/state          # expect connected: true
ros2 param get /drone0/TrajectoryGeneratorBehavior plugin_name
                                                      # expect gcopter_trajectory_generator
```

**5. Run a mission:**
```bash
python3 src/project_mavlink/mission_trajectory.py -n drone0
```

## Validating a result — don't trust a flight without this

Every claim of "it works" in this project is backed by recorded flight
data, checked against what was actually commanded — not by "it didn't
crash." The mission script writes `stage_timestamps.json`; record a bag
alongside it and analyze both together:

```bash
rm -rf my_test
ros2 bag record -o my_test /drone0/self_localization/pose < /dev/null &
python3 src/project_mavlink/mission_trajectory.py -n drone0
kill %1
python3 src/project_mavlink/analyze_trajectory.py my_test stage_timestamps.json
```

This prints per-stage tracking accuracy, smoothness, and a plain
GOOD / MARGINAL / POOR verdict with concrete tuning suggestions, plus a
color-coded plot of the flown path. **The `< /dev/null` on the record
command is not optional** — without it, `ros2 bag record` waits on
keyboard input for pause/resume and silently freezes in the background,
producing an empty or truncated recording. This cost significant time to
discover; don't skip it.

See `HANDOFF.md` for the full list of hard-won lessons like this one.

## License

BSD-3-Clause, matching Aerostack2 and MAVROS conventions.
