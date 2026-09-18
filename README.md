# AS2_ardupilot — Aerostack2 + ArduPilot Integration

Aerostack2 has, to date, only ever been used with PX4 (see
[aerostack2/aerostack2#984](https://github.com/aerostack2/aerostack2/issues/984)).
This project adds working ArduPilot Copter support via MAVROS, and layers
GCOPTER (MINCO-based smooth trajectory generation) on top of it.

## What's proven here, with evidence

- **Full ArduPilot flight cycle through Aerostack2**: connect -> arm ->
  GUIDED -> takeoff -> trajectory -> land -> disarm. Verified repeatedly
  against recorded flight data (rosbags), not just "the mission printed
  success."
- **`as2_platform_mavlink` had several PX4-only assumptions that silently
  break on ArduPilot** — fixed and isolated behind a small autopilot-profile
  abstraction, so the existing PX4 path is untouched:
  - checked for `mode == "OFFBOARD"` (ArduPilot has no such mode; `GUIDED`
    is the equivalent)
  - never sent an autopilot-side takeoff command (ArduPilot won't leave the
    ground in GUIDED without `MAV_CMD_NAV_TAKEOFF`)
  - advertised `ACRO` control (ArduPilot doesn't honor `SET_ATTITUDE_TARGET`
    body rates)
  - assumed PX4's thrust semantics on that same message (ArduPilot treats
    that field as a climb rate by default, not normalized thrust)
- **GCOPTER integration measurably improves on Aerostack2's stock
  `jerk_limited_trajectory_generator`** on the same waypoints and commanded
  speed: faster completion (46.7s vs 54.7s) and roughly half the jerk
  (smoothness proxy), at a small (~3cm) cost in mean position accuracy.
  Reproducible with `compare_trajectories.py` (plot: `trajectory_comparison.png`).
- **Straight-line and takeoff/land accuracy: 3-9 cm**, consistently.
- **Two real, non-ArduPilot-specific bugs found and fixed in Aerostack2's
  own core** (`as2_core/src/aerial_platform.cpp`): `setArmingState()` /
  `setOffboardControl()` returned `false` (failure) on an idempotent no-op
  call (e.g. disarming an already-disarmed vehicle), misleading callers
  into thinking something failed. Maintainers have offered to review this
  as a standalone PR.

## What's not done / known open items

- GCOPTER's anchor/corridor tightening (`waypoints.waypoint_margin`,
  `waypoints.waypoint_anchor_radius`, `optimization.corridor_margin`) was
  never tuned in `config/config.yaml` — every result above was flown on
  the plugin's defaults.
- Vehicle mass in config is `1.5` kg; the corrected total (base link +
  4 rotor links) is `1.6` kg — identified, not yet applied.
- `mavlinkStateCb()` in `as2_platform_mavlink` doesn't fire
  `handleStateMachineEvent()` on ArduPilot's autonomous post-landing
  disarm, so the internal state machine can log harmless
  `Invalid transition` warnings on repeated runs. Doesn't block missions
  (confirmed over 5+ consecutive runs).
- 3D obstacle-avoidance path planning (JPS3D + GCOPTER) is the intended
  next phase and has not been started. Aerostack2's own
  `as2_behaviors_path_planning` is structurally 2D (`Point2i`-based), so
  this needs a standalone planner.

## Repository layout

```
src/
  aerostack2/            Aerostack2 framework (upstream, with 2 small local patches)
  as2_platform_mavlink/  ArduPilot platform support (rewritten in this project)
  project_mavlink/       Mission scripts, launch files, SITL configuration
```

## Prerequisites

- Ubuntu 22.04 / ROS 2 Humble
- ArduPilot + `ardupilot_gazebo` (Gazebo Harmonic) built and working
  standalone first — confirm you can fly a manual ArduPilot Copter
  SITL + Gazebo session before layering this on top.
- MAVROS (`ros-humble-mavros`, `ros-humble-mavros-extras`) with the
  GeographicLib datasets installed:
  ```bash
  wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
  chmod +x install_geographiclib_datasets.sh
  ./install_geographiclib_datasets.sh
  ```

## Build

```bash
cd ~/ardupilot_as2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The GCOPTER plugin (`as2_behaviors_trajectory_generation`) fetches its
backend library automatically via CMake `FetchContent` on first build —
this needs network access to `github.com`/`codeload.github.com` and will
make the first build of that package noticeably slower. Expected, not a
hang.

## Running it

**Always sourced, every terminal:**
```bash
source /opt/ros/humble/setup.bash
source ~/ardupilot_as2_ws/install/setup.bash
```

**1. Gazebo — headless.** Gazebo's real-time factor is unstable with the
GUI attached under WSL2 (measured 0.37-0.65x vs ~1.0x headless), which
destabilizes the MAVLink heartbeat and can trip connection timeouts.
Headless is the working setup:
```bash
cd ~/ardupilot_gazebo/worlds
gz sim -v4 -r -s iris_runway.sdf
```
Confirm it's actually near real-time before doing anything else:
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

**4. Confirm it's connected and running the right plugin before trusting
anything downstream:**
```bash
ros2 topic echo --once /drone0/mavros/state          # expect connected: true
ros2 param get /drone0/TrajectoryGeneratorBehavior plugin_name
                                                      # expect gcopter_trajectory_generator
```

**5. Run a mission:**
```bash
python3 src/project_mavlink/mission_trajectory.py -n drone0
```
(also available: `mission.py`, `mission_gps.py`, `mission_interpreter.py`,
`mission_behavior_tree.py` — general Aerostack2 mission patterns, not
ArduPilot-specific)

## Validating a result — don't trust a flight without this

```bash
rm -rf my_test
ros2 bag record -o my_test /drone0/self_localization/pose < /dev/null &
python3 src/project_mavlink/mission_trajectory.py -n drone0
kill %1
python3 analyze_trajectory.py my_test
```
This prints tracking accuracy and smoothness against what was actually
commanded, plus a plot of the flown path vs. the commanded trajectory.

**The `< /dev/null` on the record command is not optional** — without it,
`ros2 bag record` waits on keyboard input for pause/resume and silently
freezes in the background, producing an empty or truncated recording.

## Other known quirks

- `mavros/param/get` service can be unavailable depending on MAVROS
  version/config — don't assume it exists when scripting parameter checks.
- ArduPilot's angle-limit parameter in this version is `ATC_ANGLE_MAX` /
  `PSC_ANGLE_MAX`, not `ANGLE_MAX` (older docs reference the latter).
- The custom Aerostack2 YAML parameter parser is fragile — malformed
  structure (empty `{}` blocks, mixed indentation) can crash it outright.

## Stopping everything

```bash
./stop_tmuxinator_as2.bash drone0
./stop_tmuxinator_ground_station.bash
# or, to force-kill everything:
tmux kill-server
```

## License

BSD-3-Clause, matching Aerostack2 and MAVROS conventions.
