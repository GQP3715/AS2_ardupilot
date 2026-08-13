# Aerostack2 + ArduPilot integration — implementation notes

## A. Final architecture

```
mission_*.py  (as2_python_api)
      v
Behaviors: Takeoff / GoTo / FollowPath / Land  +  TrajectoryGeneratorBehavior
      v   motion_reference/trajectory   (position, velocity, acceleration, yaw @ <ns>/odom, ENU)
as2_motion_controller  -- BYPASS: platform advertises TRAJECTORY, so the plugin is not used
      v   actuator_command/trajectory
as2_platform_mavlink (MavlinkPlatform)
      v   mavros/setpoint_raw/local  (mavros_msgs/PositionTarget, ENU)
MAVROS  -- converts ENU->NED, forwards coordinate_frame + type_mask untouched
      v   SET_POSITION_TARGET_LOCAL_NED  (pos + vel + accel + yaw, MAV_FRAME_LOCAL_NED)
ArduPilot Copter, GUIDED mode  (its own position controller does the tracking)
      v
Gazebo (ardupilot_gazebo, JSON FDM)
```

Takeoff / land / hover / emergency are commanded out of band:
`mavros/cmd/takeoff` (MAV_CMD_NAV_TAKEOFF), `mavros/cmd/land` (MAV_CMD_NAV_LAND),
`mavros/set_mode` (GUIDED / LOITER / BRAKE), `mavros/cmd/command`
(MAV_CMD_COMPONENT_ARM_DISARM, param2 = 21196, for the kill switch).

**aerostack2 itself is unmodified.** The seam used is
`ControllerHandler::tryToBypassController()`: when the platform advertises the
requested mode, the controller republishes the reference untouched. That is what
lets a full `as2_msgs/TrajectorySetpoints` (position + twist + acceleration + yaw)
reach the platform intact, so no second position loop is added on top of
ArduPilot's.

### Coordinate frames (verified)

| Stage | Frame |
|---|---|
| `mavros/local_position/odom` | ENU at the ArduPilot **EKF origin** |
| Platform relabels it to | `<ns>/odom` (AS2 odom frame) |
| Controller output frame (LOCAL_ENU) | `<ns>/odom` — same frame, no transform needed |
| Published `PositionTarget` | **ENU**, `coordinate_frame = FRAME_LOCAL_NED (1)` |
| MAVROS `setpoint_raw` `local_cb` | applies `transform_frame_enu_ned` to position, velocity, acceleration, yaw and yaw_rate |
| ArduPilot | NED relative to the EKF origin |

So the platform must **not** pre-convert to NED. `FRAME_BODY_NED` /
`FRAME_BODY_OFFSET_NED` are deliberately never used (MAVROS applies the same
ENU→NED rotation to them, which is wrong — mavros issue #801).

`type_mask` values used (0 = use the field):
* TRAJECTORY + yaw angle → `IGNORE_YAW_RATE` only (pos+vel+accel+yaw)
* POSITION → `IGNORE_V* | IGNORE_AF* | IGNORE_YAW_RATE`
* SPEED → `IGNORE_P* | IGNORE_AF* | (IGNORE_YAW or IGNORE_YAW_RATE)`
* HOVER → position-only hold at the pose latched when the mode was set

## B. Files

```
modified:
- as2_platform_mavlink/include/as2_platform_mavlink/mavlink_platform.hpp   (full rewrite)
- as2_platform_mavlink/src/mavlink_platform.cpp                            (full rewrite)
- as2_platform_mavlink/config/platform_config_file.yaml   (new ArduPilot parameters)
- as2_platform_mavlink/config/control_modes.yaml          (drop ACRO/ATTITUDE, add HOVER + TRAJECTORY)
- as2_platform_mavlink/config/mavros_config.yaml          (ArduPilot SITL UDP ports)
- as2_platform_mavlink/launch/mavros_launch.py            (autopilot arg -> apm_config/apm_pluginlists)
- as2_platform_mavlink/CMakeLists.txt                     (build autopilot_profile.cpp)
- project_mavlink/config/config.yaml                      (platform/behavior/mavros retarget)
- project_mavlink/tmuxinator/aerostack2.yaml              (autopilot:=ardupilot)
- project_mavlink/tmuxinator/sitl_simulation.yaml         (Gazebo + ArduPilot SITL)
- project_mavlink/launch_sitl.bash                        (ArduPilot SITL session)
- project_mavlink/stop_tmuxinator_sitl.bash
- project_mavlink/mission.py, mission_gps.py              (offboard before arm)

created:
- as2_platform_mavlink/include/as2_platform_mavlink/autopilot_profile.hpp
- as2_platform_mavlink/src/autopilot_profile.cpp
- project_mavlink/sitl_config/ardupilot/world.yaml
- project_mavlink/sitl_config/ardupilot/launch_gazebo.bash
- project_mavlink/sitl_config/ardupilot/run_instance.py
- project_mavlink/mission_trajectory.py

deleted:
- none   (sitl_config/docker/* is the legacy PX4 path, left untouched but unused)
```

### Why each incompatibility mattered

1. **`mode == "OFFBOARD"`** in the state callback → with ArduPilot `offboard`
   stayed false forever, so `AerialPlatform::sendCommand()` returned early.
   *This is the "arms but does not move" failure.* Now the offboard mode name
   comes from the autopilot profile (`GUIDED`).
2. **No `ownTakeoff`/`ownLand`** → ArduPilot Copter does not leave the ground in
   GUIDED from position/velocity setpoints; it needs MAV_CMD_NAV_TAKEOFF.
3. **ACRO advertised** → ArduPilot documents SET_ATTITUDE_TARGET body rates as
   not supported. Removed.
4. **ATTITUDE thrust normalised by `max_thrust`** → ArduPilot reads that field as
   a *climb rate* (0.5 = hold) unless `GUID_OPTIONS` bit 3 is set. ATTITUDE is
   now refused with an explanatory error unless
   `attitude_thrust_semantics: normalized_thrust` is set.
5. **No TRAJECTORY mode** → smooth trajectories were flattened into PID-generated
   velocity commands, duplicating ArduPilot's own controller.
6. **`sendCommand()` override streamed body rates while disarmed** (a PX4 idiom
   for keeping OFFBOARD alive) and bypassed the base-class safety gating. The
   override is gone; the base class gating is used.
7. **`px4_config.yaml` / `px4_pluginlists.yaml`** → replaced by the `apm_*` files.
8. **PX4 UDP ports and PX4 SITL docker** → ArduPilot SITL + `ardupilot_gazebo`.
9. **Vehicle type never declared** → `vehicle_type: copter`, validated at startup
   (fixed wing/VTOL are rejected with an explicit message).

### Safety / state logic added

* Platform starts **disconnected**; `connected` follows the MAVROS heartbeat and
  a watchdog (`connection.timeout`) clears it if state messages stop.
* Arming is refused without a connection or without a local position (no EKF
  origin ⇒ GUIDED setpoints would be rejected), and GUIDED is entered *before*
  arming so ArduPilot's arming checks are meaningful.
* `control.command_timeout`: if references go stale the setpoint stream stops and
  ArduPilot brakes (its own 3 s rule) instead of flying on an outdated command.
* `ownStopPlatform` switches to BRAKE once (not once per command cycle).
* Blocking takeoff/land use a dedicated callback group + executor, so autopilot
  feedback (and therefore AS2 odometry and TF) keeps flowing while waiting.

## C. Build

```bash
# workspace layout: ~/as2_ws/src/{aerostack2,as2_platform_mavlink,project_mavlink}
sudo apt install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh && sudo ./install_geographiclib_datasets.sh

cd ~/as2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to as2_platform_mavlink
source install/setup.bash
```

## D. Launch

```bash
# terminal 1 — Gazebo + ArduPilot SITL (needs ardupilot_gazebo env vars)
cd ~/as2_ws/src/project_mavlink && ./launch_sitl.bash

# terminal 2 — Aerostack2 + MAVROS for drone0
./launch_as2.bash -n drone0

# terminal 3 — mission
python3 mission_trajectory.py -n drone0
```

Environment needed by `sitl_config/ardupilot/launch_gazebo.bash`:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

Keep `use_sim_time` **false**: nothing publishes `/clock` in this stack.

## E. Test commands

```bash
# 1 connection
ros2 topic echo /drone0/mavros/state --once          # connected: true, mode: STABILIZE/GUIDED
# 2 state feedback
ros2 topic echo /drone0/sensor_measurements/odom --once
ros2 topic echo /drone0/platform/info --once         # connected/armed/offboard
ros2 run tf2_ros tf2_echo drone0/odom drone0/base_link
# 3-4 arm + mode
ros2 service call /drone0/platform/set_offboard_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /drone0/platform/set_arming_state std_srvs/srv/SetBool "{data: true}"
# 5 takeoff
ros2 service call /drone0/platform/takeoff std_srvs/srv/SetBool "{data: true}"
# 6-8 trajectory: what actually leaves the platform and what ArduPilot echoes back
ros2 topic echo /drone0/actuator_command/trajectory
ros2 topic hz   /drone0/mavros/setpoint_raw/local     # ~= cmd_freq (50 Hz)
ros2 topic echo /drone0/mavros/setpoint_raw/local --once
ros2 topic echo /drone0/mavros/setpoint_raw/target_local --once   # ArduPilot's accepted target
# 9 cancel
ros2 action list | grep -i trajectory                 # then cancel the goal
# 10 land
ros2 service call /drone0/platform/land std_srvs/srv/SetBool "{data: true}"
```

In the MAVProxy console: `mode`, `arm throttle`, `status`, and
`module load message` for raw MAVLink inspection.

## F. Configuration summary

`as2_platform_mavlink/config/platform_config_file.yaml` (overridable per project
in `project_mavlink/config/config.yaml` under `platform:`):
`autopilot`, `vehicle_type`, `modes.{offboard,manual,hold}`,
`attitude_thrust_semantics`, `max_thrust`/`min_thrust`, `arm_on_offboard`,
`connection.timeout`, `control.{command_timeout,mode_change_timeout}`,
`takeoff.{height,height_tolerance,timeout,blocking}`, `land.timeout`,
`trajectory.{send_acceleration,sampling_dt}`, `external_odom`, `cmd_freq`.

**Known limitation:** `takeoff_plugin_platform` calls a `std_srvs/SetBool`
service, which carries no payload, so the Takeoff *action* height is ignored —
the height comes from `takeoff.height`. Keep it consistent with the missions.

## G. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `platform/info.connected: false` | MAVROS not linked. Check `fcu_url` vs the `--out` port of `sim_vehicle.py`; `ros2 topic echo /drone0/mavros/state`. |
| Arms but does not move | Check `platform/info.offboard`; if false, `modes.offboard` does not match the reported mode string. Then check `mavros/setpoint_raw/local` is publishing and `setpoint_raw/target_local` echoes it. |
| Refuses to arm | "no local position received" ⇒ EKF has no origin (GPS not ready). Wait for `mavros/local_position/odom`; check `EK3_SRC*` and the SITL GPS lock. |
| GUIDED refused | ArduPilot rejects GUIDED without a position estimate; the mode-change wait times out with an explicit log. |
| Takeoff command accepted, no climb | Not armed, or not in GUIDED; ArduPilot needs both before MAV_CMD_NAV_TAKEOFF. Some ArduPilot versions mis-handle the CommandTOL altitude — fall back to `mavros/cmd/command` MAV_CMD 22 with param7. |
| Inverted altitude / mirrored motion | Something pre-converted ENU→NED. The platform must publish **ENU** with `FRAME_LOCAL_NED`; never use the BODY frames. |
| Vehicle stops mid-trajectory every ~3 s | Setpoint stream dropped below 1 Hz, or `control.command_timeout` fired. Check `ros2 topic hz /drone0/mavros/setpoint_raw/local`. |
| Motion is jerky | `trajectory.send_acceleration` off, or `cmd_freq` too low, or the generator's `max_acceleration` exceeds `WPNAV_ACCEL`. |
| TF errors | State estimator not running or `use_gps`/origin not set; check `tf2_echo earth drone0/odom`. |
| Gazebo not connecting | ArduPilot SITL instance N uses FDM ports 9002+10N; start Gazebo before SITL (run_instance.py already waits 5 s). |

## Status / what is NOT verified

* **Nothing was built or run.** This container has no ROS 2, no MAVROS, no
  ArduPilot and no Gazebo. `src/autopilot_profile.cpp` compiles standalone with
  g++ -Wall -Wextra; `mavlink_platform.cpp` has only been checked structurally
  (header/implementation symbol match, brace/paren balance). All YAML files parse
  and all Python/bash files pass syntax checks.
* Therefore: run `colcon build` first and expect to fix small compile issues, then
  work through tests 1–10 above. No claim is made that the vehicle flies.
* MAVROS/ArduPilot API details were verified against the ArduPilot wiki
  (Copter Commands in Guided Mode), the MAVROS `setpoint_raw` source and the
  `mavros_msgs` message definitions — not against a running system.
* The PX4 profile is preserved in intent but has not been re-validated.
* Remaining nice-to-haves: update both READMEs, and a `ros2 bag` based tracking
  error check for test 8.