# HANDOFF — exact current state, as of project pause

This document is written for whoever picks this project up next. It is
deliberately precise about what is *actually true in the repository right
now*, separate from what was tested locally and never committed, and
separate from what was discussed but never done. Every claim below was
checked against the actual source at handoff time — not recalled from
memory — so it should be trustworthy as a starting point.

## 1. What is proven, with evidence

- **Full flight cycle works**: connect -> arm -> GUIDED -> takeoff ->
  trajectory -> land -> disarm, verified repeatedly against recorded flight
  data (bag files), not just "the mission printed success."
- **GCOPTER (MINCO-based trajectory smoothing) is integrated and measurably
  better than Aerostack2's stock `jerk_limited_trajectory_generator`** on
  the same waypoints, same commanded speed: faster completion (46.7s vs
  54.7s) and roughly half the jerk (smoothness proxy), at a small (~3cm)
  cost in mean position accuracy. This comparison is reproducible with
  `compare_trajectories.py`.
- **3D multi-level flight works**: the drone can climb, descend, and move
  laterally between waypoints at different heights, in the correct
  commanded order, confirmed by matching full 3D position (not altitude
  alone) against each waypoint.
- **Straight-line and takeoff/land accuracy: 3-9 cm**, consistently.
- **Two real bugs found and fixed in Aerostack2's own core**
  (`as2_core/src/aerial_platform.cpp`), not ArduPilot-specific: idempotent
  no-op calls to `setArmingState()` / `setOffboardControl()` (e.g.
  disarming an already-disarmed vehicle) previously returned `false`
  (failure) instead of `true`, misleading any caller into thinking
  something failed. **These are fixed in this repo** (verified: both
  functions now `return true;` on the matching-state branches). This is a
  good candidate for a small, self-contained upstream PR to Aerostack2,
  independent of anything ArduPilot-specific — Aerostack2's maintainers
  have been informally made aware this project exists and have offered to
  review contributions (no PR has actually been opened yet).

## 2. What is NOT done — verified against the actual repo, not assumed

- **GCOPTER tuning was never applied.** `config/config.yaml`'s
  `gcopter_trajectory_generator` block currently has only `drone.mass`,
  `drone.gravity`, `limits.max_velocity`, `limits.max_tilt_angle`. It has
  **no** `waypoints.waypoint_margin`, `waypoints.waypoint_anchor_radius`,
  or `optimization.corridor_margin`. Per the plugin's own C++ (see
  `readConfigParameters()` in
  `as2_behaviors_trajectory_generation/.../gcopter_trajectory_generator.cpp`),
  omitting these means the anchor/corridor tightening mechanism is
  **effectively disabled** — every result recorded so far was flown
  without it. This is the most likely single highest-leverage next step;
  see Section 4.
- **The vehicle mass in config is `1.5` kg, not the corrected `1.6` kg.**
  `1.5` is only the `base_link` mass in the Gazebo `iris_with_standoffs`
  model; the four rotor links each add `0.025` kg
  (`1.5 + 4*0.025 = 1.6`). This was identified but the config was never
  updated with the corrected total. Low impact, easy fix, listed for
  completeness.
- **`max_velocity` in config is `10.0`** (real ArduPilot `WP_SPD` value,
  pulled live from SITL params), but **all recorded validation runs used
  mission speed `1.0 m/s`** (the `SPEED` constant in
  `mission_trajectory.py`), not the higher config ceiling. The two are
  independent: `max_velocity` is GCOPTER's planning ceiling,`SPEED` is
  what the mission actually commands. Nothing is broken here, just worth
  knowing they aren't the same number.
- **The FSM-sync fix was discussed at length but never applied.**
  `mavlinkStateCb()` in `as2_platform_mavlink/src/mavlink_platform.cpp`
  still unconditionally does `platform_info_msg_.set__armed(msg->armed)`
  with no call to `handleStateMachineEvent()`. Since ArduPilot
  autonomously disarms after landing (no service call from this code),
  the internal `PlatformStateMachine` can desync from reality after the
  first autonomous disarm, producing harmless-but-real
  `Invalid transition: LANDED -> ARM`-style warnings on repeated runs.
  **This does not block missions** — confirmed by 5+ consecutive
  successful runs with this exact gap present. Low priority; the intended
  fix (guard on `msg->armed != platform_info_msg_.armed`, fire
  ARM/DISARM accordingly) was designed but never written into the file.
- **`mission.py` and `mission_gps.py` still have the unsafe shutdown
  pattern** that was found and fixed in `mission_trajectory.py` but never
  back-ported to these two:
  ```python
  success = drone_start(uav) and drone_run(uav)
  success = success and drone_end(uav)
  ```
  Because of Python's short-circuit evaluation, if `drone_start()` or
  `drone_run()` returns `False` **while the vehicle is armed and
  flying**, `drone_end()` (which lands and disarms) is never called at
  all. `mission_trajectory.py` was fixed to always attempt `drone_end()`
  regardless of earlier failures; `mission.py` and `mission_gps.py` were
  never updated to match. **This is a real safety gap, worth fixing
  before either of those two scripts is used again.** The fix:
  ```python
  success = drone_start(uav)
  if success:
      success = drone_run(uav)
  landed_ok = drone_end(uav)  # always attempt, regardless of prior failure
  success = success and landed_ok
  ```
- **No obstacle avoidance exists.** No JPS3D integration, no voxel map, no
  obstacles placed in the simulator. Every waypoint in every test was
  hand-typed in `mission_trajectory.py`. Aerostack2's own built-in path
  planner (`as2_behaviors_path_planning`, A*/Voronoi) was evaluated and
  ruled out early — its `GraphSearcher`/`CellNode` types are hardcoded to
  `Point2i` throughout, genuinely 2D at the type level, not just by
  convention, confirmed by reading the source. JPS3D was the intended
  replacement per the original project scope; none of that work started.

## 3. Known issues and quirks — save yourself the rediscovery time

- **Gazebo real-time factor is unstable with the GUI attached on this
  development machine.** Measured RTF with the GUI open, nothing else
  running: 0.37-0.48 (should be ~1.0). Headless (`gz sim ... -s`) was
  consistently reliable throughout this project and is the assumed mode
  for everything documented here. If you re-enable the GUI, re-verify RTF
  with `gz topic -e -t /world/iris_runway/stats -n 5` before trusting any
  result — a bad RTF run produces symptoms (disconnects, visual glitching,
  jerky recorded trajectories) that look like code bugs but aren't.
- **`ros2 bag record` needs `< /dev/null` or it silently hangs.** It
  listens for spacebar (pause/resume); run in the background without
  redirecting stdin and the shell freezes the job the moment it tries to
  read the keyboard, producing an empty or truncated bag with no error.
  Always: `ros2 bag record -o name TOPIC < /dev/null &`.
- **`tmuxinator/aerostack2.yaml` does not source the workspace itself** —
  it relies on the invoking shell already having
  `install/setup.bash` sourced. If a fresh shell/terminal reports
  "Package not found," this is almost always why.
- **`launch_sitl.bash` uses paths relative to `project_mavlink`** — it
  must be run from inside that directory, not the workspace root.
- **The custom YAML parameter parser in `as2_core` will crash on
  malformed structure**, e.g. an empty `{}` block or mixed comment/code
  indentation. `config.yaml` has been hand-edited many times during this
  project and has a leftover fully-commented block near
  `TrajectoryGeneratorBehavior` — it's inert (every line starts with `#`)
  but if you edit that section again, be careful with indentation; this
  file has broken the whole launch more than once from a stray space.
- **MAVROS's `ANGLE_MAX` naming doesn't exist on this ArduPilot version**
  — the real parameter is `ATC_ANGLE_MAX` (and `PSC_ANGLE_MAX`, which
  reads `0.0` meaning "deferred to ATC_ANGLE_MAX"). If pulling other live
  ArduPilot parameters, don't assume older documentation's naming;
  confirm with `ros2 param list /drone0/mavros/param | grep -i <term>`
  first.
- **`mavros/param/get` (the deprecated single-param service) is not
  exposed by this MAVROS build** — calling it hangs forever waiting for a
  service that will never appear. Use the standard ROS 2 parameter
  interface instead: `ros2 param get /drone0/mavros/param <NAME>`.

## 4. Recommended next steps, in order

1. **Apply the GCOPTER tuning that was identified but never applied.**
   Add to `config/config.yaml` under `gcopter_trajectory_generator:`:
   ```yaml
   waypoints:
     waypoint_margin: 0.15
     waypoint_anchor_radius: 0.4
   optimization:
     corridor_margin: 0.5
   ```
   These starting values are reasoned estimates (tighter than the ~0.4m
   average error measured without them), not independently validated —
   treat the first re-test as calibration, not confirmation. Re-run the
   full validation (bag record + `analyze_trajectory.py`) at the *same*
   speed/radius as prior tests and compare directly against the numbers
   in Section 1.
2. **Back-port the shutdown-safety fix** to `mission.py` and
   `mission_gps.py` (Section 2) — small, low-risk, closes a real gap.
3. **Investigate the waypoint-density-vs-scale finding.** During testing,
   increasing a circle's radius from 2m to 20m while keeping a fixed
   8-waypoint count degraded path shape significantly (a rounded polygon
   instead of a circle) — increasing to 30 waypoints fixed it. The
   robust fix is to make `build_helix()`'s waypoint count scale with the
   path's arc length automatically, rather than a fixed constant. This
   was identified but not implemented.
4. **Obstacle-in-Gazebo test, before touching JPS3D.** Place one static
   obstacle at a known position in the Gazebo world, hand-write 3-4
   waypoints in a copy of `mission_trajectory.py` that route around it,
   validate the flown path keeps real clearance. This proves the
   "obstacle + smooth trajectory" combination works before adding an
   automated planner on top of it.
5. **Build JPS3D against a static voxel map**, entirely offline — no
   ROS/Gazebo needed for this step, per the original project scope.
   Obstacles and waypoints are assumed known in advance (this project
   deliberately does not target live/real-time obstacle perception).
6. **Wire JPS3D's output into GCOPTER**, replacing the hand-typed
   waypoints in the mission script with the planner's automatically
   generated path.

## 5. Validation methodology — reuse this, it's the project's real asset

`analyze_trajectory.py` and `compare_trajectories.py` are the accumulated
result of several rounds of fixing real bugs in the analysis itself (not
just the flight code) — an accel-proxy index-misalignment bug, a
worst-case/best-case miss-distance inversion, a crash on incomplete
traces, and a waypoint-matching blind spot that ignored horizontal
position entirely and only checked altitude. Each was caught by writing a
small reproducible test case, not by inspection alone. If extending these
scripts, keep that habit — write a synthetic test for the new logic
before trusting its output on real flight data.

## 6. A note on process

Large parts of this project's debugging were done with AI assistance
(Claude, and separately GitHub Copilot). Both tools produced genuinely
correct, load-bearing findings and genuinely wrong or overstated ones,
roughly in equal measure, over the course of the project — including
several cases where one tool's claim was checked against actual source
code or a reproducible test and found to be incorrect. The discipline
that made this workable was: **never accept a claim about the code
without either reading the actual source it refers to, or running a
reproducible test against real data.** Whoever continues this work should
keep that habit — several real, load-bearing bugs in this codebase were
only caught that way, and at least one confidently-stated "fix" (a
plugin_denylist override that would have silently disabled ArduPilot
safety-relevant MAVROS plugins) was caught and reverted before it caused
real harm specifically because it was checked against the actual override
semantics rather than trusted.
