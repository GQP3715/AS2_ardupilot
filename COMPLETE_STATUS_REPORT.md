# ArduPilot AS2 Integration — Complete Status Report

**Date:** 2026-08-14  
**Project:** Single-Drone MINCO Trajectory Planning with Aerostack2 + ArduPilot  
**Status:** ✅ Configuration Ready | ⏳ Code Fixes Pending

---

## Executive Summary

### ✅ Configuration Phase — COMPLETE

All architecture and configuration issues have been **identified, verified, and corrected**:

1. ✅ **Control modes** — Correct for ArduPilot (UNSET/HOVER/SPEED/POSITION/TRAJECTORY)
2. ✅ **Dead parameter removed** — `use_bypass` deleted from config.yaml
3. ✅ **MAVROS plugins** — Official denylist being used (corrected earlier mistake)
4. ✅ **Dependencies** — `mavros_extras` appropriate and verified
5. ✅ **Frame handling** — ENU-based, verified against ArduPilot docs

### ⏳ Code Phase — PENDING (3 Fixes Remaining)

Three mission script bugs need fixing before SITL testing:

1. ⏳ **mission.py** (line 174) — Success flag overwrite
2. ⏳ **mission_gps.py** (line 175) — Success flag overwrite
3. ⏳ **mission_interpreter.py** (top) — Missing `argparse` import

---

## Detailed Status by Component

### A. Aerostack2 Core Configuration

| Component | File | Status | Details |
|-----------|------|--------|---------|
| **Control Modes** | `control_modes.yaml` | ✅ CORRECT | UNSET, HOVER, SPEED, POSITION, TRAJECTORY (no ACRO/ATTITUDE) |
| **Autopilot Profile** | `autopilot_profile.cpp` | ✅ VERIFIED | Correctly defines ArduPilot GUIDED mode, CLIMB_RATE thrust, requirements |
| **Motion Controller Bypass** | `config.yaml` | ✅ FIXED | Removed dead `use_bypass: true` parameter |

### B. MAVROS Integration

| Component | File | Status | Details |
|-----------|------|--------|---------|
| **Plugin Denylist** | `/opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml` | ✅ VERIFIED | Official file loaded automatically; covers debug_value, px4flow, vision_speed_estimate, wheel_odometry |
| **Project Overrides** | `project_mavlink/config/` | ✅ CLEAN | No project-level plugin configuration (correct behavior) |
| **MAVROS Package** | `package.xml` | ✅ VERIFIED | `mavros_extras` dependency is appropriate |

### C. Launch Configuration

| Component | File | Status | Details |
|-----------|------|--------|---------|
| **MAVROS Launch** | `mavros_launch.py` | ✅ VERIFIED | Correctly loads `apm_config.yaml` + `apm_pluginlists.yaml` when `autopilot:=ardupilot` |
| **SITL Config** | `world.yaml`, `run_instance.py` | ✅ VERIFIED | Correct for native ArduPilot SITL + Gazebo |
| **UDP Port Mapping** | `config.yaml` (drone0/1/2) | ✅ VERIFIED | Correct FCU URLs and target system IDs |

### D. Mission Scripts — CODE PHASE

| Script | Bug | Severity | Status | Fix |
|--------|-----|----------|--------|-----|
| `mission.py` | Success flag overwrite (line 174) | 🔴 HIGH | ⏳ PENDING | Needs logical AND chaining |
| `mission_gps.py` | Success flag overwrite (line 175) | 🔴 HIGH | ⏳ PENDING | Needs logical AND chaining |
| `mission_interpreter.py` | Missing `argparse` import | 🔴 HIGH | ⏳ PENDING | Add import statement |
| `mission_trajectory.py` | (none found) | — | ✅ OK | No changes needed |
| `mission_swarm.py` | Infinite loop + arg parsing | 🟡 MEDIUM | ⏳ SKIP | Not required for single-drone use case |

---

## What Claude's Review Found & What We Fixed

### Issue #1: Control Modes Advertisement
- **Finding:** Verified correct — matches ArduPilot requirements
- **Status:** ✅ NO CHANGES NEEDED
- **Verification:** File reviewed and confirmed

### Issue #2: Dead Parameter
- **Finding:** `use_bypass: true` in config.yaml
- **Status:** ✅ FIXED
- **What We Did:** Removed the non-functional parameter
- **Explanation:** Bypass in AS2 is automatic via `ControllerHandler::tryToBypassController()`, not controlled by a toggle

### Issue #3: MAVROS Plugin Denylist ⚠️ CRITICAL CORRECTION
- **Finding:** I incorrectly created `apm_pluginlists.yaml` in project config
- **Status:** ✅ CORRECTED
- **What We Did:** 
  - Deleted the project-level file
  - Verified official MAVROS file is being loaded
  - Confirmed it denylists all problematic plugins
- **Why This Matters:** ROS 2 list parameters replace (not merge), so custom lists override official denylists. Using the official one is safer.

### Issue #4: Frame Handling
- **Finding:** Verified correct
- **Status:** ✅ NO CHANGES NEEDED
- **Details:** ENU input, `FRAME_LOCAL_NED` flag, no BODY frames — matches ArduPilot docs

### Issue #5: Type Masks & Setpoint Handling
- **Finding:** Verified correct
- **Status:** ✅ NO CHANGES NEEDED
- **Details:** TRAJECTORY mode uses `IGNORE_YAW_RATE` only; takeoff/land via MAV_CMD_NAV_TAKEOFF

### Issue #6: Arm Sequence
- **Finding:** Verified correct
- **Status:** ✅ NO CHANGES NEEDED
- **Details:** GUIDED entered before arm (ArduPilot safety requirement)

---

## Documents Generated

1. **[CODE_BUGS_REPORT.md](CODE_BUGS_REPORT.md)** — Detailed bug analysis with copy-paste fixes
2. **[CLAUDE_CORRECTIONS_APPLIED.md](CLAUDE_CORRECTIONS_APPLIED.md)** — Critical plugin denylist correction explanation
3. **[CLAUDE_REVIEW_VERIFICATION.md](CLAUDE_REVIEW_VERIFICATION.md)** — Full architectural review results
4. **[NEW_FILES_ANALYSIS.md](NEW_FILES_ANALYSIS.md)** — Explanation of ArduPilot-specific new files
5. **This file** — Complete status report

---

## Build Readiness Assessment

### ✅ Configuration Phase: READY
- Autopilot profile correctly defines ArduPilot behavior
- Control modes advertise TRAJECTORY mode (critical for MINCO)
- MAVROS plugins denylisted correctly (prevents SIGABRT)
- Launch parameters correct for ArduPilot SITL
- Frame transforms verified (ENU-based, no BODY frames)

### ⏳ Code Phase: PENDING FIX (3 Scripts)
- Must fix mission.py, mission_gps.py, mission_interpreter.py
- mission_trajectory.py is OK (your primary use case)
- Once fixed: Ready for `colcon build`

### 📋 Pre-Build Checklist

- ✅ Aerostack2 architecture verified
- ✅ ArduPilot integration points verified
- ✅ MAVROS configuration verified
- ✅ ROS2 protocols verified
- ⏳ **Mission code bugs must be fixed** before build
- ⏳ **Run syntax check** after fixes: `python3 -m py_compile mission.py mission_gps.py mission_interpreter.py`

---

## Next Steps (In Order)

### 1. Fix Mission Code (15 minutes)

Using [CODE_BUGS_REPORT.md](CODE_BUGS_REPORT.md):

```bash
# Fix #1: mission.py lines 171-174
# Replace:
success = drone_start(uav)
if success:
    success = drone_run(uav)
success = drone_end(uav)

# With:
success = drone_start(uav) and drone_run(uav)
success = success and drone_end(uav)

# Fix #2: mission_gps.py lines 172-175 (IDENTICAL)

# Fix #3: mission_interpreter.py top
# Add:
import argparse
```

### 2. Verify Syntax (2 minutes)

```bash
cd /root/ardupilot_as2_ws
python3 -m py_compile src/project_mavlink/mission.py
python3 -m py_compile src/project_mavlink/mission_gps.py
python3 -m py_compile src/project_mavlink/mission_interpreter.py
# Should produce no output (success)
```

### 3. Commit Changes (2 minutes)

```bash
git add -A
git commit -m "Fix: mission script bugs (success tracking, missing import)"
git push
```

### 4. Build (10-30 minutes)

```bash
cd /root/ardupilot_as2_ws
colcon build --symlink-install --packages-up-to as2_platform_mavlink
source install/setup.bash
```

### 5. SITL Testing (follows build)

Once build succeeds:
```bash
# Terminal 1: Gazebo + ArduPilot SITL
./src/project_mavlink/launch_sitl.bash

# Terminal 2: Aerostack2 + MAVROS
./src/project_mavlink/launch_as2.bash -n drone0

# Terminal 3: Your mission
python3 ./src/project_mavlink/mission_trajectory.py -n drone0
```

---

## Architecture Summary (Verified)

```
Your Mission Scripts
(mission_trajectory.py — MINCO smooth paths)
        ↓
Behaviors + TrajectoryGeneratorBehavior
(jerk_limited_trajectory_generator plugin)
        ↓
as2_motion_controller (BYPASS mode)
(forwards trajectory untouched to platform)
        ↓
as2_platform_mavlink (MavlinkPlatform)
(Platform advertises TRAJECTORY mode)
        ↓
SET_POSITION_TARGET_LOCAL_NED
(pos + vel + accel + yaw, ENU frame)
        ↓
MAVROS (converts ENU→NED)
        ↓
ArduPilot Copter GUIDED mode
(runs its own position controller)
        ↓
Gazebo (via ardupilot_gazebo plugin)
```

**Key Design Points:**
- ✅ No cascading PID loops (platform bypasses controller)
- ✅ Smooth trajectories via jerk-limited generator (MINCO-style)
- ✅ Full state setpoint (pos + vel + accel + yaw)
- ✅ Correct frame handling (ENU in, FRAME_LOCAL_NED, no BODY frames)

---

## Known Limitations (from Ardu_pilot.md)

1. **Takeoff height:** `takeoff.height` in config.yaml determines height (action goal height is ignored)
2. **Use sim time:** Keep `use_sim_time: false` (nothing publishes `/clock` in this stack)
3. **Swarm coordination:** Not part of this integration (choreography only, no collision avoidance)
4. **GPS denial fallback:** Not implemented (supports external odometry but no decision logic)

---

## Conclusion

**Architecture Phase:** ✅ **COMPLETE & VERIFIED**

All configuration, integration, and architecture-level issues have been resolved. The integration is **ready for build** once the three mission code bugs are fixed.

**Code Phase:** ⏳ **3 Fixes Remaining** (~20 minutes work)

Apply fixes from [CODE_BUGS_REPORT.md](CODE_BUGS_REPORT.md) → Verify syntax → Commit → Build

**Estimated Total Time to SITL:**
- Fixes: 20 minutes
- Build: 30 minutes
- Total: ~50 minutes to first SITL test

---

**Generated:** 2026-08-14  
**Ready:** ✅ Configuration | ⏳ Pending: Code fixes  
**Target:** SITL testing with mission_trajectory.py on drone0

