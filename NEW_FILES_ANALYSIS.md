# NEW FILES CREATED FOR ArduPilot INTEGRATION
## Why They Were Necessary & Better Than Modifying Existing PX4 Files

---

## **FILE 1: `autopilot_profile.hpp` + `autopilot_profile.cpp`**

### **What It Does:**
Encapsulates **all firmware-specific differences** between ArduPilot and PX4 into a **single abstraction layer**.

**Key Decisions It Captures:**
```cpp
enum AutopilotProfile {
  family: ARDUPILOT | PX4
  offboard_mode: "GUIDED" (ArduPilot) vs "OFFBOARD" (PX4)
  manual_mode: "LOITER" (ArduPilot) vs "POSCTL" (PX4)
  hold_mode: "BRAKE" (ArduPilot) vs "AUTO.LOITER" (PX4)
  supports_acceleration_setpoints: true | true
  supports_body_rate_setpoints: false (APM) | true (PX4)
  requires_takeoff_command: true (APM) | false (PX4)
  set_mode_before_arming: true (APM) | false (PX4)
  attitude_thrust: CLIMB_RATE (APM) | NORMALIZED_THRUST (PX4)
}
```

### **Why NEW File Instead of Modifying PX4 Code:**

| Approach | Consequence |
|----------|------------|
| **Modify existing PX4 handling code** | PX4 logic would be scattered with ArduPilot `if/else` blocks → **unmaintainable** |
| **Add enum to existing class** | Mixed concerns; firmware logic leaks into platform logic → **architectural violation** |
| **Create NEW abstraction** ✅ | **Firmware agnostic**; platform code stays clean; easy to add PX4 SUB-VARIANTS (e.g., Pixhawk 4 vs Pixhawk Mini) |

### **Example Why This Matters:**
```cpp
// BAD (in original platform code):
if (autopilot == "px4") {
  mode = "OFFBOARD";
  supports_body_rates = true;
  thrust_is_climb_rate = false;
} else if (autopilot == "ardupilot") {
  mode = "GUIDED";
  supports_body_rates = false;
  thrust_is_climb_rate = true;
}
// ... repeated 20+ times throughout mavlink_platform.cpp

// GOOD (with new abstraction):
AutopilotProfile profile;
AutopilotProfile::create(autopilot_name, profile);  // One call
String mode = profile.offboard_mode;  // Clean access
```

**Better Design Because:**
- ✅ Single source of truth for each autopilot's quirks
- ✅ Solves **issue #1**: `mode == "OFFBOARD"` hardcoded, causing "arms but doesn't move" bug
- ✅ Easy to extend: adding PX4 VTOL just needs new enum value
- ✅ Testable in isolation (C++ unit tests only this file)

---

## **FILE 2: `world.yaml`**

### **What It Does:**
YAML configuration for **ArduPilot SITL + Gazebo setup**

**Contents:**
```yaml
world: "iris_runway"  # Gazebo world model
vehicle: "ArduCopter"  # ArduPilot binary
frame: "gazebo-iris"   # SITL frame type
drones:               # Per-drone config (multi-drone support)
  - namespace: "drone0"
    instance: 0       # sim_vehicle.py instance (-I flag)
    sysid: 1          # MAVLink system ID
    mavros_out: "udp:127.0.0.1:14551"  # MAVROS connection
```

### **Why NEW File Instead of Modifying PX4 Config:**

**Original PX4 Setup:**
```
sitl_config/docker/
  docker_build.bash     # Build Docker image
  docker_run.bash       # Start Docker container
  run_instance.py       # Launch PX4 inside container
```

| Problem with PX4 Docker approach | ArduPilot SITL approach |
|---|---|
| **Docker overhead** — extra process, resource hog | **Native binary** — `sim_vehicle.py` runs locally, minimal overhead |
| **Monolithic container** — one image = all config | **Modular YAML** — change world/vehicle without rebuild |
| **Complex to extend** — Dockerfile edits needed | **Simple to extend** — add drone entry, auto-generates instance ports |
| **Hard to debug** — SITL runs inside black box | **Easy to debug** — see `sim_vehicle.py` output directly, MAVProxy console |

**Key Insight from Ardu_pilot.md:**
> "Instance N uses the Gazebo FDM ports 9002+10N / 9003+10N and the MAVProxy master port 5760+10N, so the instances never collide."

YAML-driven config **automatically handles multi-drone port management** — something the PX4 Docker setup didn't do well.

---

## **FILE 3: `launch_gazebo.bash`**

### **What It Does:**
Launches Gazebo with the ArduPilot Gazebo plugin (`ardupilot_gazebo`)

```bash
WORLD=$(python3 -c "import yaml,sys; \
        print(yaml.safe_load(open('world.yaml'))['world'])")
exec gz sim -v4 -r "${WORLD}.sdf"
```

### **Why NEW File Instead of Modifying PX4 Launch:**

**PX4 Approach:**
```bash
# docker_run.bash called:
# - Docker container startup
# - PX4 SITL inside container
# - Gazebo inside container
# All coupled in a single Docker orchestration
```

**ArduPilot Approach (NEW):**
```bash
# Gazebo runs on host
# sim_vehicle.py (ArduPilot SITL) connects via UDP socket (FDM port 9002)
# MAVROS connects via UDP socket (MAVProxy port 14551)
# Everything is independent, simpler debugging
```

**Why It's Better:**
1. **Decoupling** — Gazebo can restart without killing SITL
2. **Performance** — GPU rendering on host, not in container
3. **Debugging** — Can pause Gazebo while SITL continues; inspect MAVLink in MAVProxy
4. **Environment vars** — Relies on `GZ_SIM_SYSTEM_PLUGIN_PATH` (user setup once), not Docker volumes

---

## **FILE 4: `run_instance.py`**

### **What It Does:**
Spawns a **single ArduPilot SITL instance** with correct parameters

**Reads `world.yaml` and generates:**
```bash
sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --model JSON \
  -I 0 \
  --sysid 1 \
  --out udp:127.0.0.1:14551 \
  --console
```

Each drone gets unique:
- **Instance ID** (`-I`) → FDM port offset
- **System ID** (`--sysid`) → MAVLink identifier
- **Output port** (`--out`) → MAVROS listening port

### **Why NEW File Instead of Modifying PX4:**

**PX4 Approach:**
```bash
# docker_run.bash: hard-coded single instance inside container
# To add drone1: edit Dockerfile, rebuild, run again
# For 3 drones: 3 Docker containers = 3x the overhead
```

**ArduPilot Approach (NEW):**
```bash
run_instance.py -n drone0 -p world.yaml  # Terminal 1
run_instance.py -n drone1 -p world.yaml  # Terminal 2
run_instance.py -n drone2 -p world.yaml  # Terminal 3
# Each gets unique ports, all independent processes
```

**Why It's Better:**
1. **Multi-drone ready** — spawn 10 drones with 10 commands
2. **YAML-driven** — no code changes, just update `world.yaml`
3. **Fail-independently** — drone1 crash doesn't kill drone0
4. **Fast iteration** — restart one drone in seconds, not rebuild Docker

---

## **FILE 5: `mission_trajectory.py`**

### **What It Does:**
**Demonstrates MINCO smooth trajectory planning** your project needs

```python
def build_helix(radius, height, amplitude, samples):
    # Generates smooth 3D waypoint path
    
drone_run():
    path = build_helix(RADIUS=2.0, HEIGHT=1.0, AMPLITUDE=0.5, samples=8)
    drone_interface.follow_path.follow_path_with_path_facing(path, speed=SPEED)
```

**Flow:**
```
Waypoints (8 points on helix)
  ↓
TrajectoryGeneratorBehavior (jerk_limited_trajectory_generator plugin)
  ↓ [Generates MINCO curves]
  ↓ (smooth pos + vel + accel respecting max_jerk = 5.0 m/s³)
ArduPilot Copter controller
  ↓
Smooth flight
```

### **Why NEW File Instead of Modifying PX4 Missions:**

| PX4 Mission | ArduPilot Trajectory Mission |
|---|---|
| `mission.py` — simple go-to waypoints | `mission_trajectory.py` — MINCO smooth paths |
| Relies on motion controller | Relies on platform's TRAJECTORY mode (new) |
| Duplicates PID loops | Uses ArduPilot's native controller |
| Limited to `max_speed` | Can specify `max_acceleration`, `max_jerk` |

**Key Difference from `mission.py`:**
```python
# mission.py (basic):
drone_interface.go_to.go_to_point(goal, speed=SPEED)

# mission_trajectory.py (smooth):
drone_interface.follow_path.follow_path_with_path_facing(path, speed=SPEED)
# ^ This triggers TrajectoryGeneratorBehavior → MINCO algorithm
```

**Why It Matters for Your Project:**
- ✅ Solves **issue #5** from Ardu_pilot.md: "No TRAJECTORY mode"
- ✅ You wanted MINCO smooth trajectories — this is how they're exposed
- ✅ PX4 config had no jerk limiting; ArduPilot config does (lines 80-84 of config.yaml)

---

## **SUMMARY TABLE: NEW vs MODIFIED FILES**

| File | Type | Purpose | Why NEW > MODIFY |
|------|------|---------|-----------------|
| **autopilot_profile.{hpp,cpp}** | Core | Firmware abstraction | Keeps platform code clean; extensible |
| **world.yaml** | Config | SITL drone descriptions | Modular, auto-scales to multi-drone |
| **launch_gazebo.bash** | Script | Start Gazebo | Simpler, better debugging, no Docker |
| **run_instance.py** | Script | Spawn SITL instances | Declarative (YAML-driven), flexible |
| **mission_trajectory.py** | Mission | Smooth trajectories | Enables MINCO use case; ArduPilot-specific |

---

## **KEY INSIGHT: Why This Design is Better**

Original PX4 design:
```
as2_platform_mavlink (PX4 hardcoded)
  ↓
Docker container (PX4 SITL + Gazebo coupled)
  ↓
Single monolithic process
```

New ArduPilot design:
```
as2_platform_mavlink (autopilot-agnostic via AutopilotProfile)
  ↓
Native processes (Gazebo + sim_vehicle.py independent)
  ↓
YAML-configured, easily extensible
```

**This is why Aerostack2 chose this approach:**
- ✅ **Decoupling** — each component fails independently
- ✅ **Clarity** — no hidden Docker logic
- ✅ **Extensibility** — add Pixhawk Mini variant = add 2 lines to profile
- ✅ **Performance** — no container overhead; full GPU for Gazebo
- ✅ **Debugging** — can inspect MAVLink packets live in MAVProxy

