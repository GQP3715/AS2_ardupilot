# ArduPilot AS2 Integration — Code Bugs Report

**Date:** 2026-08-13  
**Project:** ArduPilot + Aerostack2 (Single-Drone MINCO Trajectory Planning)  
**Status:** 4 bugs identified, ready for fix

---

## Quick Reference

| # | File | Bug Type | Lines | Severity | Fix Priority |
|---|------|----------|-------|----------|---|
| 1 | `mission.py` | Success flag overwritten | 171-174 | 🔴 HIGH | ⭐⭐⭐ CRITICAL |
| 2 | `mission_gps.py` | Success flag overwritten | 172-175 | 🔴 HIGH | ⭐⭐⭐ CRITICAL |
| 3 | `mission_interpreter.py` | Missing `argparse` import | ~36-65 | 🔴 HIGH | ⭐⭐⭐ CRITICAL |
| 4 | `mission_swarm.py` | Infinite loop + arg parse | 162-169, 237-238 | 🟡 MEDIUM | ⭐ SKIP (not your use case) |

---

## BUG #1: mission.py — Success Flag Overwrite

**Location:** `src/project_mavlink/mission.py`, lines 171-174

### The Problem

```python
171:    success = drone_start(uav)
172:    if success:
173:        success = drone_run(uav)
174:    success = drone_end(uav)    # ❌ BUG: Always overwrites previous result
```

### What Goes Wrong

Line 174 **unconditionally overwrites** the `success` variable with the result of `drone_end()`, regardless of whether previous stages failed.

**Failure Scenario:**
```
Stage 1: drone_start() = True  ✓ (offboard successful)
Stage 2: drone_run()   = False ✗ (mission failed)
  → success = False at line 173
Stage 3: drone_end()   = True  ✓ (landing successful)
  → success = True at line 174 (OVERWRITES the False!)

Final Result: success = True  ❌ WRONG!
```

### Expected Behavior

The mission should only succeed if **ALL stages succeed**:
- Start fails → report failure
- Run fails → report failure (even if land succeeds)
- End fails → report failure

### The Fix

```python
success = drone_start(uav)
if success:
    success = drone_run(uav)
if success:  # Only attempt end if previous stages succeeded
    success = drone_end(uav)
```

Or more elegantly:

```python
success = drone_start(uav) and drone_run(uav) and drone_end(uav)
```

### Code to Replace

**Before:**
```python
    success = drone_start(uav)
    if success:
        success = drone_run(uav)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
```

**After:**
```python
    success = drone_start(uav) and drone_run(uav)
    success = success and drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
```

---

## BUG #2: mission_gps.py — Success Flag Overwrite (IDENTICAL)

**Location:** `src/project_mavlink/mission_gps.py`, lines 172-175

### The Problem

Exact same bug as mission.py:

```python
172:    success = drone_start(uav)
173:    if success:
174:        success = drone_run(uav)
175:    success = drone_end(uav)    # ❌ BUG: Always overwrites previous result
```

### The Fix

**Before:**
```python
    success = drone_start(uav)
    if success:
        success = drone_run(uav)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
```

**After:**
```python
    success = drone_start(uav) and drone_run(uav)
    success = success and drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
```

---

## BUG #3: mission_interpreter.py — Missing Import Statement

**Location:** `src/project_mavlink/mission_interpreter.py`, top of file (before line 61)

### The Problem

The file uses `argparse.ArgumentParser()` but **never imports `argparse`**.

**File starts at line 1:**
```python
1:  #!/usr/bin/env python3
2:
3:  # Copyright 2024 Universidad Politécnica de Madrid
...
36: import argparse   # ❌ MISSING!
37: import json
38:
39: import rclpy
40:
41: from as2_python_api.mission_interpreter.mission import Mission
42: from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
```

**Then at line 61+:**
```python
61: if __name__ == '__main__':
62:     parser = argparse.ArgumentParser(...)  # ❌ argparse not defined!
```

### When It Fails

At runtime, Python will crash:
```
Traceback (most recent call last):
  File "mission_interpreter.py", line 62, in <module>
    parser = argparse.ArgumentParser(...)
NameError: name 'argparse' is not defined
```

### The Fix

Add the missing import at the top of the file.

**Current imports:**
```python
import argparse   # ← Line currently MISSING
import json

import rclpy

from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
```

**What to add:**

Add this line after `import json` (or before `import rclpy`):

```python
import argparse
```

### Complete Fix Context

**Before (lines 33-45):**
```python
__authors__ = 'Rafael Perez-Segui, Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse   # ← MISSING LINE
import json

import rclpy

from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
```

**After (corrected):**
```python
__authors__ = 'Rafael Perez-Segui, Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import json

import rclpy

from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
```

---

## BUG #4: mission_swarm.py — Infinite Loop (Optional Fix)

**⚠️ NOTE:** This file is for **swarm missions**. You're doing **single-drone MINCO planning**, so you can skip this. Included for completeness.

### The Problem #4a: Infinite Loop in `wait()` Method

**Location:** `src/project_mavlink/mission_swarm.py`, lines 162-169

```python
162:    def wait(self):
163:        """Wait until all drones has reached their goal (aka finished its behavior)"""
164:        all_finished = False
165:        while not all_finished:        # ❌ NO TIMEOUT = INFINITE LOOP RISK
166:            all_finished = True
167:            for drone in self.drones.values():
168:                all_finished = all_finished and drone.goal_reached()
```

### What Goes Wrong

If any drone's behavior **never completes** (gets stuck in intermediate state):
- `drone.goal_reached()` always returns False
- `all_finished` stays False
- `while` loop runs forever
- **Entire swarm mission hangs indefinitely**
- No error message, no exception — just freezes

**When this happens:**
```python
# Drone 0 reaches goal → goal_reached() = True
# Drone 1 STUCK in motion  → goal_reached() = False
# → all_finished = False and continues
# → while loop cycles forever
# → program hangs forever (must kill with Ctrl+C)
```

### The Fix

Add a **timeout mechanism**:

```python
def wait(self, timeout: float = 60.0):
    """Wait until all drones has reached their goal with timeout protection"""
    import time
    start_time = time.time()
    all_finished = False
    while not all_finished:
        if time.time() - start_time > timeout:
            raise TimeoutError(f"Swarm wait timeout after {timeout}s")
        
        all_finished = True
        for drone in self.drones.values():
            all_finished = all_finished and drone.goal_reached()
        time.sleep(0.1)  # Prevent busy-wait
```

---

### The Problem #4b: Argument Parsing Error

**Location:** `src/project_mavlink/mission_swarm.py`, line 237-238

```python
237:    parser.add_argument('-n', '--namespaces',
238:                        type=list,    # ❌ WRONG: doesn't parse comma-separated values
239:                        default=['drone0', 'drone1', 'drone2'],
```

### What Goes Wrong

`type=list` **doesn't work as expected** in argparse.

If user runs:
```bash
python3 mission_swarm.py -n drone0,drone1,drone2
```

They expect: `['drone0', 'drone1', 'drone2']`

But actually get: `['d', 'r', 'o', 'n', 'e', '0', ',', 'd', 'r', 'o', 'n', 'e', '1', ',', 'd', 'r', 'o', 'n', 'e', '2']`

(list of individual characters!)

### The Fix

Option A: Use `nargs='+'` for multiple values:
```python
parser.add_argument('-n', '--namespaces',
                    nargs='+',  # Accept multiple args
                    default=['drone0', 'drone1', 'drone2'],
                    help='Drone namespaces (space-separated)')
```

Usage:
```bash
python3 mission_swarm.py -n drone0 drone1 drone2
```

Option B: Keep `type=str` and split manually:
```python
parser.add_argument('-n', '--namespaces',
                    type=str,
                    default='drone0,drone1,drone2',
                    help='Comma-separated drone namespaces')

# Then in code:
drones_namespace = args.namespaces.split(',')
```

Usage:
```bash
python3 mission_swarm.py -n drone0,drone1,drone2
```

---

## Summary: What to Fix for Your Project

### Minimum (Single-Drone, Your Use Case)

Fix these 3 files:

1. ✅ **mission.py** — Lines 171-174: Fix success flag logic
2. ✅ **mission_gps.py** — Lines 172-175: Fix success flag logic
3. ✅ **mission_interpreter.py** — Top: Add `import argparse`

### Optional (If Testing Swarm Later)

4. ⏭️ **mission_swarm.py** — Lines 162-169: Add timeout to `wait()`
5. ⏭️ **mission_swarm.py** — Lines 237-238: Fix argument parsing

---

## Code Snippets You Can Copy

### Fix #1-2: mission.py and mission_gps.py (IDENTICAL)

```python
# BEFORE:
success = drone_start(uav)
if success:
    success = drone_run(uav)
success = drone_end(uav)

# AFTER:
success = drone_start(uav) and drone_run(uav)
success = success and drone_end(uav)
```

### Fix #3: mission_interpreter.py

```python
# Add this line at the top with other imports:
import argparse
```

---

## Verification Commands

After fixing, verify there are no syntax errors:

```bash
python3 -m py_compile src/project_mavlink/mission.py
python3 -m py_compile src/project_mavlink/mission_gps.py
python3 -m py_compile src/project_mavlink/mission_interpreter.py
# Should produce no output if correct
```

Or run with verbose check:
```bash
python3 -m py_compile -v src/project_mavlink/mission.py
# Should print: Compiling 'src/project_mavlink/mission.py'...
```

---

## Next Steps

1. Apply fixes to mission.py, mission_gps.py, mission_interpreter.py
2. Verify syntax with `python3 -m py_compile`
3. Commit to git:
   ```bash
   git add -A
   git commit -m "Fix: mission script bugs (success tracking, argparse import)"
   git push
   ```
4. Ready for SITL build and test!

---

**Document Generated:** 2026-08-13  
**Ready to Copy:** ✅ Yes

