# Hardware Validation Run-Log: SO-101 Follower Arm

This document is the **hardware-gated "done" criterion** for the three
hardware verbs (`find-port`, `setup-motors`, `calibrate`). CI validates
the command surface, contracts, profile round-trips, and port enumeration
logic with no physical hardware. That is necessary but **not sufficient**
for a release. The feature is not releasable until a human operator with a
real SO-101 follower arm completes this procedure and signs off a run-log.

## Prerequisites

Before starting:

- **Hardware:** SO-101 follower arm with USB-to-serial adapter connected to a
  Linux host.
- **Power:** Arm motors powered (Feetech STS3215 bus powered via the arm's
  power input, not USB).
- **Permissions:** The operator's user account must be in the `dialout`
  group (or equivalent) to access `/dev/ttyACM*` without `sudo`.

  ```bash
  # Check and add if missing (requires logout/login to take effect)
  groups $USER
  sudo usermod -aG dialout $USER
  ```

- **Hardware extras installed:** The `.[hardware]` optional dependency group
  must be present. Without it, all three verbs raise `CliError` exit 2.

  ```bash
  # pip workflow
  pip install '.[hardware]'

  # uv workflow
  uv sync --extra hardware
  ```

- **Terminal (TTY):** `setup-motors` and `find-port --detect` both require
  an interactive terminal. Run these from a real shell, not a pipe or a
  CI runner.

---

## Procedure

Steps are ordered so port discovery comes first, EEPROM assignment second
(needed only for fresh/factory motors), and calibration third. If your
motors already have their correct ids assigned, skip Section 2.

### 1. Verify the Serial Port — `find-port`

**Purpose:** Confirm that the CLI can enumerate and correctly identify the
arm's serial port before any motor communication is attempted.

#### Step 1.1 — Non-interactive enumeration

With the arm connected and powered:

```bash
arm101 find-port
```

**Expected result:** One or more `/dev/ttyACM*` (or `/dev/ttyUSB*`) paths
printed to stdout, one per line. The arm's port should appear in the list.

#### Step 1.2 — Interactive detect (cross-check)

```bash
arm101 find-port --detect
```

The CLI prints to stderr:

```text
Unplug the arm USB cable, then press Enter...
```

Unplug only the arm's USB cable, then press Enter.

**Expected result:** Exactly one `/dev/` path printed to stdout — the path
that disappeared when the cable was unplugged. This must match the path
observed in Step 1.1.

Note: `--detect` requires an interactive TTY. Do not run through a pipe.

---

### 2. Assign Motor IDs and Baudrate — `setup-motors`

**Purpose:** Write the correct EEPROM id and baudrate to each motor. Run
this only against **fresh or factory-reset motors**. Running it against
already-configured motors that may have non-default ids on the bus
simultaneously can result in address collisions.

**Safety note:** This command writes to EEPROM (persistent non-volatile
storage). Each write is gated on an operator keypress. Connect exactly one
motor at a time as prompted. Do not connect two motors simultaneously during
this procedure.

The command walks motor ids from 6 (gripper) down to 1 (shoulder\_pan).

With the arm's USB cable connected (port identified in Step 1):

```bash
arm101 setup-motors --port /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the port identified in Section 1.

For each motor the CLI prints to stderr:

```text
connect the <joint_name> motor (id <N>) only, then press Enter
```

Follow the sequence:

| Step | CLI prompt | Action |
|------|-----------|--------|
| 2.1 | `connect the gripper motor (id 6) only, then press Enter` | Disconnect all motors. Connect only the gripper motor. Press Enter. |
| 2.2 | `connect the wrist_roll motor (id 5) only, then press Enter` | Disconnect gripper. Connect only wrist\_roll motor. Press Enter. |
| 2.3 | `connect the wrist_flex motor (id 4) only, then press Enter` | Disconnect previous. Connect only wrist\_flex motor. Press Enter. |
| 2.4 | `connect the elbow_flex motor (id 3) only, then press Enter` | Disconnect previous. Connect only elbow\_flex motor. Press Enter. |
| 2.5 | `connect the shoulder_lift motor (id 2) only, then press Enter` | Disconnect previous. Connect only shoulder\_lift motor. Press Enter. |
| 2.6 | `connect the shoulder_pan motor (id 1) only, then press Enter` | Disconnect previous. Connect only shoulder\_pan motor. Press Enter. |

**Expected result on stdout after all 6 motors:**

```text
Motors assigned:
  gripper (motor 6): id=6, baudrate=1000000
  wrist_roll (motor 5): id=5, baudrate=1000000
  wrist_flex (motor 4): id=4, baudrate=1000000
  elbow_flex (motor 3): id=3, baudrate=1000000
  shoulder_lift (motor 2): id=2, baudrate=1000000
  shoulder_pan (motor 1): id=1, baudrate=1000000
```

Reconnect all motors to the bus before proceeding to calibration.

---

### 3. Capture Calibration Profile — `calibrate`

**Purpose:** Read raw STS3215 encoder ticks (range 0–4095) for all 6
joints at three arm poses (centred/rest, minimum, maximum), compute
per-joint min/mid/max, and persist the profile to:

```text
$XDG_CONFIG_HOME/arm101/calibrations/<id>.json
# or if XDG_CONFIG_HOME is unset:
~/.config/arm101/calibrations/<id>.json
```

**Safety note:** During calibration you move each joint to its physical
limits. Move gently to approach (but not crash into) hard stops. Do not
force a joint past resistance.

With all 6 motors connected on the bus and the port identified in Section 1:

```bash
arm101 calibrate my-arm --port /dev/ttyACM0
```

Replace `my-arm` with your chosen arm identifier (used as the profile
filename) and `/dev/ttyACM0` with the correct port.

The CLI walks you through three poses:

#### Pose 1 — Centred/rest (mid readings)

The CLI prints:

```text
Move arm to centered/rest pose, then press Enter...
```

Place the arm in a neutral, upright rest pose. Press Enter.

#### Pose 2 — Minimum/fully-closed (min readings)

The CLI prints:

```text
Move arm to MINIMUM/fully-closed position, then press Enter...
```

Move all joints toward their minimum extent (arm folded, gripper closed).
Press Enter.

#### Pose 3 — Maximum/fully-open (max readings)

The CLI prints:

```text
Move arm to MAXIMUM/fully-open position, then press Enter...
```

Move all joints toward their maximum extent (arm fully extended, gripper
open). Press Enter.

**Expected result on stdout:**

```text
Calibration saved: /home/<user>/.config/arm101/calibrations/my-arm.json

Joint            min    mid    max
--------------------------------------
shoulder_pan       X      Y      Z
shoulder_lift      X      Y      Z
elbow_flex         X      Y      Z
wrist_flex         X      Y      Z
wrist_roll         X      Y      Z
gripper            X      Y      Z
```

All six joints must have distinct min/mid/max values (not all identical).
Each value must be in the range 0–4095. The min must be less than or equal
to mid, and mid must be less than or equal to max.

**Round-trip check:** After calibration, verify the file was written:

```bash
cat ~/.config/arm101/calibrations/my-arm.json
```

The JSON must contain a `"joints"` key with all six joint names and
per-joint `min`, `mid`, `max` integer fields.

---

## Run-Log Template

Complete one row per verb per validation session. A validation is complete
only when all three verbs have a "Pass" entry.

### Run-Log Table

| Date | Operator | Arm ID | Serial Port | Verb | Command Run | Observed Result | Pass/Fail |
|------|----------|--------|-------------|------|-------------|-----------------|-----------|
| | | | | `find-port` | `arm101 find-port` | Ports listed: | |
| | | | | `find-port --detect` | `arm101 find-port --detect` | Port resolved: | |
| | | | | `setup-motors` | `arm101 setup-motors --port <PORT>` | All 6 motors assigned (list ids/baudrates): | |
| | | | | `calibrate` | `arm101 calibrate <ID> --port <PORT>` | Profile path, per-joint min/mid/max (paste table): | |

### Extended Notes

```text
Date:
Operator name/contact:
Arm serial number or label:
USB adapter model:
Linux distro + kernel version:
Python version:
arm101-cli version (arm101 whoami):
Hardware extras version (pip show feetech-servo-sdk):

find-port result:
  Enumerated ports: 
  Detected port (--detect): 

setup-motors result:
  Completed: yes/no
  Any errors: 

calibrate result:
  Profile saved at: 
  shoulder_pan:   min=     mid=     max=
  shoulder_lift:  min=     mid=     max=
  elbow_flex:     min=     mid=     max=
  wrist_flex:     min=     mid=     max=
  wrist_roll:     min=     mid=     max=
  gripper:        min=     mid=     max=
  Round-trip JSON check passed: yes/no

Overall outcome: PASS / FAIL
```

---

## "Done" Gate Statement

The hardware verbs (`find-port`, `calibrate`, `setup-motors`) are **not
releasable** until:

1. CI is green on the no-hardware surface (contract tests, profile
   round-trips, port enumeration logic, lockstep checks).
2. This run-log records a successful end-to-end execution of all three
   verbs against a physical SO-101 follower arm, with a human operator
   filling in the observed results and signing off Pass.

Criterion 1 alone is not sufficient. A green CI pipeline confirms the
command surface and contracts are correct; it cannot confirm that real motor
I/O works. Criterion 2 is the manual gate that closes the gap.

---

_Procedure authored by arm101-cli (Claude); run-log entries to be filled by the operator._
