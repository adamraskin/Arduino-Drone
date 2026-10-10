# Project Status

Last update: 2026-10-09

Read this file first when you start work again.

## Where the work stopped

| Step | Status |
|---|---|
| 1. Read the old code and find problems | Done |
| 2. Design flight control, failsafe, fast loop | Done. Approved in chat. |
| 3. Restructure the repo | Done |
| 4. Write the design spec | Done: [flight-control-design.md](flight-control-design.md) |
| 5. User reviews the spec | **Next step** |
| 6. Add a task list to the spec (no separate plan tool) | Not started |
| 7. Write the code (controller and receiver) | Not started |
| 8. Compile both sketches | Not started |
| 9. Write `docs/wiring.md`, `docs/flying.md`, `docs/protocol.md`, full README | Not started |
| 10. Bench checks (propellers removed) | Not started. User does this. |

The files in `firmware/` are the OLD code. They were moved and renamed only. They do not have the new design yet.

## How to start again

1. Read the spec. Make a list of changes, if necessary.
2. Answer the open items below (you can answer them later, but item 1 is necessary before first power-on).
3. Tell Claude: "Read docs/STATUS.md and continue."

## Open items for the user

| # | Item | Why | When |
|---|---|---|---|
| 1 | Exact IMU board model. You think it is an "11DOF" board. The code will use the Adafruit 10DOF until you confirm. | Sets the sensor driver | Before first power-on |
| 2 | Motor spin directions, and which motor is on which pin | A wrong sign makes the drone flip | Bench check, propellers removed |
| 3 | How the IMU board is installed (which direction it points) | Sets the axis mapping | Bench check |
| 4 | Right stick wiring. The plan uses A2 (pitch) and A3 (roll). | Pins in `controller.ino` | When you wire it |

## Decisions made in chat

- Controls: two sticks, Mode 2. Left stick = throttle and yaw. Right stick = pitch and roll.
- The throttle stick goes back to center. Center to top = throttle 0 to 1000.
- Arming: DJI-style. Both sticks down and inward for 1 s, with the drone level. The same command disarms.
- Drop arm: one button press moves it down. The next press moves it up.
- Flight mode: self-level (angle) mode, one PID for each axis, maximum tilt 20 degrees.
- ESC output: keep the `Servo` library (50 Hz). No rewiring. All motor writes go through one function.
- Altitude: sensors are read only. Altitude hold is the next feature after this work.
- Debug print and serial tuning: each has a compile-time switch, so you can remove them to make the loop faster.
- Library zips: keep all 12, unchanged, in `libraries/`.

## Work method

- Do not use the superpowers skills or their folders (`docs/superpowers/`). Keep all plans, specs, and status in plain `.md` files in `docs/`.
- Do not run git commands. The user does all git operations.

## Tools on this PC

- `arduino-cli` comes with the Arduino IDE:
  `C:\Users\Adam\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`
- The AVR core and the necessary libraries are installed (`Documents\Arduino\libraries`).
- There is no host C++ compiler (`g++`). Logic tests on the PC are not possible. Compile checks and bench checks are used.
