# Flight Control Design

Date: 2026-10-09
Status: Approved in chat. Spec waits for user review. No flight code is written yet.

## 1. Goal

Make the drone fly with self-level stabilization.

This work adds four items:

- A radio protocol that sends all controls in one packet.
- A radio failsafe.
- A fast main loop (250 Hz) that does not block.
- A PID flight controller in self-level (angle) mode.

Altitude hold is not in this work. The altitude sensors are read only. Altitude hold is the next step.

## 2. Hardware

| Item | Value |
|---|---|
| Drone board | Arduino Mega 2560 |
| Controller board | Arduino Uno-class board (pins 3-8, A0-A3) |
| Radio | nRF24L01, both boards |
| IMU | Adafruit 10DOF (LSM303 accel and mag, L3GD20 gyro, BMP180 baro). User must confirm the board. See section 12. |
| Height sensor | HC-SR04 ultrasonic. Used for height near the ground. |
| Motor output | 4 ESCs, driven by the Arduino `Servo` library at 50 Hz |
| Frame | Quad, X layout |
| Payload | Servo "drop arm" that drops objects |

## 3. Pins

### 3.1 Controller

| Function | Pin |
|---|---|
| Radio CE / CSN | 7 / 8 |
| Left stick, throttle axis | A0 |
| Left stick, yaw axis | A1 |
| Right stick, pitch axis | A2 (not wired yet) |
| Right stick, roll axis | A3 (not wired yet) |
| Left stick push button | 6 |
| Button 1 / 2 / 3 | 3 / 4 / 5 |

All buttons use `INPUT_PULLUP`. A pressed button reads LOW.

### 3.2 Drone (receiver)

| Function | Pin |
|---|---|
| Radio CE / CSN | 48 / 49 |
| Motor FL / FR / RL / RR | 10 / 7 / 9 / 8 |
| Drop arm servo | 6 |
| Ultrasonic trigger | 11 |
| Ultrasonic echo | 12 (pin-change interrupt PCINT6) |
| IMU | I2C (SDA 20, SCL 21) |

Put all pin numbers in named constants at the top of each sketch.

## 4. Radio protocol

### 4.1 Packet

The controller sends one packet at 50 Hz. The packet holds all controls.

```cpp
struct __attribute__((packed)) ControlPacket {
  uint8_t version;   // PROTOCOL_VERSION. The receiver discards other values.
  uint8_t seq;       // Increases by 1 for each packet.
  int16_t throttle;  // Raw stick, -500 .. +500. Center = 0. Up = +.
  int16_t yaw;       // -500 .. +500. Right = +.
  int16_t pitch;     // -500 .. +500. Forward (up) = +.
  int16_t roll;      // -500 .. +500. Right = +.
  uint8_t buttons;   // Bit 0 = stick push, bit 1..3 = button 1..3. 1 = pressed.
};
```

Size is 10 bytes. The nRF24 limit is 32 bytes.

Both sketches contain a copy of this struct. If you change the struct, change both copies. Then increase `PROTOCOL_VERSION` in both sketches.

### 4.2 Controller

1. At start, read each stick 32 times. Keep the mean as the center value.
2. Scale each axis to -500 .. +500 around its center.
3. Apply a deadband of 20 around center. Inside the deadband, send 0.
4. Each axis has an invert flag. Set the flags so that the signs agree with section 4.1.
5. Send the packet every 20 ms. Do not use `delay()`.
6. Print debug text only when `DEBUG_PRINT` is 1. Print at 2 Hz maximum.

### 4.3 Receiver

1. On each loop, read all packets that are available. Keep only the newest packet.
2. Discard a packet if `version` is not equal to `PROTOCOL_VERSION`.
3. Record the time of the last good packet.

### 4.4 Throttle mapping

The throttle stick goes back to center when you release it.

- Stick center to stick top gives throttle 0 .. 1000.
- Stick below center gives throttle 0.
- Arming uses the raw value below center. See section 5.

## 5. Arming

Arming uses a DJI-style stick command. Both sticks go down and inward.

| Stick | Position |
|---|---|
| Left stick | Down and right (throttle < -400, yaw > +400) |
| Right stick | Down and left (pitch < -400, roll < -400) |

Arm rules. All must be true for 1 s:

- The stick command is held.
- The radio link is good.
- The drone is level: absolute pitch and roll are less than 10 degrees.
- The drone is disarmed.

Disarm rules. All must be true for 1 s:

- The stick command is held.
- The drone is armed.

When the drone arms, set all PID integral terms to zero.

After arm or disarm, the stick command must be released before it can operate again.

## 6. Failsafe

The link is lost when no good packet arrives for 250 ms.

| State when link is lost | Action |
|---|---|
| Disarmed | No action. Motors stay at the minimum pulse. |
| Armed | Set pitch and roll targets to 0 (level). Set yaw rate target to 0. Decrease throttle from its last value to 0 in 3 s. Then disarm. |

If the ultrasonic height is less than 20 cm during failsafe, disarm immediately.

When the link comes back and the drone is still armed, give control back to the pilot immediately.

## 7. Main loop

### 7.1 Timing

The loop runs every 4000 us (250 Hz). Use `micros()` to start each cycle. Do not use `delay()`.

If one cycle is longer than 4000 us, increase an overrun counter. Show the counter in the debug output.

### 7.2 Fast tasks (each cycle)

1. Read the radio (section 4.3).
2. Read the gyro and the accelerometer.
3. Update the attitude filter (section 8).
4. Update arming, failsafe, and drop arm state.
5. Calculate the PIDs and the motor mix.
6. Write the motor pulses.

### 7.3 Slow tasks

| Task | Rate | Notes |
|---|---|---|
| Ultrasonic | 20 Hz | Send the trigger pulse. A pin-change interrupt on pin 12 measures the echo. Do not use `pulseIn()`. |
| Magnetometer | 10 Hz | Read heading for debug only. Yaw control does not use it. |
| Barometer | 2 Hz, disarmed only | The Adafruit BMP180 read blocks for about 30 ms. Do not read it when the drone is armed. |
| Debug print | 5 Hz | Only when `DEBUG_PRINT` is 1. One short line. |
| Serial tuning | each cycle | Only when `SERIAL_TUNING` is 1. See section 10. |

### 7.4 Startup

1. Start Serial, I2C (400 kHz), radio, and sensors.
2. Attach the motor outputs. Write 1000 us to all motors.
3. Keep the drone still for 2 s. Measure the gyro offset on each axis. Subtract it from all later gyro readings.
4. Start the main loop disarmed.

## 8. Attitude filter

Use a complementary filter for pitch and roll.

```
accelAngle = atan2(...) from the accelerometer
angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accelAngle
```

- `dt` is the measured cycle time in seconds.
- Yaw uses the gyro rate only. There is no yaw angle.
- An axis-mapping block converts the sensor axes to drone axes. The board is installed turned, so the old code swapped roll and pitch. A bench check sets the mapping. See section 11.

Sign conventions:

| Value | Positive direction |
|---|---|
| Pitch | Nose up |
| Roll | Right side down |
| Yaw rate | Nose turns right (clockwise, seen from above) |

## 9. Control

### 9.1 Targets

| Axis | Target |
|---|---|
| Pitch | `-stickPitch / 500 * 20` degrees. Stick forward makes the nose go down. |
| Roll | `stickRoll / 500 * 20` degrees |
| Yaw | `stickYaw / 500 * 120` degrees per second |

The 20 degree maximum tilt comes from [notes.md](notes.md).

### 9.2 PID

Pitch and roll (angle mode):

```
error  = target - angle
iTerm  = constrain(iTerm + Ki * error * dt, -I_LIMIT, +I_LIMIT)
output = Kp * error + iTerm - Kd * gyroRate
```

Yaw (rate mode):

```
error  = targetRate - gyroRate
output = Kp * error + iTerm
```

- Limit each output to +/- 200 us.
- Set the integral terms to zero when the drone is disarmed or when throttle is 0.
- Pitch and roll use the same gains. Yaw has its own gains.
- Start with low gains. The bench and first-flight procedure in `docs/flying.md` sets them.

### 9.3 Motor mixer

X frame. The default spin directions are FL clockwise, FR counterclockwise, RL counterclockwise, RR clockwise. Put the signs in a table at the top of `control.ino`.

| Motor | Pitch | Roll | Yaw |
|---|---|---|---|
| FL | + | + | - |
| FR | + | - | + |
| RL | - | + | + |
| RR | - | - | - |

```
base  = map(throttle, 0, 1000, 1100, 1800)
motor = base + pitchSign*pitchOut + rollSign*rollOut + yawSign*yawOut
motor = constrain(motor, 1100, 2000)
```

| State | Pulse |
|---|---|
| Disarmed | 1000 us on all motors |
| Armed, throttle 0 | 1100 us (idle, propellers turn) |
| Armed, maximum | 2000 us |

All motor writes go through one function, `writeMotors()`. A later change to fast PWM (250-400 Hz) changes only this function.

### 9.4 Drop arm

- Each press of button 3 changes the drop arm position. The positions are "down" (180) and "up" (0).
- Detect the press edge only. Holding the button does not change the position again.
- Ignore a second edge that comes less than 50 ms after the first (debounce).
- The drop arm works when the drone is armed or disarmed.

Note: the old receiver code used "button4" for the drop arm. That was the 4th bit, which is pin 5 on the controller. In the new packet, pin 5 is button 3.

## 10. Configuration and tuning

Put these switches at the top of `receiver.ino`:

```cpp
#define DEBUG_PRINT   1  // 0 = no debug text. The code is not compiled.
#define SERIAL_TUNING 1  // 0 = no tuning commands. The code is not compiled.
```

The controller has its own `DEBUG_PRINT` switch.

Tuning commands (Serial Monitor, 115200 baud, newline line ending):

| Command | Effect |
|---|---|
| `kp <value>` | Pitch and roll P gain |
| `ki <value>` | Pitch and roll I gain |
| `kd <value>` | Pitch and roll D gain |
| `ykp <value>` | Yaw P gain |
| `yki <value>` | Yaw I gain |
| `gains` | Print all gains |

The new value applies immediately. The values are not saved after power off. Write good values into the code.

## 11. Files

| File | Contents |
|---|---|
| `firmware/controller/controller.ino` | Controller sketch (one file) |
| `firmware/receiver/receiver.ino` | Config switches, pins, `setup()`, loop timing |
| `firmware/receiver/radio.ino` | Packet receive, link state |
| `firmware/receiver/imu.ino` | Sensor read, gyro offset, axis mapping, filter |
| `firmware/receiver/control.ino` | Targets, PIDs, mixer, `writeMotors()` |
| `firmware/receiver/safety.ino` | Arming, failsafe |
| `firmware/receiver/altitude.ino` | Ultrasonic (interrupt), barometer |
| `firmware/receiver/payload.ino` | Drop arm |

The Arduino IDE joins all `.ino` tabs in one folder into one sketch.

## 12. Verification

### 12.1 Compile

Compile both sketches with the `arduino-cli` that the Arduino IDE installs:

```
"C:\Users\Adam\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn arduino:avr:mega firmware/receiver
"...\arduino-cli.exe" compile --fqbn arduino:avr:uno firmware/controller
```

Both must compile with no errors.

### 12.2 Bench checks

WARNING: REMOVE ALL PROPELLERS BEFORE YOU DO THESE CHECKS.

`docs/flying.md` gives the full procedure. It includes these checks:

1. Axis signs: tilt the nose down. Pitch must become negative. Tilt the right side down. Roll must become positive.
2. Arm and disarm with the stick command. Arming must fail when the drone is not level.
3. Failsafe: arm, then switch off the controller. The motors must ramp down and disarm in 3 s.
4. Motor response: arm, then tilt the drone by hand. The motors on the low side must speed up.
5. Motor order and spin direction: each motor must match the mixer table.
6. Drop arm: one press moves it down. The next press moves it up.
7. Loop time: the overrun counter must stay at 0.

## 13. Open items (user must confirm)

| # | Item | Why | When |
|---|---|---|---|
| 1 | IMU board model ("11DOF"?) | Sets the driver in `imu.ino` | Before first power-on |
| 2 | Motor spin directions and motor-to-pin map | A wrong sign makes the drone flip | Bench check, propellers removed |
| 3 | IMU installation orientation | Sets the axis mapping | Bench check |
| 4 | Right stick wiring (A2 / A3) | Pins in `controller.ino` | When wired |

## 14. Not in this work

- Altitude hold (ultrasonic near the ground, barometer above).
- Fast ESC PWM (needs motors on 16-bit timer pins).
- Cascaded angle and rate loops.
- Gains saved in EEPROM.
