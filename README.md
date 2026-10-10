# Arduino-Drone

A quadcopter drone built on the Arduino platform. A handheld controller sends stick and button data to the drone with nRF24L01 radios. An Arduino Mega on the drone reads the sensors and controls the four motors.

**Status:** Work in progress. Flight control is designed but not written yet. See [docs/STATUS.md](docs/STATUS.md).

## Repository layout

| Folder | Contents |
|---|---|
| [firmware/controller/](firmware/controller/) | Sketch for the handheld controller |
| [firmware/receiver/](firmware/receiver/) | Sketch for the drone (Arduino Mega) |
| [bench-tests/](bench-tests/) | Small test sketches for each sensor, and ESC calibration |
| [hardware/stl/](hardware/stl/) | 3D-print files for the body, brackets, and covers |
| [hardware/datasheets/](hardware/datasheets/) | nRF24L01+ datasheet |
| [libraries/](libraries/) | Arduino library zips used by the sketches |
| [docs/](docs/) | Project status, design spec, and notes |

## Hardware

- Arduino Mega 2560 (drone)
- Arduino Uno-class board (controller)
- 2 x nRF24L01 radio modules
- Adafruit 10DOF IMU (accelerometer, magnetometer, gyro, barometer)
- HC-SR04 ultrasonic sensor (height near the ground)
- 4 x brushless motors with ESCs
- 1 x servo for the drop arm (drops objects)
- 2 x joysticks and 3 buttons on the controller

## Libraries

Install each zip in [libraries/](libraries/) that the sketches need. In the Arduino IDE, select **Sketch > Include Library > Add .ZIP Library**.

| Zip | Used by |
|---|---|
| `RF24-master.zip` | Controller, receiver |
| `Adafruit_Sensor-master.zip` | Receiver, bench tests |
| `Adafruit_10DOF-master.zip` | Receiver, `imu-10dof` test |
| `Adafruit_LSM303DLHC-master.zip` | Receiver, `accel` and `imu-10dof` tests |
| `Adafruit_L3GD20_U-master.zip` | Receiver, `imu-10dof` test |
| `Adafruit_BMP085_Unified-master.zip` | Receiver, `imu-10dof` test |
| `i2cdevlib-master.zip`, `MPU6050.zip` | `mpu6050-dmp` test only |
| `ESC-calibration-master.zip`, `Gy521-Dev-Kit-master.zip`, `IMU.zip`, `RC_ESC-master.zip` | Not used. Kept for reference. |

## Safety

WARNING: REMOVE THE PROPELLERS BEFORE YOU DO WORK ON THE DRONE ON THE BENCH. THE MOTORS CAN START WITHOUT A WARNING.

## More information

- [docs/STATUS.md](docs/STATUS.md): where the work stopped and how to start again
- [Flight control design](docs/flight-control-design.md)
- [ESC calibration](bench-tests/esc-calibration/README.md)
