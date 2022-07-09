# cave-crawler-mcu

cave-crawler microcontroller firmware code (Teensy 3.5)

See also:
- [cave-crawler-lib](https://github.com/bmegli/cave-crawler-lib) that communicates with this firmware
- [cave-crawler](https://github.com/bmegli/cave-crawler) for "top-level" project

## Purpose

Precise time synchronization of multiple sensors:
- IMU (EM71780)
- encoders
- lidars (RPLidar A3, XV11Lidar)

Communication with embedded CPU.

## State

Functional.

Code was carefully written in non-blocking manner:
- IMU (EM1780) is timestamped in interrupt
  - encoders are flagged for ASAP readout to get sync between IMU/encoders
  - IMU is read through asynchronous I2C (non-blocking)
- lidar communication is non-blocking and timestamped on first received packet byte
- embedded CPU (host controller) communication is non blocking

All together this achieves [10s of microseconds](https://github.com/bmegli/cave-crawler-mcu/issues/4) order worst case loop time.

## Platforms

Firmware code written for Teensy 3.5.

## Hardware

Combination of:
- IMU (EM71780)
- lidars (RPLidar A3, XV11Lidar)
- encoders
- embedded CPU running [cave-crawler-lib](https://github.com/bmegli/cave-crawler-lib)

Using subset of above hardware requires modification of top-level [cave-crawler-mcu.ino](https://github.com/bmegli/cave-crawler-mcu/blob/master/cave-crawler-mcu.ino)

## Dependencies
- [serial-nb-arduino](https://github.com/bmegli/serial-nb-arduino.git)
- [xv11lidar-arduino](https://github.com/bmegli/xv11lidar-arduino)
   - [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)
- [rplidar-a3-arduino](https://github.com/bmegli/rplidar-a3-arduino.git)
   - [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)
- [EM7180](https://github.com/bmegli/EM7180.git) (async-i2c branch)
   - [CrossPlatformI2C](https://github.com/bmegli/CrossPlatformI2C.git)

## License

If somebody needs to relax the license from GPL to say MPL, let me know.
There is no problem with that but I no longer work on this project.

