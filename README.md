# End Effector Cube

ESP32 firmware for End Effector Cube with VL53L4CD Time-of-Flight sensor.

## Overview

This project implements distance sensing and measurement capabilities using the VL53L4CD ToF (Time-of-Flight) sensor interfaced with an ESP32 via I2C.

## Features

- **Sensor**: VL53L4CD Time-of-Flight distance measurement
- **Interface**: I2C communication (GPIO 21: SDA, GPIO 22: SCL)
- **Frequency**: 50ms measurement intervals (20Hz)
- **Output**: Distance in millimeters with signal strength monitoring

## Hardware Configuration

- **I2C Port**: I2C_NUM_0
- **SDA Pin**: GPIO 21
- **SCL Pin**: GPIO 22
- **Sensor Address**: 0x29
- **Clock Speed**: 400 kHz

## Measurements

The sensor provides:
- Distance readings in millimeters
- Signal strength (kcps/SPAD)
- Range status validation

#define NUM_SERVOS 3
const int SERVO_PINS[NUM_SERVOS] = {2, 4, 5};

#define SERVO_MIN_US 550
#define SERVO_MAX_US 2400

#define CONTROL_FREQ_HZ 50
#define HYBRID_FACTOR 0.8f  // 0.0=Linear, 1.0=Cubic
```

## Interpolation

The firmware uses a hybrid interpolation formula:

```
ease = (1 - HYBRID_FACTOR) * linear + HYBRID_FACTOR * cubic
cubic = 3t² - 2t³  (S-curve)
```

This provides:

- Smooth acceleration/deceleration
- Reduced mechanical stress
- Vibration-free motion

## Building & Flashing

```bash
cd src/servo
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Hardware

| Pin | Function |
|-----|----------|
| GPIO 2 | Servo 1 (Motor 0) |
| GPIO 4 | Servo 2 (Motor 1) |
| GPIO 5 | Servo 3 (Motor 2) |

## Servo Specifications

- **Pulse Width**: 550-2400 µs (0-180°)
- **PWM Frequency**: 50Hz (20ms period)
- **Resolution**: ~10.3 µs per degree
