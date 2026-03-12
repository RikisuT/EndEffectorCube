# ESP32 Servo Controller

Interpolating servo controller firmware for Delta Robot.

## Features

- **Protocol**: Fire-and-forget commands with duration
- **Interpolation**: Hybrid linear/cubic easing for smooth motion
- **Control Loop**: 50Hz synchronized with servo PWM

## Protocol

Commands are sent over **Serial at 115200 baud**:

```
T<idx>:<degrees> D:<duration_ms>
```

### Examples

```
T0:45.00 D:200    # Servo 0 to 45° in 200ms
T1:90.50 D:500    # Servo 1 to 90.5° in 500ms
T2:30.00 D:100    # Servo 2 to 30° in 100ms
```

## Configuration

```c
// main/mcpwm_servo_control_example_main.c

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
