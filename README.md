# Wall Following Robot with 90° Autonomous Turning

A low-level autonomous robot built using an **ATmega328P (Arduino-compatible)** microcontroller that performs **wall following** and executes **90-degree turns** until the track is completed.

The robot uses:
- PID-based wall following
- Ultrasonic distance sensing
- MPU6050 gyro-assisted turning
- Wheel encoder synchronization
- UART/Bluetooth runtime tuning
- Direct AVR register programming

The system continuously follows corridor walls, detects obstacles ahead, decides the best turning direction, performs accurate 90° turns, and continues navigating until no valid path remains.

---

# Features

- Autonomous wall-following navigation
- Accurate 90° left/right turns
- PID-based steering correction
- MPU6050 gyro integration
- Wheel encoder synchronization
- Front, left, and right ultrasonic sensing
- Bluetooth/UART parameter tuning
- Finite State Machine (FSM) control system
- Direct register-level AVR programming
- Custom `micros()` and `millis()` implementation

---

# Hardware Components

## Microcontroller
- ATmega328P / Arduino Uno

## Sensors
- MPU6050 Gyroscope
- 3 Ultrasonic Sensors
  - Front sensor
  - Left sensor
  - Right sensor

## Motors
- 2 DC Motors
- Motor Driver (L298N or equivalent)

## Feedback
- Left wheel encoder
- Right wheel encoder

## Communication
- HC-05 / HC-06 Bluetooth Module

---

# System Behavior

The robot continuously:
1. Reads front and side distances
2. Maintains alignment using PID wall following
3. Detects obstacles ahead
4. Determines whether to turn left or right
5. Executes accurate 90° turns
6. Continues moving until the track ends

If both sides are blocked and the front path is unavailable, the robot stops and reports the full turn sequence.

---

# Finite State Machine (FSM)

```text
IDLE
MOVE_FORWARD
READ_LEFT_SENSOR
READ_RIGHT_SENSOR
PREPARE_DECISION
CHECK_TURN_DIR_DECIDE
TURN_LEFT
TURN_RIGHT
STOP
````

---

# PID Wall Following

The robot dynamically adjusts motor speeds based on wall distance error:

```text
Correction = Kp * error + Ki * integral + Kd * derivative
```

## Supported Modes

* Left-wall following
* Right-wall following
* Center alignment between walls
* Straight driving in open space

---

# Gyroscope-Assisted Turning

The MPU6050 gyroscope is used to:

* Measure yaw angle
* Improve turn precision
* Reduce turning drift
* Synchronize both wheels during turns

Turning continues until:

* The target angle is reached
* Or a timeout occurs

After each turn, the robot briefly moves forward to align with the new corridor.

---

# Bluetooth Commands

The robot supports live runtime tuning through UART/Bluetooth.

| Command    | Description                   |
| ---------- | ----------------------------- |
| `P<float>` | Set Kp                        |
| `I<float>` | Set Ki                        |
| `D<float>` | Set Kd                        |
| `T<float>` | Set target wall distance      |
| `A<int>`   | Set base speed A              |
| `B<int>`   | Set base speed B              |
| `F<int>`   | Set front stop threshold      |
| `Q<int>`   | Set turn speed                |
| `X<int>`   | Set left encoder turn pulses  |
| `Y<int>`   | Set right encoder turn pulses |
| `G<float>` | Set target turn angle         |
| `C`        | Print all current parameters  |
| `S`        | Start robot                   |
| `R`        | Reset robot                   |

## Example

```text
P1.8
D2.1
T15
S
```

---

# Pin Configuration

## Ultrasonic Sensors

| Sensor | Trigger | Echo |
| ------ | ------- | ---- |
| Front  | PC0     | PC1  |
| Left   | PC2     | PC3  |
| Right  | PD6     | PD5  |

---

## Motor Driver

| Function  | Pin       |
| --------- | --------- |
| ENA (PWM) | PB1 / D9  |
| ENB (PWM) | PB2 / D10 |
| IN1       | PB0       |
| IN2       | PB3       |
| IN3       | PD4       |
| IN4       | PD7       |

---

## Encoders

| Encoder       | Pin        |
| ------------- | ---------- |
| Left Encoder  | PD3 / INT1 |
| Right Encoder | PD2 / INT0 |

---

# Low-Level AVR Techniques

## Direct Register Manipulation

The project directly controls:

* GPIO registers
* Timers
* UART
* I2C
* External interrupts

without relying heavily on Arduino abstraction layers.

## Custom Timing Functions

Custom implementations of:

* `micros()`
* `millis()`

are built using Timer1 overflow interrupts.

## Non-Blocking Timing

Most timing operations use:

```cpp
millis()
micros()
```

instead of blocking delays to improve responsiveness.

---

# Example Output

```text
Turning Left

--------------------
Turns: 3
Sequence: L, R, L
--------------------
```

---

# File Structure

```text
.
├── main.cpp
├── README.md
```

---

# Compilation

Compatible with:

* AVR-GCC
* Arduino IDE

## Recommended Settings

* CPU Frequency: 16 MHz
* Baud Rate: 9600
* PWM Prescaler: 64

---

# Future Improvements

* Smarter turn decision algorithms
* Adjustable turn calibration profiles
* Wireless telemetry dashboard
* EEPROM parameter storage
* Sensor fusion for more stable heading estimation

---

# Source

Project source code included in:
`main.cpp`