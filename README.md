# Hybrid Navigation Autonomous Robot (Bug-2 Style)

An Arduino-powered autonomous robot that combines closed-loop line following with ultrasonic wall-following. The system mimics a simplified Bug-2 navigation algorithm: it follows a path (the "m-line"), switches to boundary following when it detects an obstacle, and resumes the original path once the obstacle is cleared.

## Hardware Configuration

The codebase is designed for an Arduino microcontroller with the following components:

- **Motor driver:** L298N dual H-bridge controller
- **Actuators:** 2x DC motors, 1x micro servo (head)
- **Sensors:**
  - HC-SR04 ultrasonic distance sensor mounted on the servo
  - 5-channel IR reflectance sensor array (`LFSensor_0` to `LFSensor_4`)

## Pin Mapping

| Component | Pin / Channel | Description |
|---|---:|---|
| IR Sensors | A0-A4 | Reflectance tracking array (left to right) |
| Servo Motor | 10 | Panoramic sensor sweep positioning |
| Ultrasonic Trig | 3 | Trigger pin for distance pulses |
| Ultrasonic Echo | 2 | Echo input pin for pulse reflection |
| Motor Right PWM | 9 | Speed control (`ENA` on L298N) |
| Motor Right Dir | 12, 11 | Direction control (`IN1`, `IN2`) |
| Motor Left PWM | 6 | Speed control (`ENB` on L298N) |
| Motor Left Dir | 7, 8 | Direction control (`IN3`, `IN4`) |

## Overall Behavior and State Machine

The robot operates as a finite state machine with two main operating modes defined by the `Mode` enum:

```cpp
enum Mode { LINE_FOLLOW, WALL_FOLLOW };
```

### 1. Line-Following Mode (`LINE_FOLLOW`)

The robot treats the black floor line as its main trajectory vector, analogous to the Bug-2 m-line. A 5-channel IR reflectance array is used in closed-loop control to keep the robot centered on the line.

### 2. Wall-Following Mode (`WALL_FOLLOW`)

This mode is triggered when an obstacle blocks the path at less than 12 cm, or when the line is completely lost (`00000`). The robot then transitions into a right-hand-rule boundary-tracing pattern:

- It stops forward movement and scans the environment by sweeping the servo to `45°`, `90°`, and `135°`.
- It tries to maintain an optimal distance of `12 cm` with a tolerance of `±3 cm` from the right wall.
- Once the IR array detects the line again, the robot calculates a leave point, exits wall-following, and returns to `LINE_FOLLOW`.

### 3. Goal Condition

When all five sensors detect a solid black cross-line (`11111`), the robot registers `goalHit`, cuts motor power, and halts execution.

## Function Deep Dive

### Actuation and Low-Level Motor Control

#### `void go_Advance(void)`

Sets both motor directions to forward at a hardcoded PWM baseline of `200`.

#### `void go_Left(int t = 0)` / `void go_Right(int t = 0)`

Counter-rotates the wheels to spin the robot around its central axis. The optional `t` parameter, in milliseconds, controls open-loop timed turn duration.

#### `void go_Back(int t = 0)`

Reverses direction polarity on all H-bridge channels.

#### `void stop_Stop()`

Brings the robot to a complete halt by pulling all directional control pins `LOW`.

#### `void set_Motorspeed(int speed_L, int speed_R)`

Directly writes individual 8-bit PWM values (`0-255`) to `speedPinL` and `speedPinR` for precise torque and speed balancing.

### Sensor Data Acquisition and Processing

#### `int watch()`

Calculates the real-time distance to the nearest object using the HC-SR04 ultrasonic sensor. It uses a microsecond pulse sequence and applies the time-of-flight equation:

$$
d = \frac{t \times v_{\text{sound}}}{2}
$$

Using the speed of sound at standard sea-level conditions (343 m/s, or 0.0343 cm/μs), the code scales `pulseIn()` timing data using the calibrated constant below:

$$
\text{Distance (cm)} = \text{Time} \, (\mu s) \times 0.01657
$$

#### `String read_sensor_values()`

Reads the raw binary input from the five IR line-tracking pins. It performs bitwise shifts to pack the inverted sensor logic (`!digitalRead`) into a unified 5-bit binary string, for example `"01100"` for a line centered under the robot.

### High-Level Behavior Routines

#### `void wallFollow()`

Performs an environmental sweep using the micro-servo to collect three directional distance values:

- `frontDist` at `90°`
- `leftDist` at `45°`
- `rightDist` at `135°`

It then evaluates the environment using simple rule-based logic:

- **Front blocked:** turn right out of the hazard zone
- **Too far from wall:** adjust slightly inward to the right
- **Too close to wall:** adjust outward to the left

#### `void auto_tracking()`

The master behavior controller called from `loop()`. It fuses the sensor pattern returned by `read_sensor_values()` with distance values from `watch()` to handle state transitions and proportional motor correction.

## Known System Limitations

- **Timed open-loop turns:** Wall-following relies on fixed `delay(80)` and `delay(200)` style timing. Without wheel encoder feedback, turning accuracy may drift as battery voltage changes or surface friction varies.
- **Fixed turning modifiers:** Hardcoded line-tracking speed profiles can drop one side to `0` during aggressive correction, which may create jerky movement on winding paths.

## Author and Version Control

- **Developer:** George A A Sorial 
- **Release date:** December 8, 2025 (`18:07`)
- **Environment:** Arduino IDE, with the `Servo.h` core dependency library
