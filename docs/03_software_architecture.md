# 3. Software Architecture

This section details VizDrive's software architecture, covering essential libraries, an overview of its modular functionalities, and the overall firmware structure. The code is implemented in C++ for the Arduino Mega 2560 Pro Embed.

---

## 3.1 Required Libraries

VizDrive uses the following Arduino and Adafruit libraries to enable its functionalities:

* **`Servo.h`**: Standard Arduino library for controlling servo motors, specifically for steering.
* **`Wire.h`**: Facilitates I2C (Inter-Integrated Circuit) communication, used by the MPU6050 and the I2C color sensor.
* **`NewPing.h`**: Dedicated library for ultrasonic sensors, optimizing distance measurements.
* **`Adafruit_MPU6050.h`**: Provides the interface for the MPU6050 gyroscope and accelerometer.
* **`Adafruit_Sensor.h`**: A core library supporting various Adafruit sensors, including the MPU6050.
* **`Adafruit_TCS34725.h`**: Integrates the TCS3472 I2C color sensor (this library is compatible with both TCS3472 and TCS34725 color sensor).
* **`Pixy2.h`**: Enables communication and data retrieval from the PixyCam 2.1.

---

## 3.2 Modular Functionalities

The firmware is structured into independent modules, each abstracting a primary robot function. This modular design improves readability, simplifies debugging, and supports isolated development of specific code sections.

This document provides an **overview** of each module. We highly recommend refering to each module's dedicated documentation for a comprehensive explanation on functionalities.

### Motion and Steering

This module directs the robot's physical movement, including propulsion and directional control.

* **`driveForward(speed)` / `driveBackwards(speed)`**: PWM-controlled functions for DC motor operation, setting speed (0-255) and direction.
* **`stopMotors()`**: Implements a "soft stop" by disabling motor PWM signals, allowing for natural deceleration.
* **`stopBrake()`**: Executes a "hard stop" using short-circuit braking for immediate halts.
* **`setSteeringAngle(angle)`**: Adjusts the front-wheel steering servo position. Angle values are bounded by `SERVO_LEFT` and `SERVO_RIGHT` constants to prevent mechanical interference.
* **`performParking()`**: Manages the precise reverse maneuver required for parking, utilizing encoder feedback for distance control.
* **`encoderISR()`**: An Interrupt Service Routine (ISR) that increments an internal counter (`encoderPulseCount`) for accurate distance tracking.

**Key Constants:**

* `SERVO_LEFT`, `SERVO_RIGHT`: Define the minimum and maximum safe steering angles.
* `MOTOR_INA_PWM`, `MOTOR_INB_PWM`: Specify the PWM pins for controlling the H-bridge motor driver.
* `PARKING_PULSES`: The calibrated encoder pulse count for the parking distance.

For detailed explanations of these functions, refer to [**Robot Mobility**](./05_robot_mobility.md).

### Distance Detection

This module employs ultrasonic sensors to measure real-time distances to surrounding environmental features, primarily for maintaining optimal positioning relative to walls.

* **`getDistance(sonar)`**: Computes and filters distance measurements from multiple pings using a `NewPing` object, effectively reducing noise and enhancing precision in measurements.
* **`centreOnStart()`**: Analyzes distance readings from side-mounted ultrasonic sensors. It calculates the necessary pulses to center the robot according to both wall distances and the servo steering angle, and executes the number of pulses adjusting the robot's steering angle to correct its position.
* **`avoidWall()`**: If the distance to the walls differ with the expected maximum and minimum thresholds, the robot adjustes the target yaw for gyroscopic correction. This function has a cooldown managed by **`correctionCooldown()`** to avoid repeating a correction constantly.

For a more detailed analysis of the ultrasonic sensors integration and noise filtering, refer to [**Ultrasonic Distance Sensing**](./08_ultrasonic_distance_sensing.md).

### Vision-Based Obstacle Evasion

VizDrive uses the **PixyCam 2.1** to detect and react to colored obstacles in its path.

* **`obstacleDetected()`**: Checks for the presence of any designated colored object within a predefined **Region of Interest (ROI)** in the camera's view. Returns `true` if an object is found, `false` otherwise.
* **`handleEvasion()`**: Initiates and manages the evasion process. It identifies the largest obstacle within the ROI and adjusts the robot's steering based on the object's color signature:
  * **Signature 1 (Red)**: Triggers an evasion maneuver to the **right**.
  * **Signature 2 (Green)**: Triggers an evasion maneuver to the **left**.

**Debugging:**

* **`debugPixy()`**: Provides serial output of all detected blocks, including their coordinates and signatures. This function is typically commented out in optimized code to conserve processing power but was used during calibration and troubleshooting.

For a comprehensive explanation of PixyCam 2.1 integration and computer vision principles, consult [**Computer Vision Functions**](./07_pixycam_computer_vision.md).

### Color Detection

This module is responsible for identifying specific color patterns on both the track surface and side walls, crucial for autonomous navigation and mission objectives.

#### 1. Floor Detection (I2C TCS34725 Sensor)

This sensor, mounted underneath the robot, detects navigation cues on the track.

* **Blue**: Indicates a requirement for a **left** turn.
* **Orange**: Indicates a requirement for a **right** turn.
* **`detectFloorColor()`**: Reads raw RGB data from the sensor and normalizes these readings using the clear channel value to compensate for ambient light variations. It then compares normalized values against predefined color thresholds.

#### 2. Wall Detection (Dual TCS3200 Sensors)

Two digital TCS3200 sensors, positioned on the robot's sides, are used primarily for detecting the parking trigger.

* **Magenta**: Detected after completing a specific number of laps (e.g., 4 laps), signaling the parking zone.
* **`detectWallMagenta()`**: Reads filtered RGB values from the appropriate side sensor (selected based on the current turning `direction`) and compares them against magenta-specific thresholds.

**Thresholds:**

* All color detection relies on empirically tuned RGB intensity thresholds (0-255) for accurate color identification under varying conditions.

For an in-depth explanation of color sensor operation, calibration, and detection logic, refer to [**Color Detection Functions**](./09_color_detection.md).

### Orientation Control

This module ensures the robot maintains a stable trajectory and executes precise turns using an MPU6050 gyroscope and a Proportional-Derivative (PD) control loop.

* The **MPU6050 gyroscope** provides angular velocity data (specifically from its Z-axis) which is integrated to track the robot's current yaw angle.
* **`updateOrientation(dt)`**: Accumulates the robot's yaw angle by integrating the calibrated angular velocity over time (`dt`).
* **`keepOrientation()`**: Implements the PD controller. It calculates an error between the `targetYaw` and the current `yaw`, then applies a proportional and derivative correction to the steering servo angle to guide the robot back to its target heading.
* **`setTargetYaw(angle)`**: Dynamically defines the desired yaw angle for the robot to maintain or achieve during maneuvers (e.g., setting a 90-degree turn target).
* **Gyroscope Calibration**: An initial calibration routine at setup compensates for any inherent bias or drift in the MPU6050's Z-axis gyroscope readings.

**Tuning Parameters:**

* `Kp = 1.8`: Proportional gain, influencing the response to the current heading error.
* `Kd = 1.2`: Derivative gain, influencing the response to the rate of change of the heading error, helping to dampen oscillations.
* `Ki = 0.0`: Integral gain, currently unused, but could be implemented to eliminate steady-state errors.

For a detailed exploration of MPU6050 integration, calibration, and the PD control algorithm, consult [**PID Control for the Gyroscope**](./06_pid_gyroscope_control.md).

---

## 3.3 Program Flow and State Management

The `main.ino` file orchestrates the robot's behavior, with the `setup()` function initializing the system and the `loop()` function continuously managing operational states and transitions.

### Initialization (`setup()`)

The `setup()` function executes once at program startup:

1. **Serial Communication**: Initiates `Serial.begin(115200)` for debugging and monitoring.
2. **Module Initialization**: Calls initialization for all hardware and software modules:
    * `initMovement()`: Configures motors, servo, and encoder.
    * `initOrientation()`: Initializes MPU6050 and performs gyroscope calibration.
    * `initVision()`: Prepares the PixyCam for operation.
    * `initUltrasonic()`: Sets up ultrasonic distance sensors.
    * `initColorSensors()`: Initializes both I2C and digital color sensors.
    * `initStateLogic()`: Establishes initial state variables, including `direction` and `targetYaw`.
3. **Start Condition**: Enters a waiting loop (`waitForStartButton()`) until a physical button press signals the start of autonomous operations.
4. **Timing Baseline**: Records `lastUpdateTime` using `millis()` to establish a timestamp for loop timing.

### Main Loop (`loop()`)

The `loop()` function runs continuously, performing core functionalities in a timed sequence:

1. **Timing Control**: Ensures that main loop updates occur approximately every 50 milliseconds, calculating `dt` (delta time) for precise orientation updates.
2. **Continuous Movement**: `driveForward(motorSpeed)` maintains the robot's forward progression.
3. **Orientation Correction**:
    * `updateOrientation(dt)`: Regularly updates the robot's `yaw` angle.
    * `keepOrientation()`: Applies the PD control logic to adjust steering, maintaining the `targetYaw`.
4. **Obstacle Management**:
    * `if (obstacleDetected())`: Checks for obstacles using the PixyCam.
    * `handleEvasion()`: Executes the evasion maneuver if an obstacle is found.
    * `recentreUltrasonic()`: Realigns the robot relative to walls post-evasion.
5. **Color-Based Action**:
    * `if (colorDetected())`: Checks for relevant color signals (floor lines or wall markers).
    * `handleColorAction()`: Manages specific actions such as track turns or parking, determined by the `lapCompletedCount` and detected color.
6. **Mission Progress & Shutdown**:
    * `updateLapCount()`: Tracks completed turns and increments `lapCompletedCount`.
    * `checkForShutdown()`: Monitors the `SystemShutdown` flag; if true, the robot halts all operations.

---

## 3.4 State Variables and Logic

The robot's dynamic operational state is managed through a set of global boolean and integer flags:

### State Variables

| Variable              | Type            | Purpose |
|-----------------------|-----------------|---------|
| `direction`           | `int`           | Defines the turning direction: -1 = left, +1 = right (set at startup in `initStateLogic()`). |
| `lapTurnCount`        | `unsigned int`  | Number of turns in current lap. A full lap is currently defined as having 4 turns.|
| `lapCompletedCount`   | `unsigned int`  | Tracks the total number of completed laps. Upon reaching 4 laps, the robot transitions to searching for the parking magenta wall. |
| `SystemShutdown`      | `bool`          | When set to `true`, this flag triggers `checkForShutdown()`, which forcefully stops the motors (`stopBrake()`) and enters an infinite loop, effectively halting all robot activity. |
| `turningInProgress`   | `bool`          | Prevents re-triggering a turn while one is actively being executed. It is set to `true` upon floor color detection and turn initiation, and `false` when `abs(getYawError())` is within `TURN_THRESHOLD`, signaling turn completion. |
| `correctionState`     | `bool`          | Flag to avoid duplications on the ultrasonic correction by managing an action cooldown. |
| `turnTargetYaw`       | `float`         | Stores the yaw angle the robot aims to achieve during a turn, calculated as `getCurrentYaw() + 90.0 * direction`. |

---

### Non-Blocking Delay (`void safeDelay(unsigned long ms)`)

* **Purpose**: Provides a mechanism for pausing execution for a specified duration (`ms`) without completely freezing the robot's critical sensor updates and control logic.
* **Parameters**:
  * `ms`: The duration of the delay in milliseconds.
* **Operation**: Unlike a standard `delay()`, `safeDelay()` continuously executes essential functions such as `updateOrientation()`, `obstacleDetected()`, `handleEvasion()`, `colorDetected()`, `handleColorAction()`, and `updateLapCount()` within its loop. This ensures the robot remains responsive to environmental changes even during brief pauses in its main sequence.

---

## 3.5 Code Organization

The provided source code is organized into distinct modules (e.g., `main.ino`, `color_detection.cpp`, `movement.cpp`). While presented as a single integrated file for this documentation, in a larger development environment, these would typically be in separate `.cpp` and `.h` files within a `src/` directory. This separation improves code modularity, maintainability, and readability.

---

[Back to Main README.md Index](../README.md)
