# 6. Orientation Control

This section provides an in-depth explanation of ViZio's orientation control system, focusing on the Proportional-Derivative (PD) control loop, MPU6050 gyroscope integration, and the critical calibration process. Maintaining a stable and precise trajectory is fundamental for VizDrive's performance in the WRO circuit.

To improve accuracy, the code incorporates two crucial techniques: **sensor calibration** to eliminate bias and the application of an **exponential low-pass filter** to smooth out noise.

---

## 6.1 MPU6050 Gyroscope Integration

The **MPU6050** is a 6-axis motion tracking device that includes both a 3-axis gyroscope and a 3-axis accelerometer. VizDrive primarily utilizes the gyroscope's Z-axis (Yaw) data to determine the robot's real-time angular velocity and subsequently its orientation.

### Global Object and Variables

```cpp
Adafruit_MPU6050 mpu; // MPU6050 sensor object
static float yaw = 0.0; // Current accumulated yaw angle of the robot (in degrees)
float gyroZ_offset = 0.0; // Z-axis gyroscope offset for calibration (in degrees/second)
```

### Initialization (`void initOrientation()`)

* **Purpose**: Initializes the MPU6050 sensor and performs a crucial calibration step to determine the Z-axis gyroscope's static bias.
* **Operation**:
    1. `Wire.begin();`: Initializes the I2C communication protocol, which the MPU6050 uses.
    2. `if (!mpu.begin()) while (1);`: Attempts to initialize the MPU6050. If initialization fails (e.g., sensor not detected), the program enters an infinite loop.
    3. **Sensor Configuration**:
          * `mpu.setAccelerometerRange(MPU6050_RANGE_8_G);`: Sets the accelerometer's measurement range to ±8g.
          * `mpu.setGyroRange(MPU6050_RANGE_500_DEG);`: Sets the gyroscope's measurement range to ±500 degrees per second, suitable for robot maneuvers.
          * `mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);`: Configures the Digital Low Pass Filter (DLPF) bandwidth to 21 Hz to reduce high-frequency noise.
    4. `delay(100);`: A brief pause to allow sensor stabilization.
    5. **Z-axis Gyroscope Calibration**: This routine calculates `gyroZ_offset`, the average static drift of the gyroscope when the robot is stationary.
          * A loop collects `samples` (e.g., 250) of Z-axis gyroscope data (`g.gyro.z`).
          * The `gyroZ_offset` is computed as the sum of these readings divided by the number of samples, effectively averaging out the bias. This offset will be subtracted from all subsequent raw readings for accuracy.

### Yaw Angle Accumulation (`void updateOrientation(float dt)`)

* **Purpose**: Continuously calculates and updates the robot's current accumulated yaw angle (`yaw`) based on gyroscope data.
* **Parameters**:
  * `dt`: A float representing the time elapsed (in seconds) since the last update. In the **Open Challenge Round**, it is passed from the `loop()` function, ensuring time-based integration. In contrast, in the **Obstacle Challenge Round**, this time differential is calculated from the in the global variable `lastUpdateOrientationTime`. This implementation helped make the `updateOrientation()` function more independent from the other modules, helping update the yaw angle in the `safeDelay()` more precisely.
* **Mathematical and Physical Foundations**:
  The code operates under fundamental principles of physics and signal processing:
    a. Gyroscope and Drift Measurement

    A gyroscope measures angular velocity (ω), which is the rate of change of the angle of rotation. The MPU6050 provides this measurement in rad/s. To obtain the yaw angle (θ), the angular velocity must be integrated over time:

    $$θ(t) = θ_0 + \int_{t_0}^{t} \omega(τ) \, dτ$$

    In the digital implementation, this integral is approximated by a discrete sum:

    $$yaw_{new} = yaw_{previous} + (\omega \cdot \Delta t)$$

    On which:

    - $yaw_{previous}$ is the accumulated angle.
    - $\omega$ is the angular velocity, measured by the gyroscope.  
    - $\Delta t$ is the time interval between measurements.  

    Simple gyroscope integration is prone to a serious problem known as **drift**. This occurs because any small errors or biases in angular velocity measurements accumulate over time, causing the calculated angle to progressively deviate from the true value.


* **Operation**:
    1. `mpu.getEvent(&a, &g, &temp);`: Retrieves the latest sensor event data, including gyroscope readings (`g`).
    2. `float gyroZ_deg = (g.gyro.z - gyroZ_offset) * 57.2958;`:
          * The raw Z-axis gyroscope reading (`g.gyro.z`) is compensated by subtracting the `gyroZ_offset`.
          * The result (in radians/second) is converted to degrees/second by multiplying by $180 /π$ $\\approx$ $57.2958$.
    3. `if (abs(gyroZ_deg) > 0.1) { yaw += gyroZ_deg * dt; }`:
          * A noise threshold (0.1 degrees/second) is applied. If the angular velocity is below this, it's considered noise and not integrated, preventing drift when the robot is stationary or moving very slowly.
          * Otherwise, the compensated angular velocity (`gyroZ_deg`) is multiplied by the time difference (`dt`) and added to the `yaw` variable, integrating the angular velocity over time to obtain the accumulated angle.

---



## 6.2 Fundamentals of PID Control

The integral term accumulates error over time to eliminate steady-state offset and overcome persistent opposing forces as friction or gravity. However, our steering mechanism experiences no physical resistance or force that prevents it from turning. Because our system is high-dynamic and operates in short, real-time intervals, error accumulation is unnecessary and could result in severe overcorrection or overshoot. Thus, we exclude the integral term (Ki = 0) and switch to a proportional and derivative term controller.

* **P (Proportional) Term**:
  * **Function**: Reacts to the *current* error.
  * **Effect**: Provides the primary driving force for correction. A larger error results in a proportionally larger corrective action.
* **I (Integral) Term**:
  * **Function**: Accounts for *past* errors by summing up instantaneous errors over time.
  * **Effect**: Helps to eliminate any steady-state error (offset) that the proportional term might leave.
* **D (Derivative) Term**:
  * **Function**: Predicts *future* errors based on the *rate of change* of the current error.
  * **Effect**: Helps to dampen oscillations and improve the system's response speed by applying a larger correction when the error is changing rapidly, and a smaller one when it is stabilizing.

  <img src="../assets/flowcharts/PID.jpeg" width="400" alt="PixyCam2 ICSP Protocol Pins">

---

## 6.3 Application to Yaw Axis Control (PD Control)

VizDrive implements a **Proportional-Derivative (PD) control** scheme to determine the appropriate steering angle for the servo motor. This corrects orientation errors and maintains the robot's intended trajectory.

### Global Variables (PD Specific)

```cpp
static float targetYaw = 0.0; // The desired yaw angle for the robot (in degrees)
static float previousError = 0.0; // Stores the error from the previous control cycle for derivative calculation

// PID error variables (Ki is set to 0, indicating a PD controller)
const float Kp = 1.8; // Proportional gain
const float Kd = 1.2; // Derivative gain
const float Ki = 0.0; // Integral gain (inactive)
```

### `void keepOrientation()`

* **Purpose**: Implements the PD control loop to adjust the steering servo and maintain the `targetYaw`.
* **Operation**:
    1. **Error Calculation**:
          * `float error = targetYaw - yaw;`: Computes the difference between the desired `targetYaw` and the current `yaw`. A positive error means the robot needs to turn in one direction to reach the target, and a negative error means the opposite.
    2. **Derivative Calculation**:
          * `float derivative = error - previousError;`: Determines the rate of change of the error. This term predicts future error and helps to dampen oscillations, improving the system's stability and response time.
    3. **Correction Calculation**:
          * `int correction = SERVO_STRAIGHT + (int)(Kp * error + Kd * derivative);`:
              * `Kp * error`: The proportional component, which scales the correction based on the magnitude of the current error.
              * `Kd * derivative`: The derivative component, which scales the correction based on how rapidly the error is changing.
              * `SERVO_STRAIGHT`: The base angle for the steering servo (neutral position). The calculated PD terms are added to this base to derive the final steering angle.
    4. **Constraint and Actuation**:
          * `correction = constrain(correction, SERVO_LEFT, SERVO_RIGHT);`: Ensures the calculated steering angle remains within the safe and physically achievable limits defined by `SERVO_LEFT` and `SERVO_RIGHT`.
          * `setSteeringAngle(correction);`: Sends the adjusted steering angle to the servo motor.
    5. **Update Previous Error**:
          * `previousError = error;`: Stores the current `error` to be used as `previousError` in the next control cycle for the derivative calculation.

  <img src="../assets/animations_graphics/GifKeepOrientation.gif" width="400" alt="KeepOrientation">

### Exclusion of Integral Term (`Ki = 0`)

The integral term (`Ki`) is set to zero because the control loop operates in very short, real-time intervals. In this context, persistent, cumulative errors are negligible, and the system is highly dynamic. Introducing an integral term, which primarily addresses long-term steady-state errors, would be unnecessary and could introduce instability or over-correction due to wind-up effects.

### PID Constants Tuning

The `Kp` and `Kd` parameters are critical for optimal performance and have been determined empirically:

* `Kp = 1.8`
* `Kd = 1.2`
* `Ki = 0.0` (as explained above, the integral term is not used for yaw control)

These values provide a stable and responsive control loop:

* **`Kp` (Proportional Gain)**: Was adjusted by gradually increasing it until the robot started oscillating around the desired path (indicating overcorrection), then slightly reducing it to achieve a balanced response.
* **`Kd` (Derivative Gain)**: Was tuned after `Kp` to smooth the response, reduce overshoot, and dampen oscillations, leading to a more precise and stable trajectory.

### Target Yaw Management Functions

These functions allow external modules (e.g., `state_logic`) to interact with and query the orientation control system:

* **`void setTargetYaw(float angle_deg)`**:

  * **Purpose**: Sets a new desired `targetYaw` angle for the PD controller to aim for.
  * **Usage**: Critical for executing turns; for example, `handleColorAction()` updates `targetYaw` by adding or subtracting 90 degrees based on the detected turn direction.

* **`float getCurrentYaw()`**:

  * **Purpose**: Returns the robot's current accumulated yaw angle.
  * **Returns**: The `yaw` float value in degrees.
  * **Usage**: Used by `state_logic` to calculate `turnTargetYaw` and evaluate turn completion.

* **`float getAbsoluteYawError()`**:

  * **Purpose**: Returns the absolute difference between the `targetYaw` and the current `yaw`.
  * **Returns**: The error value (`abs(targetYaw) - abs(yaw)`) in degrees.
  * **Usage**: Crucial for the `completedTurn()` function to determine if a turn has been successfully completed by checking if `getAbsoluteYawError()` is less than `TURN_THRESHOLD`.

---

## 6.4 Code Structure

### Includes and Declarations

The code uses the `Wire.h`, `Adafruit_MPU6050.h`, and `Adafruit_Sensor.h` libraries to communicate with the MPU6050 sensor.

### `setup()`

* **Serial Initialization**: `Serial.begin(115200)` for communication and debugging via the serial monitor.
* **MPU6050 Initialization**: `mpu.begin()` and configuration of the accelerometer and gyroscope ranges (`setAccelerometerRange`, `setGyroRange`), as well as configuration of the digital low-pass filter (`setFilterBandwidth`).
* **Calibration**: The `calibrateGyroZ()` function is called to obtain the gyroscope offset.
* **Motors**: The `MOTOR_INA_PWM` and `MOTOR_INB_PWM` outputs are configured to control motors, although they are not actively used in the `loop()` to control the robot.

### `loop()`

* **Sensor Measurement**: `mpu.getEvent(&a, &g, &temp)` reads accelerometer, gyroscope, and temperature data.
* **Time Calculation (`dt`)**: `(now - lastMicros) / 1000000.0` calculates the elapsed time in seconds between readings, which is essential for accurate integration.
* **Applying `offset` and Converting**: `(g.gyro.z - gyroZ_offset) * 57.2958` subtracts the bias and converts the angular velocity from radians/s to degrees/s.
* **Filtering and Integration**: The filtered angular velocity (`gyroZ_filtered`) is updated using the exponential filter formula and then, if it exceeds the threshold, is integrated to update the yaw angle (`yaw`).
* **Print and Delay**: The calculated `yaw` angle is printed for monitoring and a small `10` ms delay is included to avoid overloading the serial port.

---

## 6.5 MPU Calibration Process

Accurate MPU readings are fundamental for effective PID control. A dedicated calibration procedure is performed to account for manufacturing biases and environmental factors.

### Calibration and Offset
Gyroscopes, even at rest, often report a small non-zero angular velocity. This bias, or offset, is a major source of drift. The code addresses this with a calibration function:

```cpp
void calibrateGyroZ(int samples = 500) {
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }
  gyroZ_offset = sum / samples;
}
````

This function takes a large number of samples from the gyroscope while assuming the sensor is stationary. The average of these samples is calculated and stored as `gyroZ_offset`. In the main loop, this value is subtracted from each raw reading to cancel out bias:

```cpp
float rawGyroZ = (g.gyro.z - gyroZ_offset) * 57.2958; // rad/s a deg/s
```

### Exponential Low-Pass Filter

To reduce high-frequency noise in the gyroscope measurements, the code uses an exponential low-pass filter. This filter is a simple but effective way to smooth the data, giving more weight to the most recent measurement and less weight to older measurements. The formula is:

$Output_{filtered} = \alpha \cdot Output_{filtered} + (1 - \alpha) \cdot Input_{raw}$

In code, this translates to:

```cpp
float alpha = 0.8; // coeficiente del filtro exponencial
// ...
gyroZ_filtered = alpha * gyroZ_filtered + (1 - alpha) * rawGyroZ;
```

* **`alpha`**: This is a coefficient between 0 and 1.

* A high value (close to 1) makes the filter more sensitive to rapid changes (less smoothing).
* A low value (close to 0) smooths the data more but with greater delay.
* The value `0.8` in this code is adjusted accordingly to the performance.

### Noise Threshold

Even after calibration and filtering, some residual noise may persist, which, when integrated, would still cause drift. The code implements a **threshold** to ignore any very small changes:

```cpp
if (abs(gyroZ_filtered) > 0.3) {
  yaw += gyroZ_filtered * dt;
}
```

This ensures that angle integration only occurs when there is significant movement, preventing accumulated noise from causing unwanted drift. The value of `0.3` (degrees/s) acts as a "dead zone" for the sensor.


* **Data Collection**: This code collects gyroscope data over a measurement period (e.g., 10 seconds), testing various sample sizes to determine the optimal average correction.

* **Offset Determination**: The `gyroZ_offset` is calculated by averaging gyroscope readings over a set number of samples (e.g., 250 samples) while the sensor is stationary. This offset is then subtracted from subsequent raw gyroscope readings to ensure accurate, bias-corrected measurements.

* **Optimal Sample Size**: Data visualization (as shown in the graph below) indicates that approximately **200-250 samples** are optimal for calibrating the `gyroZ_offset`, as the error reduction becomes insignificant beyond this point, achieving an accuracy of approximately `±0.01`.

* **Data Visualization**:

![Accumulated error vs Number of samples](../assets/data_graphs/MPU_data_graph.png)

Proper calibration of this sensor is essential and can significantly reduce offset, and we tried to reduce to the minimum the error. 
However, with the MPU-6050, it is not possible to achieve zero offset when working with accumulated yaw. For this reason, we implemented different angle correction strategies depending on the challenge.
In the Open Challenge, where touching the walls is not allowed, we adapted a correction system based on ultrasonic sensors. 
In contrast, during the Obstacle Challenge, where contact with walls is permitted, we used the walls themselves as reference points to reset the accumulated yaw. 

For more information about the difference between the Open Challenge and Obstacle Challenge Rounds correction logic, refer to: [Color Detection: 9.3 Turn Maneuver Logic()](./09_color_detection.md)

At the end of the day, we are optimizing the performance of our sensors; but we have to acknowledge their limitations.

---

[Back to Main README.md Index](./../README.md)
