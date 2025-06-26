# 2. Robot Hardware and Components

This section provides a detailed enumeration and description of all hardware components integrated into the robot.

## Main Control and Processing Unit

* **Component:**  **MEGA 2560 Pro (Embed)**
* **Quantity:** 1
* **Description** Serves as the main controller board, based on the ATmega2560 microcontroller. It processes all sensor data, executes control algorithms, and manages actuation.

## Motion System Components

### **Component:** **Hobby Gearmotor with 48:1 gearbox**
* **Quantity:** 1
* **Description:** A DC motor with an integrated gearbox for increased torque. Utilized for rear-wheel propulsion; also for rear drive.
### **Component:** **Servo Motor SG90**
* **Quantity:** 1
* **Description:** Employed for precise steering of the front wheels.
### **Component:** **L298N Motor Driver**
* **Quantity:** 1
* **Description:** A dual H-bridge motor driver responsible for interfacing the low-current control signals from the Arduino Mega to the higher-current requirements of the DC motors, enabling bidirectional speed control.

## Sensor Suite

### **Component:** **PixyCam 2**
* **Quantity:** 1
* **Description:** A vision sensor for real-time color and object recognition, essential for vision-based obstacle evasion.
### **Component:** **TCS230 Color Sensor**
* **Quantity:** 1
* **Description:** An I2C-based color sensor placed at the bottom of the robot for detecting specific floor line colors (blue and orange) indicating turns.
### **Component:** **TCS34725 Color Sensor**
* **Quantity:** 2
* **Description:** Side-mounted color recognition sensors used for detecting the magenta signal for parking.
### **Component:** **HC-SR04 Ultrasonic Sensors**
* **Quantity:** 4
* **Description:** Front and side-mounted distance sensors utilized for measuring real-time distances to walls, used for collision avoidance and maintaining orientation.
### **Component:** **MPU6050 Accelerometer + Gyroscope**
* **Quantity:** 1
* **Description:** A 3-axis MPU providing acceleration and angular velocity data, used for angle correction and maintaining trajectory stability through PID control.
### **Component:** **Infrared Optocoupler Encoder**
* **Quantity:** 1
* **Description:** A laser interruption-based encoder used for detecting rotational position, enabling accurate measurement of distance and speed (odometry).

## Power Supply and Regulation

### **Component:** **3.7V batteries**
* **Quantity:** 2
* **Description:** Power supply for the robot's electronic components. Specifically, **Li-ion 18650 Battery 3.7V 3400 mAh** is used.
### **Component:** **9V Battery**
* **Quantity:** 1
* **Description:** A standard 9V battery used in the power system.
### **Component:** **Step Up Boost 3V-5V**
* **Quantity:** 1
* **Description:** A voltage booster that steps up 3V to 5V.
### **Component:** **DC to DC Step Down**
* **Quantity:** 1
* **Description:** A voltage regulator that steps down voltage, enabling the robot to adapt to various battery types.

## Miscellaneous Components

### **Component:** **Custom Chassis (RWD)**
* **Quantity:** 1
* **Description:** A 3D designed robot frame and mount, providing the structural basis for all components.
### **Component:** **Pushbutton**
* **Quantity:** 1
* **Description:** A simple pushbutton switch used to initiate the main program.
### **Component:** **Toggle Switch**
* **Quantity:** 1
* **Description:** A switch for toggling between two states, serving as the main power switch for the robot.
### **Component:** **LED**
* **Quantity:** 1
* **Description:** A red LED used as a visual indicator, signaling program readiness.
### **Component:** **Resistor**
* **Quantity:** 1
* **Description:** A 330 Ohms resistor used in conjunction with the LED.

## Circuit Design

We developed a custom shield circuit using a Printed Circuit Board (PCB) to streamline the electrical connections. This approach significantly reduced cable clutter, minimizing the overall weight, size, and complexity of the system. All components were manually soldered onto the PCB to ensure a compact and reliable integration.

* **Detailed Electromechanical Diagram:** For a more in-depth view of how all electronic and mechanical components are interconnected, please refer to the [Electromechanical Diagram](./../schemes/electromechanical_diagram.pdf) in the `schemes/` folder.
* **Cirkit Circuit Design:** A detailed visual representation of our main control circuit, including sensor and motor driver connections to the Raspberry Pi, can be found in the [Cirkit Circuit Design](./../schemes/cirkit_circuit.png) export. This illustrates component placement and wiring logic.

---
[Back to Main README.md Index](#main-readme-index)
```