# 2. Robot Hardware and Components

This section provides a detailed enumeration and description of all hardware components integrated into the robot and their specifications.

## 2.1 Main Control and Processing Unit

### **Component:**  **MEGA 2560 Pro Embed**

* **Quantity:** 1
* **Voltage:** 5V (operating), 7-9V (Vin)
* **Description:** Serves as the main controller board, based on the ATmega2560 microcontroller. It processes all sensor data, executes control algorithms, and manages actuation.
* **Features:** 54 digital I/O pins, 16 analog inputs, 256 KB flash memory, and 16 MHz clock speed.

<img src="../assets/hardware_photos/mega2560pro.jpg" width="300">

## 2.2 Motion System Components


### **Component:** **Hobby Gearmotor with 48:1 gearbox**

* **Quantity:** 1
* **Voltage Range:** 3-12V (operating at ~8V)
* **Current Consumption** ~120 mA
* **Stall Torque:** ~1.5 kg·cm (0.15 N·m)
* **Stall Current:** ~1.6 A
* **Description:** A brushed DC motor with an integrated gearbox for increased torque. Utilized for rear-wheel propulsion.

<img src="../assets/hardware_photos/hobbymotor.jpg" width="300">

### **Component:** **Servo Motor MG90S**

* **Quantity:** 1
* **Voltage Range:** 4.8-6V
* **Current Consumption:** ~250 mA (max)
* **Torque:** 1.8 kg·cm @ 5V (0.18 N·m)
* **Description:** A micro servo motor employed for front-wheel steering. Operates via PWM signal.

<img src="../assets/hardware_photos/servo.jpg" width="300">

### **Component:** **Mini MX1508 Motor Driver**

* **Quantity:** 1
* **Logic Voltage:** 5V
* **Motor Voltage Range:** Up to 12V
* **Current Output:** Up to 2A per channel
* **Description:** A dual H-bridge motor driver responsible for interfacing the low-current control signals from the Arduino Mega to the higher-current requirements of the DC motors.

<img src="../assets/hardware_photos/motordriver.jpg" width="300">

### **Mechanism**: **Mechanical Differential**

* **Quantity**: 1
* **Gear Ratio**: 1.8:1
* **Material**: Nylon and steel.
* **Description**: A gear assembly that splits engine torque evenly between two wheels while allowing them to rotate at different speeds.

<img src="../assets/hardware_photos/differencial.jpg" width="300">

## 2.3 Sensor Suite

### **Component:** **OpenMV H7 Plus**

* **Quantity:** 1
* **Voltage:** 3.6–5V (via VIN) / 3.3V logic (5V tolerant for most pins)
* **Current Consumption:** ~140 mA
* **Interface:** USB, UART, I²C, SPI
* **Description:** Embedded machine-vision camera module capable of real-time image processing using MicroPython. It includes an STM32H7 microcontroller and camera sensor, allowing tasks such as color tracking, object detection, line following, and AprilTag recognition. In this project it is used for obstacle detection and color-based navigation.

<img src="../assets/hardware_photos/OpenMV.png" width="300">

### **Component:** **TCS3472 Color Sensor**

* **Quantity:** 1
* **Voltage:** 3.3-5V
* **Current Consumption** ~235 µA (active)
* **Interface:** I2C
* **Description:** High-resolution color sensor with IR filtering. Mounted underneath the robot to detect blue and orange floor lines that indicate turns.

<img src="../assets/hardware_photos/tcs3472color.jpg" width="300">

### **Component:** **HC-SR04 Ultrasonic Sensors**

* **Quantity:** 4
* **Voltage:** 5V
* **Current Consumption:** ~15 mA
* **Range of Detection:** ~2 cm to ~4 m
* **Description:** Distance sensors placed at the front and rear utilized for measuring real-time distances to walls, used for collision avoidance and maintaining orientation.

<img src="../assets/hardware_photos/ultrasonicsensor.jpg" width="300">

### **Component:** **MPU6050 Accelerometer + Gyroscope**

* **Quantity:** 1
* **Voltage:** 3.3-5V
* **Current Consumption:** ~3.8 mA
* **Interface:** I2C
* **Description:** A 3-axis MPU providing acceleration and angular velocity data, used for angle correction and maintaining trajectory stability through PID control.

<img src="../assets/hardware_photos/MPU6050.jpg" width="300">

### **Component:** **Infrared Optocoupler Encoder**

* **Quantity:** 1
* **Voltage:** 3.3-5V
* **Current Consumption:** <20 mA
* **Description:** A laser interruption-based encoder used for detecting rotational position, enabling accurate measurement of distance and speed (odometry).

<img src="../assets/hardware_photos/encoder.jpg" width="300">

## 2.4 Power Supply and Regulation

### **Component:** **4.2V batteries/3.7V**

* **Quantity:** 2
* **Voltage:** 4.2V fully charged, 3.7 nominal
* **Capacity:** 8800 mAh, 5000 mAh
* **Discharge Current:** Up to 2A (depending on model)
* **Description:** Power supply for the robot's DC motors and auxiliary LED headlights. Specifically, **Li-ion 18650 Battery 4.2V 8800 mAh** and **Li-ion 18650 Battery 3.7V 5000 mAh** are used alternatively.

<img src="../assets/hardware_photos/4.2battery.jpg" width="300">

### **Component:** **Rechargable 9V Battery**

* **Quantity:** 1
* **Voltage:** 9V nominal
* **Capacity:** ~600 mAh
* **Description:** A standard 9V battery used for the sensors and Arduino MEGA unit.

<img src="../assets/hardware_photos/9Vbattery.jpg" width="300">

## 2.5 Miscellaneous Components

  ### **Component:** **Custom Chassis (RWD)**

  * **Quantity:** 1
  * **Material:** 3D printed (PLA)
  * **Description:** A 3D designed rear-wheel drive chassis designed to house motors, mounts, sensors, battery holders, mechanical differential, and the controller.

  [**3D Modeling**](./10_3d_modeling.md)


  ### **Component:** **Pushbutton**

  * **Quantity:** 1
  * **Voltage Handling:** Signal-level (logic input)
  * **Current:** <10 mA
  * **Description:** A momentary switch used to initiate the main program.

  <img src="../assets/hardware_photos/PUSHBUTTON.jpg" width="300">

  ### **Component:** **Self-Locking Push Button Switch**

  * **Quantity:** 1
  * **Voltage Handling:** Up to ~12V
  * **Current:** Up to ~500mA
  * **Description:** A switch for toggling between two states, serving as the main power switch for the robot.

 <img src="../assets/hardware_photos/SWITCH.jpg" width="300">

  ### **Component:** **LED**

  * **Quantity:** 1
  * **Forward Voltage:** ~3V
  * **Current:** ~10-20 mA
  * **Description:** A standard blue LED used as a visual indicator, signaling program readiness.

  <img src="../assets/hardware_photos/LEDBLUE.jpg" width="300">

  ### **Component:** **Headlight LEDs**

  * **Quantity:** 2
  * **Forward Voltage:** ~3V
  * **Current:** ~10-20 mA
  * **Description:** Standard white LEDs used as headlights for the bot.

  <img src="../assets/hardware_photos/HEADLIGHTled.jpg" width="300">

  ### **Component:** **Resistor**

  * **Quantity:** 2
  * **Resistance:** 330Ω, 10kΩ
  * **Description:** Current-limiting resistor for the LED indicator 
  and pushbutton.
  
  <img src="../assets/hardware_photos/resistor.jpg" width="300">

## 2.6 Removed Components

### **Component:** **TCS3200 Color Sensor (Removed in ViZio 2.0)**

* **Quantity:** 2
* **Voltage:** 2.7-5.5V
* **Power Consumption:** ~10-50 mA
* **Output:** Frequency proportional to color intensity
* **Description:** Side-mounted color recognition sensors used for detecting the magenta signal for parking. Converts color input to frequency output.

<img src="../assets/hardware_photos/tcs3200color.jpg" width="300">

### **Component:** **PixyCam 2.1 (Replaced by the OpenMV in ViZio IV)**

* **Quantity:** 1
* **Voltage:** 5V
* **Current Consumption:** ~140 mA
* **Interface:** ICSP
* **Description:** Smart vision sensor for real-time color and object recognition. Used for obstacle detection. For information regarding removal, please refer to: [Component Selection](#27-component-selection).

<img src="../assets/hardware_photos/pixycam.jpg" width="300">

## 2.7 Power Management  

The robot integrates a **dual-supply power architecture** to balance performance, efficiency, and protection of sensitive electronics:  

### Motor Power Supply  
* **Source:** **2x** [4.2V 18650 Li-ion Battery (8800 mAh)](#component-42v-batteries37v)  
* **Load:** [DC gearmotor](#component-hobby-gearmotor-with-481-gearbox), [L298N motor driver](#component-mini-l298n-motor-driver) + [LED Headlights](#component-headlight-leds) (optional)  
* **Current Draw:**  
  * ~120 mA average during motion  
  * +40 mA with headlights (2 × 20 mA)  
  * Up to ~1.6 A at stall (short bursts only)  
* **Estimated Runtime:**  
  * Continuous average draw (~120 mA): ~70 hours  
  * With headlights (~160 mA): ~55 hours  
  * Realistic mixed load with peaks: **~10–12 hours** of operation  

This battery is dedicated to propulsion and lighting, ensuring that motor noise and current surges do not affect sensor accuracy or logic circuits while maintaining long-lasting illumination for optimal operation.

### Logic and Sensor Power Supply

* **Source:** [Rechargeable 9 V Battery (~600 mAh)](#component-rechargable-9v-battery)
* **Load:** [MEGA 2560 Pro](#component--mega-2560-pro-embed) (voltage regulator) + [OpenMV](#component-openmv-h7-plus) + [MPU6050](#component-mpu6050-accelerometer--gyroscope) + [Color Sensor](#component-tcs3472-color-sensor) + [Ultrasonics](#component-hc-sr04-ultrasonic-sensors)  
* **Current Draw:**  
  * MEGA 2560 Pro: ~50–70 mA  
  * OpenMV H7: ~140 mA  
  * MPU6050: ~4 mA  
  * TCS3472 Color Sensor: ~0.24 mA  
  * Ultrasonic Sensors (x3): ~60 mA total  
  * **Total Average:** ~260–300 mA  
* **Estimated Runtime:**  
  * ~600 mAh / 280 mA ≈ **2 hours** continuous operation  
  * Recommended use time ≈ **1 hour** for best performance

This battery ensures clean and stable voltage delivery for sensors and the controller.  

### Voltage Regulation

* The **MEGA 2560 Pro** regulates the 9V input to 5V for its logic and peripherals.  
* I²C devices (MPU6050, color sensors) operate safely at 3.3–5V.  

### Switching and Protection

* **[Main Switch](#component-self-locking-push-button-switch)**: Master power cutoff for the system.  
* **[Pushbutton](#component-pushbutton)**: Starts the main program after initialization.  
* **[330Ω and 10kΩ Resistor](#component-resistor)**: Ensures safe current limiting for the LED indicator and facilitates pushbutton grounding.
* **[LED Indicator](#component-led)**: Shows program readiness.  
* **[Headlight LEDs (x2)](#component-headlight-leds)**: White LEDs for illumination during operation.  

**Summary of Power Autonomy**  

* **Motors (4.2V Li-ion):** 12–15 hours (practical runtime)
* **Logic & Sensors (9V battery):** ~2 hours (main limiting factor)  

The system's autonomy is therefore governed by the 9V supply, after which recharging or replacement is required.

## 2.8 Circuit Design

The robot utilizes a custom-made PCB, avoiding unnecessary clutter and improving overall order.

<img src="./../assets/hardware_photos/SolderedPCB.jpg" height="500">

You can also download the fixed circuit files: [kicad_pcb](../src/kicad_pcb).

---

[Back to Main README.md Index](../README.md)
