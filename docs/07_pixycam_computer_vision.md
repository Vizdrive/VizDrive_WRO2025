# 7. Vision and Distance Sensing

VizDrive uses the PixyCam 2.1 to detect and react to obstacles in real time.

---

## 7.1 PixyCam 2.1 for Vision-Based Obstacle Evasion

The **PixyCam 2.1** is an intelligent, compact vision sensor that provides rapid object recognition based on pre-trained color signatures. Its integrated processor significantly offloads computational burden from the main microcontroller, enabling real-time obstacle detection.

### Global Object

```cpp
Pixy2 pixy; // PixyCam 2.1 object for communication and control
````

### Initialization (`void initVision()`)

* **Purpose**: Establishes communication with and initializes the PixyCam 2.1 module.
* **Operation**:
  * `pixy.init();`: Calls the initialization function from the `Pixy2` library. This function typically configures the SPI interface and prepares the camera for block detection.

### Region of Interest (ROI)

To focus detection on relevant areas directly in the robot's path, a specific Region of Interest (ROI) is defined within the PixyCam's horizontal field of view. The camera's standard resolution is 316x208 pixels.

```cpp
const int LEFT_BOUND = 316 / 3; // Left X-coordinate boundary for ROI (approx. 105 pixels)
const int RIGHT_BOUND = 2 * 316 / 3; // Right X-coordinate boundary for ROI (approx. 210 pixels)
```

These bounds define the central third of the camera's horizontal output, ensuring the robot primarily reacts to obstacles directly ahead.

### Obstacle Detection (`bool obstacleDetected()`)

* **Purpose**: Determines if any detected colored object ("block") is currently present within the robot's defined Region of Interest (ROI).
* **Operation**:
    1. `pixy.ccc.getBlocks();`: Commands the PixyCam to process the current frame and retrieve a list of detected blocks (objects with assigned color signatures).
    2. `if (pixy.ccc.numBlocks == 0) return false;`: If no blocks are reported by the PixyCam, no obstacles are present, and the function returns `false`.
    3. **Iterative ROI Check**: The function then iterates through all detected `pixy.ccc.blocks`. For each block:
          * `int x = pixy.ccc.blocks[i].m_x;`: Retrieves the X-coordinate of the block's center.
          * `if (x > LEFT_BOUND && x < RIGHT_BOUND) { return true; }`: Checks if the block's X-coordinate falls within the `LEFT_BOUND` and `RIGHT_BOUND` of the ROI. If an object is found within this range, it is considered an obstacle, and the function immediately returns `true`.
    4. If the loop completes without identifying any blocks within the ROI, the function returns `false`.

### Obstacle Evasion Logic (`void handleEvasion()`)

* **Purpose**: Manages the robot's steering to actively evade a detected obstacle, continuing until the path is clear.
* **Operation**:
    1. **Continuous Evasion Loop**: The function operates within a `do-while (obstacleDetected())` loop. This ensures the robot continues to execute evasion maneuvers as long as an obstacle remains within its ROI.
    2. `pixy.ccc.getBlocks();`: At the beginning of each loop iteration, updated block information is retrieved. If `pixy.ccc.numBlocks` is 0 (no blocks detected), the evasion loop breaks.
    3. **Largest Block Selection in ROI**:
          * The function iterates through all detected blocks to find the largest one (based on `width * height`) that is also within the ROI. This ensures the robot prioritizes reacting to the most prominent obstruction.
    4. **Maneuver Based on Signature**: If a valid block is selected:
          * `int signature = pixy.ccc.blocks[selectedIndex].m_signature;`: The color signature of the selected block is read.
          * **Signature 1 (Red)**: `if (signature == 1) { setSteeringAngle(SERVO_RIGHT); }` - The robot steers sharply to the right.
          * **Signature 2 (Green)**: `else if (signature == 2) { setSteeringAngle(SERVO_LEFT); }` - The robot steers sharply to the left.
          * **Other Signatures**: `else { setSteeringAngle(SERVO_STRAIGHT); }` - If an unexpected signature is encountered, the robot attempts to steer straight as a fallback.
    5. `safeDelay(100);`: A brief, non-blocking delay is introduced to allow the robot to respond to steering adjustments and for the PixyCam to acquire new readings. This "safe delay" ensures that crucial background tasks like orientation updates continue to run.
    6. **Post-Evasion**: Once the `do-while` loop condition (`obstacleDetected()`) becomes `false` (indicating the path is clear), `setSteeringAngle(SERVO_STRAIGHT);` is called to return the steering to the neutral position, allowing the robot to resume its primary mission.

### Debugging Function (`void debugPixy()`)

* **Purpose**: Provides detailed real-time information about blocks detected by the PixyCam to the Serial Monitor, aiding in calibration and troubleshooting.
* **Operation**: It iterates through `pixy.ccc.numBlocks` and prints the index, X/Y coordinates, and color signature of each block. It also explicitly indicates whether each block falls within the defined ROI (`LEFT_BOUND` and `RIGHT_BOUND`). This function is typically commented out in the main code for performance optimization but is invaluable during development.

---

[Back to Main README.md Index](https://www.google.com/search?q=../README.md)
