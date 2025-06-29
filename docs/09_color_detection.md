# 9. Color Sensors for Circuit Perception

VizDrive's autonomous navigation and mission completion rely heavily on its ability to perceive crucial color cues from the competition circuit. This section provides a detailed explanation of the various color sensors implemented, their specific functions, communication protocols, and the associated data processing techniques.

---

## 9.1 Floor Color Sensor (I2C - Adafruit TCS34725)

A dedicated TCS34725 color sensor is mounted on the underside of the robot. Its primary function is to detect colored lines on the circuit mat, which serve as strategic indicators for turns.

### Global Object

```cpp
static Adafruit_TCS34725 floorTcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X); // I2C color sensor object
```

This object is initialized with an integration time of 2.4 milliseconds and a gain of 1X, parameters that control the sensor's light exposure and sensitivity.

### Communication

The TCS34725 sensor communicates with the Arduino Mega via the **I2C (Inter-Integrated Circuit)** bus. This two-wire serial interface (SDA and SCL) allows for efficient data exchange.

### Color Thresholds for Floor Detection

These empirically determined thresholds are crucial for identifying specific colors (Blue and Orange) after the raw sensor data has been normalized. They represent the minimum required intensity for a color channel or the maximum allowed intensity for others.

```cpp
// For BLUE color detection (expected intense blue channel, and mild red and green colors)
const int BLUE_RED_THRESHOLD = 120;
const int BLUE_GREEN_THRESHOLD = 120;
const int BLUE_BLUE_THRESHOLD = 150;

// For ORANGE color detection (expected intense red and green colors, and mildly intense blue colors)
const int ORANGE_RED_THRESHOLD = 150;
const int ORANGE_GREEN_THRESHOLD = 150;
const int ORANGE_BLUE_THRESHOLD = 120;
```

### Initialization (`void initColorSensors()`)

* **Purpose**: Initializes the floor-facing I2C color sensor and calls the initialization for the digital wall color sensors.
* **Operation**:
  * `floorTcs.begin();`: This command initiates the I2C communication with the TCS34725 sensor and configures it for operation.

### `bool detectFloorColor()`

* **Purpose**: Detects specific turn-signal colors (Blue or Orange) on the track based on the robot's expected turning direction.
* **Operation**:
    1. **Raw Data Acquisition**: `uint16_t r,g,b,c; floorTcs.getRawData(&r,&g,&b,&c);` retrieves the raw red, green, blue, and clear light intensity values from the TCS34725 sensor.
    2. **Sensor Validation**: `if (!c) return false;`: If the clear light reading (`c`) is zero, it indicates a sensor error or insufficient ambient light, and the function returns `false`.
    3. **Color Normalization (TCS34725)**:
          * `int rn=r*255/c, gn=g*255/c, bn=b*255/c;`: The raw R, G, B values are normalized to a 0-255 scale by dividing each channel by the clear light value (`c`). This normalization is essential to compensate for variations in ambient light conditions, ensuring that color detection remains consistent regardless of light intensity. For example, a red object will always have a high "redness" ratio relative to its total brightness, even if the overall light is dim.
    4. **Color Recognition based on `direction`**:
          * If the global `direction` is `-1` (indicating an expected left turn), the function checks for **Blue** by comparing `bn` against `BLUE_BLUE_THRESHOLD` (high blue) and `rn`, `gn` against `BLUE_RED_THRESHOLD`, `BLUE_GREEN_THRESHOLD` (low red and green).
          * If the global `direction` is `+1` (indicating an expected right turn), the function checks for **Orange** by comparing `rn`, `gn` against `ORANGE_RED_THRESHOLD`, `ORANGE_GREEN_THRESHOLD` (high red and green) and `bn` against `ORANGE_BLUE_THRESHOLD` (low blue).
    5. Returns `true` if the expected color for the current turn `direction` is detected, otherwise `false`.

---

## 9.2 Wall Color Sensors (Digital - TCS3200)

Two digital TCS3200 color sensors are strategically mounted on the sides of the robot. These sensors are primarily used for detecting the magenta "parking signal" on walls during the final phase of the mission.

### Hardware Definitions

The following pins are used to control and read data from the left and right TCS3200 sensors:

```cpp
// ------------- Wall sensors (digital TCS3200) -------------
#define S0_L 10 // Left sensor frequency scaling pin S0
#define S1_L 12 // Left sensor frequency scaling pin S1
#define S2_L 16 // Left sensor color filter pin S2
#define S3_L 18 // Left sensor color filter pin S3
#define OUT_L 48 // Left sensor output frequency pin
#define S0_R 51 // Right sensor frequency scaling pin S0
#define S1_R 49 // Right sensor frequency scaling pin S1
#define S2_R 50 // Right sensor color filter pin S2
#define S3_R 52 // Right sensor color filter pin S3
#define OUT_R 53 // Right sensor output frequency pin
```

### Global Structure and Color Thresholds

```cpp
struct RGB { // A simple struct to hold normalized RGB values
  int red;
  int green;
  int blue;
}

// For MAGENTA color detection (expected intense red and blue colors, and mildly intense green colors)
const int MAGENTA_RED_THRESHOLD = 150;
const int MAGENTA_GREEN_THRESHOLD = 120; // Lower green intensity expected for magenta
const int MAGENTA_BLUE_THRESHOLD = 150;
```

### Initialization (`void initWallColorSensors()`)

* **Purpose**: Configures the digital I/O pins for the left and right TCS3200 color sensors.
* **Operation**:
    1. Sets all `S0_L`, `S1_L`, `S2_L`, `S3_L`, `OUT_L` pins for the left sensor and `S0_R`, `S1_R`, `S2_R`, `S3_R`, `OUT_R` pins for the right sensor as `OUTPUT` or `INPUT` as appropriate.
    2. `digitalWrite(S0_L, HIGH); digitalWrite(S1_L, HIGH);`: The `S0` and `S1` pins control the output frequency scaling of the TCS3200. Setting both to `HIGH` typically configures the sensor to output at 100% frequency scale, providing maximum resolution for readings.

### `RGB readTCS3200(int s2, int s3, int outPin)`

* **Purpose**: Reads the raw R, G, B color components from a single TCS3200 sensor by measuring its output frequency for each color filter.
* **Parameters**:
  * `s2`, `s3`: The pins used to select the color filter (Red, Green, or Blue) on the TCS3200.
  * `outPin`: The output frequency pin of the TCS3200.
* **Operation**:
    1. **Filter Selection and Frequency Measurement**:
          * **Red**: `digitalWrite(s2, LOW); digitalWrite(s3, LOW);` (selects red filter). `delay(5);` `int redRaw = pulseIn(outPin, LOW);` (measures the pulse width for red light).
          * **Green**: `digitalWrite(s2, HIGH); digitalWrite(s3, HIGH);` (selects green filter). `delay(5);` `int greenRaw = pulseIn(outPin, LOW);`
          * **Blue**: `digitalWrite(s2, LOW); digitalWrite(s3, HIGH);` (selects blue filter). `delay(5);` `int blueRaw = pulseIn(outPin, LOW);`
          * **Note on `pulseIn()`**: The `pulseIn()` function returns the duration of a pulse in microseconds. For the TCS3200, this duration is *inversely proportional* to the light intensity for the selected color. Higher light intensity results in higher output frequency, which means a *shorter* pulse duration.
    2. **Normalization and Inversion (TCS3200)**:
          * `int maxRaw = max(max(redRaw, greenRaw), blueRaw); if (maxRaw == 0) maxRaw = 1;`: Finds the maximum raw pulse width among the three colors. This maximum is used as a reference for normalization. A check for `maxRaw == 0` prevents division by zero.
          * `rgb.red = map(constrain(redRaw, 0, maxRaw), 0, maxRaw, 255, 0);`:
              * `constrain(redRaw, 0, maxRaw)`: Ensures the raw reading is within a valid range relative to `maxRaw`.
              * `map(..., 0, maxRaw, 255, 0)`: This is the crucial **inversion and scaling** step. Since `pulseIn` returns a duration (where a *smaller* value means *more* light), the `map` function is used to convert this inverse relationship into a standard intensity scale (0-255). A raw reading of `0` (maximum light/frequency) is mapped to `255` (maximum intensity), and `maxRaw` (minimum light/frequency) is mapped to `0` (minimum intensity). This process is repeated for the green and blue channels.
    3. Returns an `RGB` struct containing the normalized and inverted R, G, B values.

### `bool detectWallMagenta()`

* **Purpose**: Detects the presence of a magenta color on the side walls, which serves as the signal for the robot's arrival at the parking zone.
* **Operation**:
    1. **Sensor Selection**:
          * Based on the global `direction` variable, the appropriate side sensor's `S2`, `S3`, and `OUT` pins are selected. If `direction` is `-1` (left turn, implying parking on the right wall), the right sensor's pins (`S2_R`, `S3_R`, `OUT_R`) are chosen. Otherwise, the left sensor's pins are used.
    2. `RGB rgb = readTCS3200(s2, s3, outPin);`: Reads the normalized RGB values from the selected wall-facing sensor.
    3. **Color Recognition**:
          * `return (rgb.red > MAGENTA_RED_THRESHOLD && rgb.blue > MAGENTA_BLUE_THRESHOLD && rgb.green < MAGENTA_GREEN_THRESHOLD);`: Returns `true` if the detected color matches the defined `MAGENTA` thresholds (high red and blue intensity, relatively low green intensity). Otherwise, returns `false`. This function is activated once `lapCompletedCount` reaches a certain threshold (e.g., 4 in the current code, not 12 as per the original draft).

---

## 9.3 Unified Color Detection API (`bool colorDetected()`)

To simplify the main program loop's logic, a unified function is provided to check for the currently relevant color signal.

* **Purpose**: Dynamically switches between floor line detection and wall parking signal detection based on the robot's mission progress.
* **Operation**:
  * `if (lapCompletedCount < 4) return detectFloorColor();`: During the initial phases of the mission (i.e., when `lapCompletedCount` is less than 4), the robot focuses on detecting blue or orange lines on the floor for navigation turns.
  * `return detectWallMagenta();`: Once `lapCompletedCount` is 4 or more, the robot's focus shifts to detecting the magenta parking signal on the side walls.

---

[Back to Main README.md Index](https://www.google.com/search?q=../README.md)
