// libraries
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <NewPing.h>
#include <Pixy2.h>

// GLOBAL VARIABLES FROM ALL MODULES

// main.ino
const int buttonPin = A1;
const int ledPin = 39;
const int motorSpeed = 135;
unsigned long lastUpdateTime = 0;

// color_detection.cpp
struct RGB {
  int red;
  int green;
  int blue;
};

// Color thresholds
// For MAGENTA color detection:
// Expected intense red and blue colors, and mildly intense green colors.
const int MAGENTA_RED_THRESHOLD = 150;
const int MAGENTA_GREEN_THRESHOLD = 120;
const int MAGENTA_BLUE_THRESHOLD = 150;
// For BLUE color detection:
// Expected intense blue channel, and mild red and green colors
const int BLUE_RED_THRESHOLD = 80;
const int BLUE_GREEN_THRESHOLD = 80;
const int BLUE_BLUE_THRESHOLD = 90;
// For ORANGE color detection:
// Expected intense red and green colors, and mildly intense blue colors.
const int ORANGE_RED_THRESHOLD = 115;
const int ORANGE_GREEN_THRESHOLD = 100;
const int ORANGE_BLUE_THRESHOLD = 70;

// ------------- Floor color sensor (I2C) -------------
static Adafruit_TCS34725 floorTcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
int direction = 0;  // +1: right, -1: left (initialized in setupStateLogic)

// ------------- Wall sensors (digital TCS3200) -------------
#define S0_L 10
#define S1_L 12
#define S2_L 16
#define S3_L 18
#define OUT_L 48
#define S0_R 51
#define S1_R 49
#define S2_R 50
#define S3_R 52
#define OUT_R 53

// movement.cpp
#define MOTOR_INA_PWM 4
#define MOTOR_INB_PWM 5
#define SERVO_PIN 6
#define ENCODER_PIN 2

const int SERVO_STRAIGHT = 82;
const int SERVO_MAX = 35;
const int SERVO_LEFT = SERVO_STRAIGHT - SERVO_MAX;   // minimum servo angle
const int SERVO_RIGHT = SERVO_STRAIGHT + SERVO_MAX;  // maximum servo angle

Servo steeringServo;
volatile unsigned long encoderPulseCount = 0;
static const unsigned long PARKING_PULSES = 200;  // Adjust according to distance

// orientation.cpp
Adafruit_MPU6050 mpu;

static float yaw = 0.0;
static float targetYaw = 0.0;
static float previousError = 0.0;

// PID error variables
const float Kp = 2.0;
const float Kd = 1.2;
const float Ki = 0.0;
float gyroZ_offset = 0.0;

// state_logic.cpp
// int direction = 0; // +1: right, -1: left (already defined in color_detection.cpp)
unsigned int lapTurnCount = 0, lapCompletedCount = 0;
bool parkingMode = false, SystemShutdown = false;
bool turningInProgress = false;
float turnTargetYaw = 0.0;
const float TURN_THRESHOLD = 6.0;  // tolerance for turns in degrees

// ultrasonic.cpp
#define US_LEFT_TRIG A9
#define US_LEFT_ECHO A11
#define US_RIGHT_TRIG 47
#define US_RIGHT_ECHO 28
#define MAX_DISTANCE 200

int diff_threshold = 8;
int num_measurements = 3;  // number of measurements for average

NewPing sonarLeft(US_LEFT_TRIG, US_LEFT_ECHO, MAX_DISTANCE);
NewPing sonarRight(US_RIGHT_TRIG, US_RIGHT_ECHO, MAX_DISTANCE);

// vision.cpp
Pixy2 pixy;
// Region of Interest (ROI) bounds
const int LEFT_BOUND = 316 / 3;
const int RIGHT_BOUND = 2 * 316 / 3;


// FUNCTION PROTOTYPES

void safeDelay(unsigned long ms);
void waitForStartButton();

// color_detection.h
void initColorSensors();
bool colorDetected();
// color_detection.cpp
void initWallColorSensors();
RGB readTCS3200(int s2, int s3, int outPin);
bool detectFloorColor();
bool detectWallMagenta();

// movement.h
void initMovement();
void driveForward(int speed);
void driveBackward(int speed);
void stopMotors();
void stopBrake();
void setSteeringAngle(int angle);
void encoderISR();
void performParking();

// orientation.h
void initOrientation();
void updateOrientation(float dt);
void keepOrientation();
void setTargetYaw(float angle_deg);
float getCurrentYaw();
float getYawError();

// state_logic.h
void initStateLogic();
void updateLapCount();
void handleColorAction();
void checkForShutdown();

// ultrasonic.h
void initUltrasonic();
void recentreUltrasonic();
// ultrasonic.cpp
int getAvgDistance(NewPing& sonar);

// vision.h
void initVision();
bool obstacleDetected();
void handleEvasion();
// vision.cpp
void debugPixy();


// SETUP
void setup() {
  delay(3000);
  Serial.println("Initializing serial monitor...");
  Serial.begin(115200);
  Serial.println("Initializing movement...");
  initMovement();
  Serial.println("Initializing orientation control...");
  initOrientation();
  Serial.println("Initializing artificial vision...");
  initVision();
  Serial.println("Initializing ultrasonic sensors...");
  initUltrasonic();
  Serial.println("Initializing color sensors...");
  initColorSensors();
  Serial.println("Initializing state logic...");
  initStateLogic();
  waitForStartButton();
  lastUpdateTime = millis();
}

// LOOP
void loop() {
  unsigned long now = millis();
  if (now - lastUpdateTime < 25) return;
  float dt = (now - lastUpdateTime) / 1000.0;
  lastUpdateTime = now;

  driveForward(motorSpeed);
  updateOrientation(dt);
  keepOrientation();

  if (obstacleDetected()) {
    handleEvasion();
  }

  if (colorDetected()) {
    handleColorAction();
  }

  updateLapCount();
  checkForShutdown();
}

// FUNCTION IMPLEMENTATIONS

// main.ino
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateOrientation((millis() - lastUpdateTime) / 1000.0);
    if (obstacleDetected()) handleEvasion();
    if (colorDetected()) handleColorAction();
    updateLapCount();
  }
}

void waitForStartButton() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  while (!digitalRead(buttonPin)) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    Serial.println("Waiting for START BUTTON...");
  }
  digitalWrite(ledPin, LOW);
}

// COLOR DETECTION

void initColorSensors() {  // setup
  floorTcs.begin();
  initWallColorSensors();  // Initialize digital wall sensors here too
}

// Detects blue or orange, according to turn direction and normalizes raw measurements (RGB 0-255)
bool detectFloorColor() {
  uint16_t r, g, b, c;
  floorTcs.getRawData(&r, &g, &b, &c);
  if (!c) return false;
  int rn = r * 255 / c, gn = g * 255 / c, bn = b * 255 / c;

  if (direction < 0 && bn > BLUE_BLUE_THRESHOLD && rn < BLUE_RED_THRESHOLD && gn < BLUE_GREEN_THRESHOLD) return true;        // blue for left
  if (direction > 0 && rn > ORANGE_RED_THRESHOLD && gn > ORANGE_GREEN_THRESHOLD && bn < ORANGE_BLUE_THRESHOLD) return true;  // orange for right
  return false;
}

void initWallColorSensors() {
  pinMode(S0_L, OUTPUT);
  pinMode(S1_L, OUTPUT);
  pinMode(S2_L, OUTPUT);
  pinMode(S3_L, OUTPUT);
  pinMode(OUT_L, INPUT);
  pinMode(S0_R, OUTPUT);
  pinMode(S1_R, OUTPUT);
  pinMode(S2_R, OUTPUT);
  pinMode(S3_R, OUTPUT);
  pinMode(OUT_R, INPUT);
  digitalWrite(S0_L, HIGH);
  digitalWrite(S1_L, HIGH);
  digitalWrite(S0_R, HIGH);
  digitalWrite(S1_R, HIGH);
}

RGB readTCS3200(int s2, int s3, int outPin) {
  RGB rgb;

  // Read red
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(5);
  int redRaw = pulseIn(outPin, LOW);

  // Read green
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(5);
  int greenRaw = pulseIn(outPin, LOW);
  // Read blue
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delay(5);
  int blueRaw = pulseIn(outPin, LOW);
  // Determine the maximum to normalize (invert frequency)
  int maxRaw = max(max(redRaw, greenRaw), blueRaw);
  if (maxRaw == 0) maxRaw = 1;  // avoid division by 0

  rgb.red = map(constrain(redRaw, 0, maxRaw), 0, maxRaw, 255, 0);
  rgb.green = map(constrain(greenRaw, 0, maxRaw), 0, maxRaw, 255, 0);
  rgb.blue = map(constrain(blueRaw, 0, maxRaw), 0, maxRaw, 255, 0);
  return rgb;
}

// After 3 turns, detects magenta walls: uses right color sensor if direction=-1, and left if =+1
bool detectWallMagenta() {
  // Fix for the S0/S1 being global defines in color_detection.cpp and not function parameters
  // The original code used global #defines for S0_L, S1_L, S0_R, S1_R which are not passed.
  // The readTCS3200 function only takes s2, s3, outPin.
  // Given S0 and S1 are set to HIGH in initWallColorSensors() and never changed,
  // we assume they are constant and don't need to be passed or changed.
  int s2 = (direction < 0 ? S2_R : S2_L);
  int s3 = (direction < 0 ? S3_R : S3_L);
  int outPin = (direction < 0 ? OUT_R : OUT_L);
  RGB rgb = readTCS3200(s2, s3, outPin);

  return (rgb.red > MAGENTA_RED_THRESHOLD && rgb.blue > MAGENTA_BLUE_THRESHOLD && rgb.green < MAGENTA_GREEN_THRESHOLD);
}

// Unified API. If completed 4 turns, start detecting sides
bool colorDetected() {
  if (lapCompletedCount < 12) return detectFloorColor();
  return detectWallMagenta();
}


// MOVEMENT

// Setup for movement
void initMovement() {
  pinMode(MOTOR_INA_PWM, OUTPUT);
  pinMode(MOTOR_INB_PWM, OUTPUT);
  steeringServo.attach(SERVO_PIN);
  setSteeringAngle(SERVO_STRAIGHT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
}

// Movement functions (turn engine on/off)
void driveForward(int speed) {
  analogWrite(MOTOR_INA_PWM, speed);
  digitalWrite(MOTOR_INB_PWM, LOW);
}

void driveBackward(int speed) {
  analogWrite(MOTOR_INA_PWM, LOW);
  digitalWrite(MOTOR_INB_PWM, speed);
}

void stopMotors() {
  digitalWrite(MOTOR_INA_PWM, LOW);
  digitalWrite(MOTOR_INB_PWM, LOW);
}

void stopBrake() {
  digitalWrite(MOTOR_INA_PWM, HIGH);
  digitalWrite(MOTOR_INB_PWM, HIGH);
}

// Steering function (set the angle of the servomotor)
void setSteeringAngle(int angle) {
  steeringServo.write(angle);
}

// Function for encoder pulse count
void encoderISR() {
  encoderPulseCount++;
}

// Parking maneuver function
void performParking() {
  // Reset encoderPulseCount
  encoderPulseCount = 0;
  // Turn servomotor for parking maneuver
  int parkingAngle = (direction < 0 ? SERVO_RIGHT : SERVO_LEFT);
  setSteeringAngle(parkingAngle);
  delay(100);
  // Drive backwards till completing PARKING_PULSES
  while (true) {
    driveBackward(150);
    unsigned long cnt = encoderPulseCount;
    if (cnt >= PARKING_PULSES) break;
  }
}

// ORIENTATION CONTROL

void initOrientation() {  // setup
  Wire.begin();
  if (!mpu.begin())
    while (1)
      ;  // if the MPU is not detected, stops the program
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  // Calibration of Z axis offset
  float sum = 0.0;
  int samples = 250;  // Requiered samples
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }
  gyroZ_offset = sum / samples;  // Calculates the average offset
}

// Updates the accumulated yaw angle
void updateOrientation(float dt) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyroZ_deg = (g.gyro.z - gyroZ_offset) * 57.2958;  // converts radians to degrees
  if (abs(gyroZ_deg) > 0.1) {                             // threshold for noise reduction
    yaw += gyroZ_deg * dt;                                // angular speed times time
  }
}

// PID control function
void keepOrientation() {
  float error = targetYaw - yaw;
  float derivative = error - previousError;  // error variance

  int correction = SERVO_STRAIGHT + (int)(Kp * error + Kd * derivative);  // adds the PID correction to the straight angle
  correction = constrain(correction, SERVO_LEFT, SERVO_RIGHT);            // limits the correction to the maximum and minimum servo angles
  setSteeringAngle(correction);

  previousError = error;
}

// Function to change the target angle (in 90° turns)
void setTargetYaw(float angle_deg) {
  targetYaw = angle_deg;
}

// Auxiliar API functions
float getCurrentYaw() {
  return yaw;
}

float getYawError() {
  return targetYaw - yaw;
}

// STATE LOGIC
void initStateLogic() {
  int dl = getAvgDistance(sonarLeft);
  int dr = getAvgDistance(sonarRight);
  // direction = (dl > dr) ? -1 : +1;  // direction established by distance to walls
  direction = -1;
  setTargetYaw(0.0);  // start in straight orientation
}

void updateLapCount() {
  if (lapTurnCount >= 4) {  // While laps are not completed
    lapCompletedCount++;
    lapTurnCount = 0;
  }
}

void handleColorAction() {
  if (lapCompletedCount < 12) {
    if (!turningInProgress && detectFloorColor()) {
      turnTargetYaw = (turnTargetYaw + (90.0 * direction * -1));
      setTargetYaw(turnTargetYaw);
      digitalWrite(ledPin, HIGH);
      turningInProgress = true;
    }

    if (turningInProgress && abs(abs(targetYaw)-abs(yaw)) < TURN_THRESHOLD) {
      digitalWrite(ledPin, LOW);
      turningInProgress = false;
      digitalWrite(ledPin, LOW);
      lapTurnCount++;
    }
  } else {
    if (detectWallMagenta()) {
      performParking();
      SystemShutdown = true;
    }
  }
}

void checkForShutdown() {
  if (SystemShutdown) {
    stopBrake();
    while (1)
      ;
  }
}

// ULTRASONIC DISTANCE MEASUREMENTS

// Get the average distance of 3 measurements
int getAvgDistance(NewPing& sonar) {
  int d = sonar.ping_cm();
  safeDelay(5);
  if (d > 100) {
    return 0;
  }
  return d;
}

void initUltrasonic() {
  // Nothing specific to initialize for NewPing beyond object creation
}

// For recentring after avoiding an obstacle
void recentreUltrasonic() {
  int dLeft = getAvgDistance(sonarLeft);
  int dRight = getAvgDistance(sonarRight);
  int diff = dLeft - dRight;
  if (abs(diff) > diff_threshold) {
    if (diff > 0) setSteeringAngle(SERVO_RIGHT);  // right
    else setSteeringAngle(SERVO_LEFT);            // left
  } else {
    setSteeringAngle(SERVO_STRAIGHT);  // If within threshold, go straight
  }
}

// ARTIFICIAL VISION WITH PIXYCAM

// Initialize Pixy2
void initVision() {
  pixy.init();
}

// Checks if any block is inside the ROI
bool obstacleDetected() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks == 0) {
    Serial.println("No se encuentran bloques... ");
    return false;
  }
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int x = pixy.ccc.blocks[i].m_x;
    if (x > LEFT_BOUND && x < RIGHT_BOUND) {
      Serial.println("Bloque en el área de interés... ");
      return true;  // block inside ROI
    }
  }
  return false;
}

// Evasion based on color signature within ROI
void handleEvasion() {
  debugPixy(); // Uncomment to show block data during evasion
  do {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks == 0) {
      Serial.println("No bloques detectados. ");
      break;
    }

    int selectedIndex = -1;
    int largestArea = 0;  // Select the largest block inside ROI
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int x = pixy.ccc.blocks[i].m_x;
      Serial.print("x: ");
      Serial.println(x);
      if (x > LEFT_BOUND && x < RIGHT_BOUND) {
        int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
        if (area > largestArea) {
          largestArea = area;
          selectedIndex = i;
        }
      }
    }

    if (selectedIndex == -1) break;  // no valid block in ROI

    int signature = pixy.ccc.blocks[selectedIndex].m_signature;  // Maneuver according to signature
    if (signature == 1) {
      setSteeringAngle(SERVO_RIGHT);
    } else if (signature == 2) {
      setSteeringAngle(SERVO_LEFT);
    } else {
      setSteeringAngle(SERVO_STRAIGHT);
    }


    safeDelay(100);  // brief pause

  } while (obstacleDetected());
  setSteeringAngle(SERVO_STRAIGHT);  // Return to straight after evasion
}

void debugPixy() {
  Serial.println("Pixy ROI Blocks:");
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int x = pixy.ccc.blocks[i].m_x;
    int y = pixy.ccc.blocks[i].m_y;
    int sig = pixy.ccc.blocks[i].m_signature;
    Serial.print("Block ");
    Serial.print(i);
    Serial.print(" | X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Signature: ");
    Serial.print(sig);
    if (x > LEFT_BOUND && x < RIGHT_BOUND) {
      Serial.println(" -> IN ROI");
    } else {
      Serial.println(" -> out of ROI");
    }
  }
  Serial.println("---------------------");
}