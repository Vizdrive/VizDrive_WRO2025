#include <Wire.h> // i2c connection
#include <Adafruit_MPU6050.h> // mpu6050
#include <Adafruit_Sensor.h> 
#include <Adafruit_TCS34725.h>  // color sensor
#include <Servo.h>  // servomotor
#include <NewPing.h>  // ultrasonic sensors

// GLOBAL VARIABLES

// 1. Mobility Functionalities

// DC motor PWM pins
#define MOTOR_INA_PWM 4
#define MOTOR_INB_PWM 5
int motorSpeed = 200; // motor PWM value for velocity
// Servomotor pin
#define SERVO_PIN     6

// Constant servo angles
const int SERVO_STRAIGHT = 85;
int SERVO_MAX = 30; // maximum angle of servo's rotation
int SERVO_LEFT  = SERVO_STRAIGHT - SERVO_MAX;
int SERVO_RIGHT = SERVO_STRAIGHT + SERVO_MAX;

// Defining the servomotor
Servo steeringServo;

// IR encoder pin
#define ENCODER_PIN   2
volatile unsigned long encoderPulseCount = 0; // volatile variable for pulse count

const int WHEEL_PERIMETER = 22; // cm
const int PULSES_PER_REVOLUTION = 16; // PPR
const float PULSES_PER_CM = PULSES_PER_REVOLUTION/WHEEL_PERIMETER;

const float cos30deg = 0.866025;

// 2. PID Control and Orientation

// Defining the MPU6050 with Adafruit library
Adafruit_MPU6050 mpu;
float gyroZ_offset = 0.0; // MPU6050 calibrated offset

static float yaw = 0.0; // variable for accumulated yaw
static float targetYaw = 0.0; // target yaw for PID correction
const int samples = 250;  // samples for MPU6050 calibration

static float previousError = 0.0; // previous error for the time derivative correction

// PID constants
const float Kp = 1.8; // proportional gain
const float Kd = 1.2; // derivative gain
const float Ki = 0.0; // integral gain (inactive to avoid wind-up and over-correction)

// 3. Color Detection

// Color detection thresholds
const int MAGENTA_RED_THRESHOLD =   150;  // side-mounted color sensors for magenta walls detection
const int MAGENTA_GREEN_THRESHOLD = 120;
const int MAGENTA_BLUE_THRESHOLD =  150;

// STRUCTS
struct RGB {  // struct to store side-mounted color sensors' reads.
  int red;
  int green;
  int blue;
};

// color sensor located at the bottom
const int BLUE_RED_THRESHOLD =   80;   // blue color thresholds
const int BLUE_GREEN_THRESHOLD = 85;
const int BLUE_BLUE_THRESHOLD =  75;
const int ORANGE_RED_THRESHOLD =   100; // orange color thresholds
const int ORANGE_GREEN_THRESHOLD = 100;
const int ORANGE_BLUE_THRESHOLD =  70;

// Defining the i2c color sensor as "floorTcs"
static Adafruit_TCS34725 floorTcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// Defining the digital pins for the side-mounted sensors
const int S0_L = 10; const int S1_L = 12; const int S2_L = 16; const int S3_L = 18; const int OUT_L = 48; // left
const int S0_R = 51; const int S1_R = 49; const int S2_R = 50; const int S3_R = 52; const int OUT_R = 53; // right

// 4. Ultrasonic Sensor Distance Measurement

// Left ultrasonic sensor trigger and echo pins
#define US_LEFT_TRIG 47
#define US_LEFT_ECHO 28
// Right ultrasonic sensor trigger and echo pins
#define US_RIGHT_TRIG A9
#define US_RIGHT_ECHO A11

// Max detection distance for the definition of the ultrasonic sensors
#define MAX_DISTANCE 200

// Defining the ultrasonic sensors with NewPing library
NewPing sonarLeft(US_LEFT_TRIG, US_LEFT_ECHO, MAX_DISTANCE);
NewPing sonarRight(US_RIGHT_TRIG, US_RIGHT_ECHO, MAX_DISTANCE);

const int avoidance_amount = 8;

// 5. State Logic
unsigned int lapTurnCount = 0, lapCompletedCount = 0; // variables for lap counting
bool parkingMode = false, SystemShutdown = false; // variables for parking and shutdown
bool turningInProgress = false; // variable to avoid duplicate color detections

bool correctionState = false;  // variable to avoid repetitive ultrasonic corrections in one lap
unsigned long lastCorrectionTime = 0; // variables for a correction cooldown
const long correctionCooldownMillis = 2000;

float turnTargetYaw = 0.0;  // target yaw for turning
const float TURN_THRESHOLD = 5.0; // tolerance threshold for turns
int direction = 0;  // turn direction → -1: counterclockwise, +1: clockwise

// Miscellaneous
const int buttonPin = A0;
const int ledPin = 30;  // indicator LED
unsigned long lastUpdateTime = 0; // for main loop non-blocking delay
unsigned long lastCentreUpdateTime = 0; // for centring non-blocking while loop

void setup() {
  Serial.begin(115200); // initializing the serial monitor (115200 bauds is the Pixycam requirement)

  // Call all setup functions
  initMovement();
  initOrientation();
  initColorSensors();
  initUltrasonic();
  initStateLogic();

  waitForStartButton(); // wait for button press
  lastUpdateTime = millis();
  centreOnStart();
}

void loop() {
  unsigned long now = millis(); // updates current time

  // Code executed at 20 Hz
  if (now - lastUpdateTime < 50) return;  // if 50 ms have not passed, do not execute code
  
  float dt = (now-lastUpdateTime)/1000.0; // time differential in seconds (for MPU6050)
  lastUpdateTime = now; // updates loop's last update time
  // debugging();

  driveForward(motorSpeed); // drive forward with a specific speed

  updateOrientation(dt);  // PID control and MPU6050 orientation control
  keepOrientation();

  avoidWall(avoidance_amount);  // avoid walls with ultrasonic sensors
  correctionCooldown();
  
  if (detectFloorColor()) { // detect floor lines for turns
    handleColorAction();
    // Serial.println("Color detected."); // debugging
    }
  if (turningInProgress){ // check if turn is completed
    completedTurn();
  }

}

// Functions

// Function for a non-blocking delay
void safeDelay(unsigned long ms) {
  unsigned long start = millis(); 
  while (millis() - start < ms) {
    updateOrientation((millis() - lastUpdateTime)/1000.0);  // updates orientation
    // keepOrientation();
    if (detectFloorColor()) handleColorAction(); // checks for colors in the delay
  }
}

// SETUP FUNCTIONS

// Setup for Mobility Functionalities
void initMovement() {
  // DC Motor PWM pins as OUTPUT
  pinMode(MOTOR_INA_PWM, OUTPUT);
  pinMode(MOTOR_INB_PWM, OUTPUT);
  // Attach pin to the servo
  steeringServo.attach(SERVO_PIN);
  setSteeringAngle(SERVO_STRAIGHT); // set steering angle to straight in the setup
  pinMode(ENCODER_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING); // attach the interruption to the ISR, that counts the pulses
  Serial.println("Movement initialized...");
}

// Setup for Orientation Functionalities
void initOrientation() {
  Wire.begin(); // i2c connection to the MPU
  while (!mpu.begin()) {
   Serial.println("MPU6050 not found, trying again in 3 seconds...");
   delay(3000);
  }
  Serial.println("MPU6050 initialized succesfully.")
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // sets the accelerometer's measurement range to ±8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // sets the gyroscope's measurement range to ±500 degrees/second.
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // configures the DLPF bandwidth to 21 Hz, to reduce high-frequency noise.
  delay(250); // brief pause for stabilization

  // MPU6050 calibration
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }
  gyroZ_offset = sum / samples; // gets the average yaw offset in the static robot
  Serial.print("MPU Calibration completed.  Calculated offset: ");
  Serial.println(gyroZ_offset);
  Serial.println("Orientation control fully initialized...");
}

// Setup for Color Detection with TCS color sensors
void initColorSensors() {
  floorTcs.begin(); // start i2c tcs color sensor

  // defining pins s0, s1, s2, s3, and out as outputs.
  pinMode(S0_L, OUTPUT); pinMode(S1_L, OUTPUT); pinMode(S2_L, OUTPUT); pinMode(S3_L, OUTPUT); pinMode(OUT_L, INPUT); 
  pinMode(S0_R, OUTPUT); pinMode(S1_R, OUTPUT); pinMode(S2_R, OUTPUT); pinMode(S3_R, OUTPUT); pinMode(OUT_R, INPUT);
  digitalWrite(S0_L, HIGH); digitalWrite(S1_L, HIGH); // for frequency scaling set to 100%
  digitalWrite(S0_R, HIGH); digitalWrite(S1_R, HIGH); // set both s0 and s1 to HIGH
  Serial.println("Color sensors initialized...");
}

// Setup for Ultrasonic Sensors
void initUltrasonic() {
  // No setup needed; sonars declared in the header
  Serial.println("Ultrasonic sensors initialized...");
}

// Setup for State Logic Functionalities
void initStateLogic() {
  direction = 0;
  setTargetYaw(0.0);
  Serial.println("State logic initialized...");
}

// Start button function.
// Blocks the program until the button is pressed.
void waitForStartButton() {
  pinMode(buttonPin, INPUT);  // configures the button and led pin
  pinMode(ledPin, OUTPUT);
  Serial.println("Ready for initiation. Waiting for button press...");
  digitalWrite(ledPin, HIGH); // turns on the LED indicator
  
  while (!digitalRead(buttonPin)) { // while the button is not pressed
    delay(5);
  }

  digitalWrite(ledPin, LOW);  // turns off the LED indicator
}

// Movement Functions

// Drive forwards at a specific speed
void driveForward(int speed) {
  analogWrite(MOTOR_INA_PWM, speed);  // sets the Motor INA to the speed PWM signal
  digitalWrite(MOTOR_INB_PWM, LOW); // sets the Motor INB to LOW (0 PWM)
}

// Drive backwards at a specific speed
void driveBackward(int speed) {
  analogWrite(MOTOR_INA_PWM, LOW);
  digitalWrite(MOTOR_INB_PWM, speed);
}

// Decelerates motors naturally
void stopMotors() {
  digitalWrite(MOTOR_INA_PWM, LOW); // sets both motor input pin to LOW (0)
  digitalWrite(MOTOR_INB_PWM, LOW);
}

// Stops motors forcefully
void stopBrake() {
  digitalWrite(MOTOR_INA_PWM, HIGH); // sets both input pins to HIGH, to create a short-circuit braking effect
  digitalWrite(MOTOR_INB_PWM, HIGH);
}

// Function for controlling angular position of front steering wheels with attached servomotor
void setSteeringAngle(int angle) {
  angle = constrain(angle, SERVO_LEFT, SERVO_RIGHT);  // constrains the angle to limit the maximum and minimum servo angles
  steeringServo.write(angle); // writes the given angle to the servo motor
  // Serial.print("Servo angle set to: "); 
  // Serial.print(angle); 
  // Serial.println("°"); // prints the servo angle to the serial monitor
}

// Encoder Interrupt Service Routine to count pulses
void encoderISR() {
  encoderPulseCount++;
}

// Orientation Functions

// Function to get the accumulated yaw angle
void updateOrientation(float dt) {  // receives the time differential (in seconds)
  sensors_event_t a, g, temp; 
  mpu.getEvent(&a, &g, &temp);  // retrieves the sensor event data (gyroscope readings)
  float gyroZ_deg = (g.gyro.z - gyroZ_offset) * 57.2958;  // the result in radians/second is converted to degrees/second, and is compensated by subtracting the calculated offset
  if (abs(gyroZ_deg) > 0.1) { // a 0.1 noise threshold
    yaw += gyroZ_deg * dt;  // integrates the angular velocity over time, to obtain accumulated yaw
  }
}

// Function to apply the PID control for orientation correction
void keepOrientation() {
  float error = targetYaw - yaw;  // calculates the error in orientation
  float derivative = error - previousError; // error change over time
  int correction = SERVO_STRAIGHT - (int)(Kp * error + Kd * derivative);  // correction applying PID control
  setSteeringAngle(correction); // sets the servo angle to correct the error
  previousError = error;  // updates previous error
}

// Function to change the target orientation for PID correction
void setTargetYaw(float angle_deg) {
  targetYaw = angle_deg;
}

// Function that returns the current accumulated yaw
float getCurrentYaw() {
  return yaw;
}

// Function that returns the absolute value of the difference between the target yaw and yaw.
float getAbsoluteYawError() {
  return abs(abs(targetYaw)-abs(yaw));
}

// Color Functions

// Function for orange and blue floor lines detection
bool detectFloorColor() {
  uint16_t r,g,b,c;
  floorTcs.getRawData(&r,&g,&b,&c); // gets raw rgbc data from the sensor
  if (!c) return false; // avoid dividing by 0
  int rn=r*255/c, gn=g*255/c, bn=b*255/c; // normalizes values with the total light (c), for different light levels adaptability
  
  // Debugging for color sensor
  Serial.print("R: ");  Serial.println(rn);
  Serial.print("G: ");  Serial.println(gn);
  Serial.print("B: ");  Serial.println(bn);

  // For the first lap turn, defines direction:
  if (lapTurnCount == 0 && bn>BLUE_BLUE_THRESHOLD) {  // if blue color is detected
    direction = -1; // blue: left direction (-1)
    // Serial.print("Dirección establecida a: ");  // debugging
    // Serial.println(direction);
    return true;
  } else if (lapTurnCount == 0 && rn>ORANGE_RED_THRESHOLD) { // if orange color is detected
    direction = +1; // orange: right direction (+1)
    // Serial.print("Dirección establecida a: ");  // debugging
    // Serial.println(direction);
    return true;
  }

  if (direction<0 && bn>BLUE_BLUE_THRESHOLD) return true; // returns true if a color detection matches the threshold
  if (direction>0 && rn>ORANGE_RED_THRESHOLD) return true;
  return false;
}

// Function for turns when a color is detected
void handleColorAction() {
  if (detectFloorColor() && !turningInProgress) { // if the a color is detected at the bottom, and there's no turning in progress...
    lapTurnCount++; // lap turn counting
    digitalWrite(ledPin, HIGH); // turns on the LED for visualization of states
    
    turnTargetYaw = (turnTargetYaw + (90.0 * -direction));  // sets the target yaw to ± 90°, according to direction
    setTargetYaw(turnTargetYaw);
    
    turningInProgress = true; // flag for turning in progress, avoids color detection repetition
    motorSpeed = 250; // accelerates during turns
    
    // Serial.println("Turning... "); // serial print for debugging
  } else {
    // Serial.println("No color detected...");
    return;
  }
}

void completedTurn() {
  if (turningInProgress && getAbsoluteYawError() < TURN_THRESHOLD) {
    digitalWrite(ledPin, LOW); // turns off the LED for visualization of states
    turningInProgress = false;
    correctionState = false;

    motorSpeed = 145; // slows down for color detection
    Serial.println("Turn completed...");

    if (lapTurnCount == 12) { // if 3 laps are completed (4 lap turns = 1 lap)
    safeDelay(500); // waits to arrive to the defined stopping area
    stopBrake();  // stops the motors
    while (1);  // stops the program
    }
  }
}

void avoidWall(int correctionAmount) {
  // A correction is made only if there is no turn in progress and no correction was made in a cooldown time
  // A correction will not be made in the first lap turn, it is already placed in a straight orientation
  if (!turningInProgress && !correctionState && lapTurnCount != 0) {
    NewPing sonar = (direction < 0) ? sonarRight : sonarLeft;
  
    // Debugging for ultrasonic use
    /* if (direction < 0) {
      Serial.println("Usando el sensor ultrasónico derecho.");
    } else {
      Serial.println("Usando el sensor ultrasónico izquierdo.");
    }*/

    int distance = getDistance(sonar);

    // Debugging of ultrasonic distance data
    // Serial.print("Distance to wall: "); Serial.println(distance);
      
    if(distance < 20 && distance != 0) { // if it is too close to he wall
      turnTargetYaw = (turnTargetYaw + (correctionAmount*-direction)); // adjusts the target yaw to the left to correct deviations
      correctionState = true;
      setTargetYaw(turnTargetYaw);
      // Serial.println("Correcting trajectory, too close to the wall."); 
      lastCorrectionTime = millis(); // Registers the last correction time
    } 
      
    else if (distance > 40 && distance != 0) {
      turnTargetYaw = (turnTargetYaw + (correctionAmount*direction)); // adjusts the target yaw to the right to correct deviations
      correctionState = true;
      setTargetYaw(turnTargetYaw);
      lastCorrectionTime = millis(); // Registers the last correction time
      // Serial.println("Correcting trajectory, too far from the wall.");
    }

  } else {
    // Debugging for correction
    // if (turningInProgress) Serial.println("No correction: Turn in progress.");
    // if (correctionState) Serial.println("No correction: Correction already active.");
    // if (lapTurnCount == 0) Serial.println("No correction: First lap turn.");
    }
}

void correctionCooldown() {
  if (millis() - lastCorrectionTime > correctionCooldownMillis)) {
    correctionState = false;
    // Serial.println("Correction cooldown passed... Correction state = false."); // debugging
  } else {
    // Serial.println("Correction cooldown...");
  }
}

// Ultrasonic Sensors Functions

// Function to get distance, with a median filter for precision
int getDistance(NewPing& sonar) { // uses the original sonar object
  const int NUM_SAMPLES = 3;  // number of readings
  const int MAX_VALID_DISTANCE = 80;  // maximum distance in cm
  const int MIN_VALID_DISTANCE = 4; // minimum distance in cm
  const int STABILITY_THRESHOLD = 10; // disregards values outside the average range
  
  int samples[NUM_SAMPLES]; // defines an array to store readings
  int validCount = 0; // counts valid readings
  int sum = 0;  // variable to sum the three the valid readings

  for (int i = 0; i < NUM_SAMPLES; i++) {
    int d = sonar.ping_cm();  // takes three readings
    safeDelay(3); // brief delay to avoid interferences
    
    // Only accepts readings in the desired range
    if (d >= MIN_VALID_DISTANCE && d <= MAX_VALID_DISTANCE) {
      samples[validCount++] = d;  
      sum += d;
    }
  }

  if (validCount == 0) return 0;  // if no valid readings

  // Calculates the average readings
  int avg = sum / validCount;

  // Second filter: discards outliners
  sum = 0;  // restarts the sum to 0, to calculate a new average
  int filteredCount = 0;  // counts the readings that pass the second filter
  for (int i = 0; i < validCount; i++) {  // considers only the valid readings in the first filter
    if (abs(samples[i] - avg) <= STABILITY_THRESHOLD) { // calculates the absolute difference between the reading and the average
      sum += samples[i];  // if the difference is less than the STABILITY_THRESHOLD, it passes the second filter
      filteredCount++;
    }
  }

  if (filteredCount == 0) return 0; // if no readings passed the seconds filter, return 0

  return sum / filteredCount; // if there are many readings that passed the filter, returns the average
} 

void centreOnStart() {
  // measures distance to both sides
  int dl = sonarLeft.ping_cm();
  int dr = sonarRight.ping_cm();

  // calculates to difference, to center
  int diff = dl - dr;
  int centreAngle = (diff < 0) ? SERVO_RIGHT : SERVO_LEFT;  // if the measurement to the right is larger, adjust to the right; otherwise, adjust to the left

  // calculates the necessary pulses to centre
  int cm = ((abs(diff) / 2) / cos30deg);  // necessary cm to centre
  int pulses = cm * PULSES_PER_CM;  // necessary pulses to centre
  // Serial.print("Centre pulses: "); // debugging
  // Serial.println(pulses);

  encoderPulseCount = 0;
  lastCentreUpdateTime = millis(); // Inicializa el tiempo para este bucle

  setSteeringAngle(centreAngle); // sets the servo angle according to the distance to both sides
  delay(50);
  driveForward(200);  
  
  while (true) {
    unsigned long now = millis(); // Tiempo actual para el cálculo de dt

    if (now - lastCentreUpdateTime < 50) {
      continue; // update the orientation every 50 ms
    }
    
    float dt = (now - lastCentreUpdateTime) / 1000.0; // time differential in seconds
    lastCentreUpdateTime = now; // updates the time for next iteration

    // Updates the accumulated yaw
    updateOrientation(dt);

    // Checks the break condition
    if (encoderPulseCount >= pulses) {
      Serial.println("Centring completed...");
      break;
    }
  }

// State Logic Functions

void debugging() {
  Serial.print("Target yaw: ");   Serial.print(targetYaw);
  Serial.print("    Current Yaw: ");  Serial.println(yaw);
  Serial.print("Lap turns: ");  Serial.println(lapTurnCount);
  Serial.print("Motor speed: ");  Serial.println(motorSpeed);
  Serial.print("Turning in progress: "); Serial.print(turningInProgress);
  Serial.print("    Correction state: "); Serial.println(correctionState);
}
