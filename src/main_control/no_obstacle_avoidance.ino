#include <Servo.h>
#include <Wire.h>
#include <Pixy2.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>

// ================== DEFINICIÓN DE PINES Y CONSTANTES ==================

// MOTORES (L298N)
/*Necesita dos entradas analógicas (PWM) para su dirección y velocidad:*/
#define MOTOR_INA_PWM 3
#define MOTOR_INB_PWM 4

int motorSpeed = 200;  // Velocidad base del motor

// SERVOMOTOR (Servo 9g)
#define SERVO_PIN 9
Servo steeringServo;

const int SERVO_STRAIGHT = 90;         // Dirección recta
const int SERVO_GIRO = 55;             // Ángulo de rotación de Servo para giro en cualquier dirección
const int SERVO_LEFT = SERVO_STRAIGHT - SERVO_GIRO;     // 35° para giro a la izquierda
const int SERVO_RIGHT = SERVO_STRAIGHT + SERVO_GIRO;    // 145° para giro a la derecha

// LLANTAS
#define ENCODER_PIN 2 // Encoder de llanta
volatile unsigned long encoderPulseCount = 0;

// Parámetros del encoder y la llanta
#define WHEEL_CIRCUMFERENCE 22.0   // Perímetro de la llanta en cm (ejemplo)
#define PULSES_PER_REVOLUTION 32.0   // Pulsos detectados por revolución
#define PULSES_PER_CM (PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE)  // Pulsos por centímetro

// SENSORES DE COLOR
#define s0_left 20    // izquierda
#define s1_left 21
#define s2_left 22
#define s3_left 23
#define sensorOut_left 24

#define s0_right 25   // derecha
#define s1_right 26
#define s2_right 27
#define s3_right 28
#define sensorOut_right 29

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);  // abajo (I2C)

/*Las detecciones de color se normalizan a un valor de entre 0 y 255:
MAGENTA: se busca un valor alto de rojo y azul; y uno bajo en verde.
NARANJA: se busca un valor alto de rojo y verde; y uno bajo de azul.
AZUL: se busca un valor alto en azul; bajo en rojo y verde.*/

#define MAGENTA_RED_THRESHOLD 100 // mayor al valor
#define MAGENTA_BLUE_THRESHOLD 100  // mayor al valor
#define MAGENTA_GREEN_THRESHOLD 50  // menor al valor

#define ORANGE_RED_THRESHOLD   100
#define ORANGE_GREEN_THRESHOLD 100
#define ORANGE_BLUE_THRESHOLD  50

#define BLUE_BLUE_BOTTOM 100
#define BLUE_RED_THRESHOLD 50
#define BLUE_GREEN_THRESHOLD 50

// SENSORES ULTRASÓNICOS
#define US_CENTER_TRIGGER_PIN 10 // centro
#define US_CENTER_ECHO_PIN 11
#define US_LEFT_TRIGGER_PIN 12  // izquierda
#define US_LEFT_ECHO_PIN 13
#define US_RIGHT_TRIGGER_PIN 14 // derecha
#define US_RIGHT_ECHO_PIN 15

const int MAX_DISTANCE 200;

// Crear los ultrasónicos
NewPing sonarCenter(US_CENTER_TRIGGER_PIN, US_CENTER_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(US_LEFT_TRIGGER_PIN, US_LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(US_RIGHT_TRIGGER_PIN, US_RIGHT_ECHO_PIN, MAX_DISTANCE);

// PIXYCAM Y VISIÓN ARTIFICIAL 
Pixy2 pixy;

const int FRAME_WIDTH = 316;
const int FRAME_HEIGHT = 208;

// Definir la región de interés
const int ROI_TOP = 60;
const int ROI_BOTTOM = 208;

const int LEFT_BOUND = FRAME_WIDTH / 3;
const int RIGHT_BOUND = 2*FRAME_WIDTH / 3;

// MPU (MPU6050)
Adafruit_MPU6050 mpu;
float yaw = 0.0;
float gyroZ_offset = 0.0; // Sesgo del giroscopio
unsigned long currentTime = 0;
unsigned long lastTime = 0;
bool calibrated = false;
float straight_direction = 0.0; // Dirección dependiendo de la vuelta

// CONTROL PID para mantener trayectoria
const long Kp = 3;
const long Kd = 0.5;
volatile unsigned long error = 0; // distancia izquierda - distancia derecha
volatile unsigned long previous_error = 0;
volatile unsigned long derivative; // error - error anterior

// ESTADO DEL ROBOT
String direction; // variable para configurar si el robot gira a la derecha ("right") o a la izquierda ("left") en cada vuelta
unsigned int lapTurnCount;  // conteo de giros realizados (3 giros indican una vuelta completa)
unsigned int lapCompletedCount = 0;
bool parkingMode = false; // indicador para ejecutar el estacionamiento una vez (y detección de color)

bool SystemShutdown;  // indicador para detener el programa
bool ProgramStarted = false;

// Para encendido del robot
#define buttonPin = 16;
#define ledPin 17

// ============================================================================

void setup() {
  SystemShutdown = false; // encender el sistema

  // Inicialización del monitor serial para Pixycam (115200)
  Serial.begin(115200);
  // Inicializar el bus I2C
  Wire.begin();

  // Configurar pines del motor
  pinMode(MOTOR_INA_PWM, OUTPUT);
  pinMode(MOTOR_INB_PWM, OUTPUT);

  // Configurar los pines del sensor de color
  pinMode(s0_left, OUTPUT);
  pinMode(s1_left, OUTPUT);
  pinMode(s2_left, OUTPUT);
  pinMode(s3_left, OUTPUT);
  pinMode(sensorOut_left, INPUT);

  pinMode(s0_right, OUTPUT);
  pinMode(s1_right, OUTPUT);
  pinMode(s2_right, OUTPUT);
  pinMode(s3_right, OUTPUT);
  pinMode(sensorOut_right, INPUT);

  // Establecer la escalado de frecuencia al 100%
  digitalWrite(s0_left, HIGH);
  digitalWrite(s1_left, HIGH);
  digitalWrite(s0_right, HIGH);
  digitalWrite(s1_right, HIGH);

  // Inicializar el servomotor y colocarlo en posición recta
  steeringServo.attach(SERVO_PIN);
  setSteeringAngle(SERVO_STRAIGHT);

  // Configurar el pin del encoder y asociar la interrupción
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // Inicializar la PixyCam
  pixy.init();

  // Revisar la conexión con el MPU
  while (!Serial)
    delay(10);
  if (!mpu.begin()) {
    Serial.println("No se encontró el MPU6050");
    while (1) delay(10);
  }
  Serial.println("MPU6050 encontrado");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // Calibrar el sesgo del giroscopio (gyroZ)
  Serial.println("Calibrando giroscopio, no mover el sensor...");
  float sum = 0.0;
  int samples = 1000;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    Serial.println(i);
    delay(10);
  }
  gyroZ_offset = sum / samples;
  Serial.print("Offset Z: ");
  Serial.println(gyroZ_offset, 6);

  // Indicador LED al iniciar el robot
  pinMode(ledPin, OUTPUT); // Pin del LED como OUTPUT
  Serial.println("Programa listo para iniciar");
  digitalWrite(ledPin, HIGH); // Encender el LED al iniciar el programa
  delay(100);
}

void loop() {
  if (!programStarted) {
    if (digitalRead(buttonPin)) {
      digitalWrite(ledPin, LOW);
      lastUpdateTime = millis();
      programStarted = true;
    } else {
      Serial.println("Esperando para iniciar el programa... ");
      delay(50);
    }
    return;
  }

  unsigned long now = millis();

  if (now - lastUpdateTime >= 20) {
    float deltaTime = (now - lastUpdateTime) / 1000.0;  // Intervalo de tiempo en segundos
    lastUpdateTime = now;

    measureGyroscope(deltaTime);
    keepOrientation();
  }
}

// FUNCIONES DE MOVIMIENTO
void driveForward(int speed) {
  analogWrite(MOTOR_INA_PWM, speed);
  digitalWrite(MOTOR_INB_PWM, LOW);
}

void driveBackward(int speed) {
  analogWrite(MOTOR_INA_PWM, LOW);
  digitalWrite(MOTOR_INB_PWM, speed);
}

void stopMotors() {
  digitalWrite(MOTOR_INA_PIN, LOW);
  digitalWrite(MOTOR_INB_PIN, LOW);
}

void stopBrake() {
  digitalWrite(MOTOR_INA_PIN, HIGH);
  digitalWrite(MOTOR_INB_PIN, HIGH);
}

void setSteeringAngle(int angle) {
  steeringServo.write(angle);
  Serial.print("Ángulo de dirección establecido en: ");
  Serial.println(angle);
}

// FUNCIONES PARA GIROSCOPIO Y CONTROL PID
void measureGyroscope(float deltaTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Restar offset, convertir a grados/s
  float gyroZ_deg_per_sec = (g.gyro.z - gyroZ_offset) * 57.2958;  // Convertir radianes a grados

  // Umbral contra ruido de medida
  float umbral = 0.1;
  if (abs(gyroZ_deg_per_sec) > umbral) {
    yaw += gyroZ_deg_per_sec * deltaTime;
  }
  
  Serial.print("Ángulo Yaw relativo: ");
  Serial.print(yaw);
  Serial.println("°");
}

void keepOrientation() {
  unsigned long now = millis();

  // Ejecutar cada 20 ms
  if (now - lastUpdateTime >= 20) {
    // Control PID
    previous_error = error;
    error = straight_direction - yaw;
    derivative = error - previous_error;
    PD_adjustment = kp * error + kd * derivative;

    int servo_angle = servo_straight + PD_adjustment;
    servo_angle = constrain(servo_angle, servo_left, servo_right);

    setSteeringAngle(servo_angle);
  }
}

// FUNCIÓN PARA EL ENCODER
void encoderISR() {
  encoderPulseCount++;
}

// FUNCIONES DE LA PIXY CAM
void checkForObstacles() {
  pixy.ccc.getBlocks(); // Actualizar los datos de la PixyCam
  
  // Si se detectan bloques (objetos)
  if (pixy.ccc.numBlocks) {
    // Buscar el bloque con el área mayor
    int largestBlockIndex = 0;
    int largestArea = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int area = pixy.ccc.blocks[i].width * pixy.ccc.blocks[i].height;
      if (area > largestArea) {
        largestArea = area;
        largestBlockIndex = i;
      }
    }
    
    // Obtener la firma de color del bloque (1 = rojo, 2 = verde)
    int colorSignature = pixy.ccc.blocks[largestBlockIndex].m_signature;
    String obstacleColor = "";
    if (colorSignature == 1) {
      obstacleColor = "red";
    } else if (colorSignature == 2) {
      obstacleColor = "green";
    } else {
      Serial.println("Se ha detectado un color diferente.")
      return;
    }
    
    // Determinar la posición horizontal del objeto en la imagen (eje X)
    int BlockX = pixy.ccc.blocks[largestBlockIndex].m_x;
    int BlockY = pixy.ccc.blocks[largestBlockIndex].m_y;
    
    
    // Depuración: mostrar información del obstáculo
    Serial.print("Obstáculo detectado - Color: ");
    Serial.print(obstacleColor);
    Serial.print(" | Posición: ");
    Serial.println(blockPosition);
    
    // Ejecutar la maniobra de evasión según el color detectado
    avoidObstacle(obstacleColor);
  }
}

// Normaliza e invierte el valor crudo para obtener valores similares a RGB
int normalize(int value, int maxVal) {
  return map(constrain(value, 0, maxVal), 0, maxVal, 255, 0);
}

/* Para medir colores, se configuran los pines S2 y S3 de la siguiente forma:
ROJO: s2 LOW, s3 LOW
AZUL: s2 LOW, s3 HIGH
VERDE: s2 HIGH, s3 HIGH 
*/

int readRedLeft() {
  digitalWrite(s2_left, LOW);
  digitalWrite(s3_left, LOW);
  delay(10); // Retardo breve para estabilización
  return pulseIn(sensorOut_left, LOW);
}

int readGreenLeft() {
  digitalWrite(s2_left, LOW);
  digitalWrite(s3_left, HIGH);
  delay(10);
  return pulseIn(sensorOut_left, LOW);
}

int readBlueLeft() {
  digitalWrite(s2_left, LOW);
  digitalWrite(s3_left, HIGH);
  delay(10);
  return pulseIn(sensorOut_left, LOW);
}

int readRedRight() {
  digitalWrite(s2_right, LOW);
  digitalWrite(s3_right, LOW);
  delay(10);
  return pulseIn(sensorOut_right, LOW);
}

int readGreenRight() {
  digitalWrite(s2_left, HIGH);
  digitalWrite(s3_left, HIGH);
  delay(10);
  return pulseIn(sensorOut_left, LOW);
}

int readBlueRight() {
  digitalWrite(s2_right, LOW);
  digitalWrite(s3_right, HIGH);
  delay(10);
  return pulseIn(sensorOut_right, LOW);
}

// Normaliza e invierte el valor crudo para obtener valores similares a RGB (0 a 255)
int normalize(long value, long maxValue) {
  return map(constrain(value, 0, maxValue), 0, maxValue, 255, 0);
}

bool detectMagenta(direction) {
  if (direction == "left") {
    int redRaw = readRedRight();
    int greenRaw = readGreenRight();
    int blueRaw = readBlueRight();

  } else if (direction == "right") {
    int redRaw = readRedLeft();
    int greenRaw = readGreenLeft();
    int blueRaw = readBlueLeft();
  }

  // Determina el valor máximo para normalizar
  int maxRaw = max(max(redRaw, greenRaw), blueRaw);

  // Normalización e inversión de frecuencia para obtener valores RGB
  int red = normalize(redRaw, maxRaw);
  int green = normalize(greenRaw, maxRaw);
  int blue = normalize(blueRaw, maxRaw);

  Serial.print("RGB Normalizado: R=");
  Serial.print(red);
  Serial.print(" G=");
  Serial.print(green);
  Serial.print(" B=");
  Serial.print(blue);

  return (red > MAGENTA_RED_THRESHOLD && blue > MAGENTA_BLUE_THRESHOLD && green < MAGENTA_GREEN_THRESHOLD);
}

