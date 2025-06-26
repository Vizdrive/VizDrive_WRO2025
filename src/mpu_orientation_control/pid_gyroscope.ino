#include <Servo.h>
#include <NewPing.h>

// Pin para Servo
#define SERVO_PIN 27
Servo steeringServo;

// Constantes para el giro del servomotor
const int SERVO_STRAIGHT = 90; 
const int SERVO_GIRO = 55;
const int SERVO_LEFT = SERVO_STRAIGHT - SERVO_GIRO;   // 35° para giro a la izquierda
const int SERVO_RIGHT = SERVO_STRAIGHT + SERVO_GIRO;  // 145° para giro a la derecha

// Pines de los sensores ultrasónicos
#define US_LEFT_TRIGGER_PIN 3
#define US_LEFT_ECHO_PIN 4
#define US_RIGHT_TRIGGER_PIN 6
#define US_RIGHT_ECHO_PIN 5

const int MAX_DISTANCE = 100;

NewPing sonarLeft(US_LEFT_TRIGGER_PIN, US_LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(US_RIGHT_TRIGGER_PIN, US_RIGHT_ECHO_PIN, MAX_DISTANCE);

int left_distance = 0;
int right_distance = 0;

// CONTROL PD
// Control PD para el ajuste de la dirección
const long Kp = 1.3;
const long Kd = 1;
volatile unsigned long error = 0; // distancia izquierda - distancia derecha
volatile unsigned long previous_error = 0;
volatile unsigned long derivative; // error - error anterior

volatile unsigned long PD_adjustment;

void setup() {
  Serial.begin(115200);

  steeringServo.attach(SERVO_PIN);
  setSteeringAngle(SERVO_STRAIGHT);

  Serial.println("Iniciando programa...");
  delay(3000);
}

void loop() {
  keepCenter();
  delay(1000);
}

void keepCenter() {
  // Control PD
  previous_error = error;
  left_distance = sonarLeft.ping_cm();
  right_distance = sonarRight.ping_cm();

  Serial.print("Derecha: ");
  Serial.println(right_distance);
  Serial.print("Izquierda: ");
  Serial.println(left_distance);

  error = (unsigned long)(left_distance - right_distance);
  derivative = (unsigned long)(error - previous_error);
  PD_adjustment = Kp * error + Kd * derivative;

  int servo_angle = SERVO_STRAIGHT + PD_adjustment;
  servo_angle = constrain(servo_angle, SERVO_LEFT, SERVO_RIGHT);

  setSteeringAngle(servo_angle);
}

void setSteeringAngle(int angle) {
  steeringServo.write(angle);
  // Depuración: imprimir el ángulo de dirección
  Serial.print("Ángulo de dirección establecido en: ");
  Serial.println(angle);
}