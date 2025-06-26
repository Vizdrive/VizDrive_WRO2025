#include <SPI.h>
#include <Pixy2.h>
#include <Servo.h>

#define SERVO_PIN 6
#define SERVO_STRAIGHT 85
#define SERVO_LEFT 50
#define SERVO_RIGHT 110

Pixy2 pixy; // Objeto Pixy
Servo steeringServo; // Objeto Servo

void setup() {
  Serial.begin(115200); // Initializes Serial Bus at 115200 bauds for Pixycam
  Serial.print("Iniciando PixyCam...");

  pixy.init(); 
  Serial.println(" PixyCam inicializada.");

  steeringServo.attach(SERVO_PIN);
  setSteeringAngle(SERVO_STRAIGHT);
}

void loop() {
  int i = pixy.ccc.getBlocks();

  if (i > 0) {
    for (int j = 0; j < i; j++) {
      if (pixy.ccc.blocks[j].m_signature == 1) { 
        Serial.println("RED detected. Turning RIGHT.");
        setSteeringAngle(SERVO_RIGHT);
        delay(100);
        break; 
      } else if (pixy.ccc.blocks[j].m_signature == 2) {
        Serial.println("GREEN detected. Turning LEFT.");
        setSteeringAngle(SERVO_LEFT);
        delay(100);
        break; 
      }
    }
  } else {
    Serial.println("No blocks detected...");
    setSteeringAngle(SERVO_STRAIGHT);
  }

  delay(50);
}

void setSteeringAngle(int angle) {
  angle = constrain(angle, SERVO_LEFT, SERVO_RIGHT);
  steeringServo.write(angle);
}
