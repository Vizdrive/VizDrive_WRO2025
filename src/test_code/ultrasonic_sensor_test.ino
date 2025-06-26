#include <NewPing.h>

// Pines de los sensores ultrasónicos
#define US_LEFT_TRIGGER_PIN 47
#define US_LEFT_ECHO_PIN 28
#define US_RIGHT_TRIGGER_PIN A9
#define US_RIGHT_ECHO_PIN A11

#define US_FRONT_RIGHT_ECHO_PIN 35
#define US_FRONT_RIGHT_TRIGGER_PIN 37
#define US_FRONT_LEFT_ECHO_PIN 33
#define US_FRONT_LEFT_TRIGGER_PIN A15

const int MAX_DISTANCE = 200;

NewPing sonarLeft(US_LEFT_TRIGGER_PIN, US_LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarRight(US_RIGHT_TRIGGER_PIN, US_RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarFrontLeft(US_FRONT_LEFT_TRIGGER_PIN, US_FRONT_LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarFrontRight(US_FRONT_RIGHT_TRIGGER_PIN, US_FRONT_RIGHT_ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Izquierda delantero: ");
  Serial.print(sonarFrontLeft.ping_cm());

  Serial.print("                     Derecha delantero: ");
  Serial.println(sonarFrontRight.ping_cm());

  Serial.print("Izquierda: ");
  Serial.print(sonarLeft.ping_cm());

  Serial.print("                                   Derecha: ");
  Serial.println(sonarRight.ping_cm());

  Serial.println("--------------------------------------------------------------------");

  delay(1000);
}
