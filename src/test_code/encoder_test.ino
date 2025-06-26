int encoderPin = 2;
volatile unsigned long encoder_count = 0;
unsigned long anteriorTiempo = 0;
const unsigned long intervalo = 50;

void setup() {
  Serial.begin(115200);
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
}

void loop() {
  unsigned long actualTiempo = millis();
  if (actualTiempo - anteriorTiempo >= intervalo) {
    Serial.println(encoder_count);
    anteriorTiempo = actualTiempo;
  }
}

void encoderISR() {
  encoder_count++;
}