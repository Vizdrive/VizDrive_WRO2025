/*#define s0 50        //Module pins   wiring
#define s1 52 //
#define s2 51
#define s3 49
#define out 53
#define led 48 //
*/

#define s0 10        //Module pins   wiring
#define s1 12 //
#define s2 16
#define s3 18
#define out 14
#define led 8 //

/*
#define s0 10        //Module pins   wiring
#define s1 8
#define s2 16
#define s3 18
#define out 14
#define led 12

LO Q SIRVE
#define s0 10        //Module pins   wiring
#define s1 12 //
#define s2 16
#define s3 18
#define out 14
#define led 8 //

OUT 48*

  S0 51*

  S1 49*

  S2 50*

  S3 52*

  LED 53*

++++++++++++++++++++

  OUT 18*

  S0 10*

  S1 8*

  s2 16*

  s3 14*

  LED 12*
*/

// Ajustar parámetros
const int colorBlue = 100;
const int colorRed = 135;
const int colorGreen = 50;

void setup() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);

  digitalWrite(s0, HIGH); // Frecuencia al 100%
  digitalWrite(s1, HIGH);

  Serial.begin(9600);
}

// Lee la duración del pulso para un filtro de color específico
int readColorFrequency(int colorS2, int colorS3) {
  digitalWrite(s2, colorS2);
  digitalWrite(s3, colorS3);
  delay(10); // Espera para estabilizar la lectura
  return pulseIn(out, LOW);
}

// Normaliza e invierte el valor crudo para obtener valores similares a RGB
int normalize(int value, int maxVal) {
  return map(constrain(value, 0, maxVal), 0, maxVal, 255, 0);
}

// Identifica un color básico a partir de los valores RGB
String getColorName(int r, int g, int b) {
  if (r > colorRed && g < colorGreen && b < colorBlue) return "Rojo";
  if (r < colorRed && g > colorGreen && b < colorBlue) return "Verde";
  if (r < colorRed && g < colorGreen && b > colorBlue) return "Azul";
  if (r > colorRed && g > colorGreen && b < colorBlue) return "Amarillo";
  if (r < colorRed && g > colorGreen && b > colorBlue) return "Cian";
  if (r > colorRed && g < colorGreen && b > colorBlue) return "Magenta";
  if (r > colorRed && g > colorGreen && b > colorBlue) return "Blanco";
  if (r < 50 && g < 50 && b < 50) return "Negro";
  return "Color Desconocido";
}

void loop() {
  // Lecturas crudas
  int redRaw   = readColorFrequency(LOW, LOW);    // Rojo
  int greenRaw = readColorFrequency(HIGH, HIGH);  // Verde
  int blueRaw  = readColorFrequency(LOW, HIGH);   // Azul

  // Determinar el valor máximo para normalizar
  int maxRaw = max(max(redRaw, greenRaw), blueRaw);

  // Normalización (inversión de frecuencia)
  int red   = normalize(redRaw, maxRaw);
  int green = normalize(greenRaw, maxRaw);
  int blue  = normalize(blueRaw, maxRaw);

  // Mostrar datos
  Serial.print("RGB Normalizado: R=");
  Serial.print(red);
  Serial.print(" G=");
  Serial.print(green);
  Serial.print(" B=");
  Serial.print(blue);
  Serial.print(" | Color: ");
  Serial.println(getColorName(red, green, blue));

}
