#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* * Código para el sensor de color TCS3472 con detección rápida.
 * * - Muestra los valores crudos de Rojo, Verde, Azul y Claridad (Clear).
 * - Normaliza los valores RGB a una escala de 0-255.
 * - Configurado para lecturas rápidas ajustando el tiempo de integración y la ganancia.
 * * Conexiones (Arduino Uno):
 * - VIN -> 5V
 * - GND -> GND
 * - SDA -> A4
 * - SCL -> A5
 */

// Inicializa el sensor con los parámetros para detección rápida
// Tiempo de integración: 2.4ms (el más rápido)
// Ganancia: 1x (la más baja)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(115200);
  
  if (tcs.begin()) {
    Serial.println("Sensor TCS34725 encontrado!");
  } else {
    Serial.println("No se encontró el sensor TCS34725... revisa las conexiones.");
    while (1); // Detiene el programa si no hay sensor
  }
}

void loop() {
  uint16_t r_raw, g_raw, b_raw, c_raw;

  // Lee los datos crudos del sensor
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

  // --- Normalización de 0 a 255 ---
  // Se usa el valor del canal "Clear" (c_raw) para normalizar, 
  // ya que representa la cantidad total de luz detectada.
  // Esto ayuda a compensar las variaciones en la intensidad de la luz.
  
  // Evitar división por cero si no hay luz
  if (c_raw == 0) {
    return;
  }

  float r_norm = (float)r_raw / c_raw * 255.0;
  float g_norm = (float)g_raw / c_raw * 255.0;
  float b_norm = (float)b_raw / c_raw * 255.0;

  // Imprime los valores en el Monitor Serie
  Serial.print("RAW -> ");
  Serial.print("R: "); Serial.print(r_raw);
  Serial.print(" G: "); Serial.print(g_raw);
  Serial.print(" B: "); Serial.print(b_raw);
  Serial.print(" C: "); Serial.print(c_raw);
  Serial.print("  |  ");
  
  Serial.print("Normalizado (0-255) -> ");
  Serial.print("R: "); Serial.print((int)r_norm);
  Serial.print(" G: "); Serial.print((int)g_norm);
  Serial.print(" B: "); Serial.println((int)b_norm);
  
  delay(10); // Pequeña pausa para no saturar el Monitor Serie
}
