#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Lista de diferentes cantidades de samples para calibrar
int sampleCounts[] = {10, 25, 50, 100, 150, 200, 300, 500, 750, 1000};
const int numConfigs = sizeof(sampleCounts) / sizeof(sampleCounts[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("No se encontró el MPU6050");
    while (1) delay(10);
  }
  Serial.println("MPU6050 encontrado");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);

  for (int config = 0; config < numConfigs; config++) {
    int muestras = sampleCounts[config];
    Serial.print("\nCalibrando con ");
    Serial.print(muestras);
    Serial.println(" muestras...");

    float suma = 0.0;
    for (int i = 0; i < muestras; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      suma += g.gyro.z;
      delay(10); // 100Hz
    }
    float offsetZ = suma / muestras;
    Serial.print("Offset Z obtenido: ");
    Serial.println(offsetZ, 6);

    // Medir error acumulado de yaw durante 10 segundos
    float yaw = 0.0;
    unsigned long tInicio = millis();
    unsigned long lastTime = tInicio;

    while (millis() - tInicio < 10000) { // 10 segundos
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      unsigned long currentTime = millis();
      float deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;

      float gyroZ_deg_per_sec = (g.gyro.z - offsetZ) * 57.2958;
      yaw += gyroZ_deg_per_sec * deltaTime;

      delay(10); // 100 Hz
    }

    Serial.print("Yaw acumulado tras 10 segundos con ");
    Serial.print(muestras);
    Serial.print(" muestras: ");
    Serial.print(yaw, 6);
    Serial.println(" °");
    delay(3000); // Espera antes de la siguiente prueba
  }

  Serial.println("\nPruebas finalizadas.");
}

void loop() {
}