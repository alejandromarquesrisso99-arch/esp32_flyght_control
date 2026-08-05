#include <Wire.h>
#include <math.h>
#include "Kalman1D.h"

const uint8_t MPU6050_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
//const float RAD_TO_DEG = 180.0 / PI;

// Variables globales modificables por la función de perfiles
float ACCEL_SCALE = 16384.0;
float GYRO_SCALE = 131.0;

// Instanciamos dos filtros separados, uno para cada eje
Kalman1D kalmanRoll;
Kalman1D kalmanPitch;

unsigned long timer;

// --- FUNCIÓN MAESTRA DE PERFILES (Hardware + Software) ---
bool setSystemProfile(uint8_t perfil) {
  uint8_t accelConfig = 0x00;
  uint8_t gyroConfig = 0x00;

  switch (perfil) {
    case 0:                // [Perfil 0] Dron de Observación / Trípode (Movimiento Suave)
      accelConfig = 0x00;  // ±2g
      gyroConfig = 0x00;   // ±250 °/s
      ACCEL_SCALE = 16384.0;
      GYRO_SCALE = 131.0;
      kalmanRoll.setRmeasure(0.005f);
      kalmanPitch.setRmeasure(0.005f);  // Confiamos mucho en Accel
      kalmanRoll.setQangle(0.001f);
      kalmanPitch.setQangle(0.001f);
      Serial.println("\n[SISTEMA] Perfil 0: Ultra Preciso (Anclado a gravedad).");
      break;

    case 1:                // [Perfil 1] Vehículo Terrestre (Movimiento Moderado)
      accelConfig = 0x08;  // ±4g
      gyroConfig = 0x08;   // ±500 °/s
      ACCEL_SCALE = 8192.0;
      GYRO_SCALE = 65.5;
      kalmanRoll.setRmeasure(0.03f);
      kalmanPitch.setRmeasure(0.03f);
      kalmanRoll.setQangle(0.001f);
      kalmanPitch.setQangle(0.001f);
      Serial.println("\n[SISTEMA] Perfil 1: Moderado (Balance estándar).");
      break;

    case 2:                // [Perfil 2] Dron Estándar (Dinámico)
      accelConfig = 0x10;  // ±8g
      gyroConfig = 0x10;   // ±1000 °/s
      ACCEL_SCALE = 4096.0;
      GYRO_SCALE = 32.8;
      kalmanRoll.setRmeasure(0.06f);
      kalmanPitch.setRmeasure(0.06f);
      kalmanRoll.setQangle(0.0005f);
      kalmanPitch.setQangle(0.0005f);
      Serial.println("\n[SISTEMA] Perfil 2: Dinámico (Alta movilidad).");
      break;

    case 3:                // [Perfil 3] Misil / Caza (Acrobático y Violento)
      accelConfig = 0x18;  // ±16g
      gyroConfig = 0x18;   // ±2000 °/s
      ACCEL_SCALE = 2048.0;
      GYRO_SCALE = 16.4;
      kalmanRoll.setRmeasure(0.1f);
      kalmanPitch.setRmeasure(0.1f);  // Confiamos poco en Accel por ruido G
      kalmanRoll.setQangle(0.0001f);
      kalmanPitch.setQangle(0.0001f);  // Confiamos mucho en el Gyro
      Serial.println("\n[SISTEMA] Perfil 3: Acrobático (Filtro priorizando inercial).");
      break;

    default: return false;
  }

  // Transmitimos la configuración al hardware I2C
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_GYRO_CONFIG);
  Wire.write(gyroConfig);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_CONFIG);
  Wire.write(accelConfig);
  Wire.endTransmission();

  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { yield(); }

  Wire.begin(21, 22);
  Wire.setClock(400000);  // Forzamos I2C a alta velocidad (400kHz) para control rápido

  // Despertar el MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  // Iniciamos la nave en Perfil 0 (Cámbialo aquí para experimentar)
  setSystemProfile(3);

  timer = micros();
}

void loop() {
  // 1. Lectura en bloque
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  if (Wire.requestFrom(MPU6050_ADDR, (uint8_t)14) != 14) return;

  int16_t rawAccelX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelZ = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();  // Omitir Temp
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

  // 2. Aplicar escala física
  float accX = (float)rawAccelX / ACCEL_SCALE;
  float accY = (float)rawAccelY / ACCEL_SCALE;
  float accZ = (float)rawAccelZ / ACCEL_SCALE;
  float gyroX = (float)rawGyroX / GYRO_SCALE;
  float gyroY = (float)rawGyroY / GYRO_SCALE;

  // 3. Ecuación del Acelerómetro (Gravedad vectorizada)
  float accelRoll = atan2(accY, accZ) * RAD_TO_DEG;
  float accelPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  // 4. Calcular el tiempo exacto transcurrido (dt)
  float dt = (float)(micros() - timer) / 1000000.0;
  timer = micros();

  // 5. Motor de Fusión de Sensores (Kalman Filter)
  float finalRoll = kalmanRoll.getAngle(accelRoll, gyroX, dt);
  float finalPitch = kalmanPitch.getAngle(accelPitch, gyroY, dt);

  // 6. Preparar salida para el "Serial Plotter"
  //Serial.print("RawPitch:");
  //Serial.print(accelPitch);
  //Serial.print("\tKalmanPitch:");
  Serial.print(finalPitch);
  // Serial.print("\tRawRoll:");
  Serial.print(",");
  //Serial.print(accelRoll);
  //Serial.print("\tKalmanRoll:");
  Serial.println(finalRoll);

  delay(10);  // Loop estabilizado a ~100Hz
}