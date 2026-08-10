#include <Wire.h>
#include <math.h>
#include "Kalman1D.h"

const uint8_t MPU6050_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// Variables globales modificables por la función de perfiles
float ACCEL_SCALE = 16384.0;
float GYRO_SCALE = 131.0;

// Instanciamos dos filtros separados, uno para cada eje
Kalman1D kalmanRoll;
Kalman1D kalmanPitch;

unsigned long lastMicros = 0;
hw_timer_t *sampleTimer = NULL;
volatile bool timerFlag = false;


void IRAM_ATTR timerISR() {
  timerFlag = true;
}

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
  Wire.begin(21, 22);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  setSystemProfile(1);

  sampleTimer = timerBegin(1000000);
  timerAttachInterrupt(sampleTimer, &timerISR);
  timerAlarm(sampleTimer, 10000, true, 0);

  lastMicros = micros();
}

void loop() {

  if (timerFlag) {
    timerFlag = false;

    unsigned long currentMicros = micros();
    float dt = (float)(currentMicros - lastMicros) / 1000000.0;
    lastMicros = currentMicros;

    if (dt <= 0) dt = 0.01;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    if (Wire.available() >= 14) {
      int16_t rawAccelX = (Wire.read() << 8) | Wire.read();
      int16_t rawAccelY = (Wire.read() << 8) | Wire.read();
      int16_t rawAccelZ = (Wire.read() << 8) | Wire.read();
      int16_t rawTemp = (Wire.read() << 8) | Wire.read();
      int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
      int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
      int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

      float accelX = (float)rawAccelX / ACCEL_SCALE;
      float accelY = (float)rawAccelY / ACCEL_SCALE;
      float accelZ = (float)rawAccelZ / ACCEL_SCALE;

      float tempC = ((float)rawTemp / 340.0) + 36.53;

      float gyroX = (float)rawGyroX / GYRO_SCALE;
      float gyroY = (float)rawGyroY / GYRO_SCALE;

      float rollAcc = atan2(accelY, accelZ) * RAD_TO_DEG;
      float pitchAcc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

      float finalRoll = kalmanRoll.getAngle(rollAcc, gyroX, dt);
      float finalPitch = kalmanPitch.getAngle(pitchAcc, gyroY, dt);

      Serial.print(finalPitch);
      Serial.print(",");
      Serial.print(finalRoll);
      Serial.print(",");
      Serial.print(accelX * 9.81);
      Serial.print(",");
      Serial.print(accelY * 9.81);
      Serial.print(",");
      Serial.print(accelZ * 9.81);
      Serial.print(",");
      Serial.println(tempC);
    }
  }
}