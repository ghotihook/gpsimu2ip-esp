#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Custom I2C pins for M5CoreS3
#define SDA_PIN 12
#define SCL_PIN 11
TwoWire myWire(1);

// BNO085 I2C address
#define BNO085_ADDR 0x4A

// Optional: report type (more stable vs faster)
#define USE_STABILIZED_RV

Adafruit_BNO08x bno08x(-1);  // No reset pin
sh2_SensorValue_t sensorValue;

// Euler structure
struct EulerAngles {
  float yaw;
  float pitch;
  float roll;
} ypr;

void quaternionToEuler(float qr, float qi, float qj, float qk, EulerAngles* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw   = atan2(2.0 * (qi * qj + qk * qr), sqi - sqj - sqk + sqr);
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll  = atan2(2.0 * (qj * qk + qi * qr), -sqi - sqj + sqk + sqr);

  if (degrees) {
    ypr->yaw   *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll  *= RAD_TO_DEG;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Starting BNO085 Yaw/Pitch/Roll example...");

  myWire.begin(SDA_PIN, SCL_PIN, 400000);
  if (!bno08x.begin_I2C(BNO085_ADDR, &myWire)) {
    Serial.println("Failed to find BNO085 on I2C bus");
    while (1) delay(10);
  }

  Serial.println("BNO085 Found!");

  // Select report type and frequency
#ifdef USE_STABILIZED_RV
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {  // 200 Hz
    Serial.println("Could not enable ARVR_STABILIZED_RV");
  }
#else
  if (!bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, 2000)) {  // 500 Hz
    Serial.println("Could not enable GYRO_INTEGRATED_RV");
  }
#endif
}

void loop() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV || sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
      float qr, qi, qj, qk;
      if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        qr = sensorValue.un.arvrStabilizedRV.real;
        qi = sensorValue.un.arvrStabilizedRV.i;
        qj = sensorValue.un.arvrStabilizedRV.j;
        qk = sensorValue.un.arvrStabilizedRV.k;
      } else {
        qr = sensorValue.un.gyroIntegratedRV.real;
        qi = sensorValue.un.gyroIntegratedRV.i;
        qj = sensorValue.un.gyroIntegratedRV.j;
        qk = sensorValue.un.gyroIntegratedRV.k;
      }

      quaternionToEuler(qr, qi, qj, qk, &ypr, true);  // Output in degrees

      Serial.print("Yaw: ");
      Serial.print(ypr.yaw, 1);
      Serial.print("  Pitch: ");
      Serial.print(ypr.pitch, 1);
      Serial.print("  Roll: ");
      Serial.println(ypr.roll, 1);
    }
  }
}