// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Wire.h>

#include "mpu6050.h"

Adafruit_MPU6050 mpu;

bool mpu6050_init() {
  log(F("MPU6050 starting..."));

  // Try to initialize!
  if (!mpu.begin(0x68, &Wire, 0)) {
    log(F("Failed to find MPU6050"));
    return false;
  }
  log(F("MPU6050 ok."));
  return true;
}


void mpu6050_update(IMUUpdate & upd)
{
  mpu.getEvent(&upd.a, &upd.g, &upd.temp);
}
