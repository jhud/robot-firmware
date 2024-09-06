#include "config.h"

// By default LIDAR device is at 0x29

#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

bool vl53l0x_init() {
  // VL53L0X_SENSE_HIGH_SPEED VL53L0X_SENSE_LONG_RANGE
  if (!lox.begin(VL53L0X_I2C_ADDR, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    log(F("Failed to boot VL53L0X"));
    SET_ERROR();
    return false;
  }

  lox.startRangeContinuous();

  return true;
}

double vl53l0x_rangeMM() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.getRangingMeasurement(&measure, false);
  if (measure.RangeStatus != 4) { // phase failures have incorrect data
      return measure.RangeMilliMeter;
    } else {
      return 9999.0;
    }
}
