#ifndef CONFIG_H_
#define CONFIG_H_

// The first complete robot, using ESP32-S3. Otherwise an older ESP32 board with different pin mappings.
#define FIRST_ROBOT

#define USE_OTA
//#define USE_IR
#define HIGH_QUALITY_SPEECH

#ifdef FIRST_ROBOT
  #define USE_VL53l0X
#else
  #define USE_LIDAR
#endif
#define USE_DS2438
//#define USE_IMU
#define I2C_SCAN

#endif // CONFIG_H_
