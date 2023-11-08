#ifndef MPU6050_H
#define MPU6050_H

#include <Adafruit_Sensor.h>

typedef struct
{
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t temp;
} IMUUpdate;

bool mpu6050_init();
void mpu6050_update(IMUUpdate & upd);

#endif // MPU6050_H
