// Basic demo for accelerometer readings from Adafruit MPU6050

#include "lidar_okdo.h"

#define BAUD_RATE 230400

//#define RXD2 16
//#define TXD2 17



static int state = 0;
static int ptr = 0;
static uint8_t data[PKG_VER_LEN]; 


bool lidar_init() {
  Serial2.begin(BAUD_RATE);
  return Serial2.available() ;
}


/**
 * Nonblocking read of lidar.
 * @return packet ptr if we have a full packet, otherwise null.
 */
LiDARFrameTypeDef * lidar_update()
{
  if (Serial2.available())
  {    
    auto val = Serial2.read();
    if (state == 0 && val == 0x54) {// start header
      state = 1;
            data[ptr++] = val;
    }
    else if (state == 1) { // length - never changes
      if (val == PKG_VER_LEN) {
        state = 2;
                    data[ptr++] = val;
        ptr = 0;
      }
      else {     
        Serial.println(val, HEX);
        state = 0;
      }
    }
    else if (state == 2) {
            data[ptr++] = val;
      if (ptr == PKG_VER_LEN) {
        state = 0;
        return (LiDARFrameTypeDef *)data;
    }
    }
   
  }
  return NULL;
}
