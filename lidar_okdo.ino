// Sampling frquency is 4500Hz
// Spin speed ~10Hz
// UART TX only: 230400 8N1
// Begins sending data as soon as it becomes stable.


#include "lidar_okdo.h"

#define BAUD_RATE 230400

#define LIDAR_RX_PIN 5 // switch0 on the board
#define LIDAR_TX_PIN 18 // switch1 on the board

static int state = 0;
static int ptr = 0;
static uint8_t data[sizeof(LiDARFrameTypeDef)] = "l\n"; 


bool lidar_init() {
  Serial2.begin(BAUD_RATE, SERIAL_8N1, LIDAR_RX_PIN);
  delay(100);
  return Serial2.available() ;
}


/**
 * Nonblocking read of lidar.
 * @return packet ptr if we have a full packet, otherwise null.
 */
LiDARFrameTypeDef * lidar_update()
{
  const int robotHeaderSize = sizeof(LiDARFrameTypeDef().robot_header);
  
  while (Serial2.available())
  {    
    auto val = Serial2.read();
    if (state == 0 && val == 0x54) {// start header
      state = 1;
              ptr = robotHeaderSize;
            data[ptr++] = val;
    }
    else if (state == 1) { // length - never changes
      if (val == PKG_VER_LEN) {
        state = 2;
        data[ptr++] = val;
      }
      else {     
        state = 0;
      }
    }
    else if (state == 2) {
      data[ptr++] = val;
      if (ptr == sizeof(data)) {
        state = 0;
        ptr = robotHeaderSize;
        return (LiDARFrameTypeDef *)data;
    }
    }
  }
  return NULL;
}
