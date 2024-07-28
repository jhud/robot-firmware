#include <Arduino.h>
#include <OneWire.h>
#include <DS2438.h>

static int ONE_WIRE_PIN = 14;

//uint8_t DS2438_address[] = { 0x26, 0x62, 0x43, 0xB2, 0x00, 0x00, 0x00, 0xFD }; // veroboard bot

//uint8_t DS2438_addressPrimary[] = { 0x26, 0xe2, 0x79, 0x90, 0x01, 0x00, 0x00, 0xf2 };  // proto - battery, current and servo A
//uint8_t DS2438_addressSecondary[] = { 0x26, 0x8, 0x6a, 0x90, 0x01, 0x00, 0x00, 0x26 };  // proto - VCC and servo B

#ifndef FIRST_ROBOT
// robot 2
uint8_t DS2438_addressSecondary[] = { 0x26, 0x2D, 0x83, 0x90, 0x1, 0x0, 0x0, 0x52 };  // new chip, 3 and 5v
uint8_t DS2438_addressPrimary[] = { 0x26, 0xB5, 0x73, 0x90, 0x01, 0x00, 0x00, 0xE4  };  
#else
// robot 3 - ESP32-S3 board
//uint8_t DS2438_addressSecondary[] = { 0x26, 0xC9, 0xD6, 0xBA, 0x01, 0x00, 0x00, 0x8B  };  // new chip, 3 and 5v
//uint8_t DS2438_addressPrimary[] = { 0x26, 0xAA, 0xFD, 0x8C, 0x1, 0x0, 0x0, 0xD8 };  
#endif

OneWire ow(ONE_WIRE_PIN);
DS2438 ds2438Primary(&ow, DS2438_addressPrimary);
DS2438 ds2438Secondary(&ow, DS2438_addressSecondary);

bool ds2438_init() {
  log(F("DS2438s starting..."));

  if (ds2438Primary.begin(DS2438_MODE_CHA | DS2438_MODE_CHB | DS2438_MODE_TEMPERATURE | DS2438_MODE_CURRENT) == false) {
      log(F("Primary DS2438 failed."));
      return false;
  }

  log(F("Primary ADC ok."));

  if (ds2438Secondary.begin(DS2438_MODE_CHA | DS2438_MODE_CHB) == false) {
      log("Secondary DS2438 failed.");
      return false;
  }

  log(F("Secondary ADC ok."));

  // Should be saved in the DS2438's EEPROM, but can recalibrate here. Make sure shunt resistor is shorted.

  // Cycle through the states so that we already have all measurements ready to read.
  ds2438_blockingUpdate();

  return true;
}


/**
 * DS2438 library does a non-blocking round-robin reading, so to guarantee a read, we block and
 * wait for the results from every ADC read.
 */
void ds2438_blockingUpdate() {
  for (int i=0; i<4; i++) {
    ds2438Primary.update();
    ds2438Secondary.update();
    delay(40);
  }
}

void ds2438_update() {
    ds2438Primary.update(); // James modified this library for greater debugging
    if (ds2438Primary.isError()) {
        Serial.println(F("Error reading from DS2438 device"));
    } 

    ds2438Secondary.update(); // James modified this library for greater debugging
    if (ds2438Primary.isError()) {
        Serial.println(F("Error reading from DS2438 device"));
    }
}

double ds2438_temp() {
  return ds2438Primary.getTemperature();
}


double ds2438_currentAmps() {
  return -ds2438Primary.getCurrent(0.01f); // Chip has vSense flipped, so reverse it. This is fixed in the next hardware revision 4.
}

int ds2438_accumulatedCharge() {
  return ds2438Primary.getAccumulatedChargeData(); 
}


double ds2438_mainBatteryVoltage() {
  return ds2438Primary.getVoltage(DS2438_CHB);
}

double ds2438_VccVoltage() {
  return ds2438Secondary.getVoltage(DS2438_CHB);
}

double ds2438_ServoAVoltage() {
   return ds2438Primary.getVoltage(DS2438_CHA);
}

double ds2438_ServoBVoltage() {
    return ds2438Secondary.getVoltage(DS2438_CHA);
}
