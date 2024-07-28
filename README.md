# Robot Firmware

This robot is designed as a cheap and simple testbed for the GPT age; a physical body for your machine learning projects.

This is the Arduino firmware for my small robot motherboard. [The corresponding KiCAD files](https://github.com/jhud/robot_board).

![robot image](https://github.com/jhud/robot_board/raw/main/images/robot.jpg)

![robot board](https://github.com/jhud/robot_board/raw/main/images/robot_2.jpg)

## Features

 - PWM control for unlimited servos (32 by default)
 - spline interpolation between movement keyframes
 - power management and telemetry for multiple power rails
 - WiFi connection to a controller computer via TCP/IP
 - retro 80s speech synthesis (need extra PSRAM on your ESP32)
 - IMU, LIDAR, and extra PWM servo hardware board support
 - Infrared beacon detection
 - 5V rail for an optional ESP32-CAM camera board
 - Over-the-air updates via WiFi
 - Physical switches can be configured to interrupt movement locally, to avoid hitting obstacles.

# Setup 

The Espressif ESP32 3.* library seems to be incompatible with Flite, so we are stuck at 2.0

Install all the needed Arduino libraries, and flash the motherboard via USB or UART.

Recommended Arduino board settings for the stock hardware rev 3 are as follows:
![esp32_s3_USB_settings](/media/esp32_s3_USB_settings.png)


On first use, the robot will create a wifi hotspot called "Robot init", where you can setup your wifi connection. You can always tap the option key just after startup to reset the wifi. 

## ESP32 partitions
This is the recommended Arduino partition scheme for the robot, if you are using a 16MB ESP32 S3. It allows OTA, with more than enough room for the speech synthesis library.

Boards.txt entry

```
esp32.menu.PartitionScheme.large_app_small_spiffs_ota_16M=16M OTA (7MB APP/2MB SPIFFS)
esp32.menu.PartitionScheme.large_app_small_spiffs_ota_16M.build.partitions=large_app_small_spiffs_ota_16M
esp32.menu.PartitionScheme.large_app_small_spiffs_ota_16M.upload.maximum_size=7307264
```

```
#Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x700000,
app1,     app,  ota_1,   0x710000,0x700000,
spiffs,   data, spiffs,  0xE10000,0x1E0000,
coredump, data, coredump,0xFF0000,0x10000,
```
Or you can find all partition files in /arduino/esp32_toolchain_patches.

## PSRAM

This is good, because it allows much longer voice synthesis. The latest revision of the robot hardware has now been updated for ESP32-S3 which usually comes with PSRAM, but if it doesn't you have other options.

You can decapsule an ESP32 module and "dead bug" an ESP-PSRAM32H into it to get extra PSRAM for the voice synthesis. There is also an ESP32 "in the wild" with 2MB PSRAM, but it is hard to find: ESP32-WROOM-32E-N8R2. But it is a drop-in replacement for the usual ESP32-WROOM-32E-N16, and the robot program only needs about 4MB of flash memory, so 8MB is fine.


## Setting speaker gain

There is a header pin on the hardware for setting the speaker gain, or you can configure a solder bridge.

```
Gain Rate	Gain Pin Connection
15 dB	Connected to GND through a 100kΩ resistor
12 dB	Connected to GND
9 dB	Unconnected (Default)
6 dB	Connected to VDD/Vin
3 dB	Connected to VDD/Vin through a 100kΩ resistor
```

## Current calibration

You can call this to calibrate current sensing automatically:
```
ds2438Primary.writeCurrentOffset(1);
```
Note that it should only be called with zero current draw across the shunt, ie you could short out the shunt resistor.

## Voice

### Setup

Set this to true in the config file
```
// Activate/Deactivate built in I2S 
#define ESP32_I2S_ACTIVE true
```

### Modifying

For Arduino Flite, you can change the speed and pitch of the voice within the library for the default voice.

int_f0_target_mean = change pitch
int_f0_target_stddev = makes it more flat and robotic
duration_stretch = change speed

These values are a good fit for the robot. Look in cmu_us_kal:
    /* Intonation */
    flite_feat_set_float(v->features,"int_f0_target_mean", 100.0); // JAMES was 95.0
    flite_feat_set_float(v->features,"int_f0_target_stddev", 7.0); // JAMES was 11.0 = lower is more robotic

    flite_feat_set_float(v->features,"duration_stretch", 1.17);  // JAMES: changed from 1.1

### Problems with Flite library

Although the speech code is non-blocking, and the watchdog timer is set to 15 sec and doesn't seem to be triggering, the network thread shuts down after about 5 sec. So best to keep speech short until this gets properly debugged.

## ESP32 VROOM considerations

This firmware should work "as is" with ESP32-S3. But if you use a 1st-gen ESP32, not an S3, there are certain limitations in the pin choices:

 - pin 35 gets a clock signal, so don't use it for momentary switches
 - PSRAM: Make sure PSRAM is enabled in Arduino IDE settings! Otherwise GPIO 17 will be floating and psram will not be detected.
 - switch 0 and status LED ARE NOW TAKEN UP BY THE PSRAM. These have been bodge wired on the current board.
