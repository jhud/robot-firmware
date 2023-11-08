/*
 * ESP32 S3 Robot Controller
 * (c) 2022-23 James Hudson
 * 
 *  Note that for S3-N16R8 SPI flash must be Quad, and PSRAM is Oct.
 *  
 *  You can log into the robot on the command line with this: nc <robot IP address, ie 192.168.x.x > 1234
 *  
 *  USB SUPPORT
 *  Set USB CDC On Boot to Enabled in the Tools menu to enable logging/building through the USB port instead of UART.
 *  Note the UART will not output stdout after this.
 *  
 */

#define USE_OTA
//#define USE_IR
#define HIGH_QUALITY_SPEECH
#define USE_VL53l0X
#define USE_DS2438
//#define USE_IMU
//#define I2C_SCAN

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#ifdef USE_OTA
#include <ArduinoOTA.h>
#endif

#include <esp_task_wdt.h>

// To tweak wifi performance / stop intrrupt on pin 36 conflict
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

#include <AsyncTCP.h>

#include <Arduino.h>

#include "mpu6050.h"
#include "fast_io.h"

#define WDT_TIMEOUT_S 15 // Overrride the OS default

#define STATUS_LED_PIN 17
#define ERROR_LED_PIN 12
#define INFO_LED_PIN 39
#define BUSY_LED_PIN 41

#define FLASH_SWITCH_PIN 0  // Flashing switch GPIO0. Note that setting an interrupt on this breaks I2S, and it always outputs 2MHz square wave wit I2S for some reason

#define TOGGLE_SWITCH_PIN 15 
//#define OPTION_PUSHBUTTON_PIN 11 // For some reason, this pin bleeds over into GPIO 12. Even with a minimal example sketch. Seems OK on other S3 in test jig. Maybe broken ESP32? 
#define KILLSWITCH_PUSHBUTTON_PIN 13

#define FOOT_1_PIN 1
#define FOOT_2_PIN 5
#define FOOT_3_PIN 16 // Is a strapping pin on ESP32-S3. Switched to GPIO16 on the latest board revision

#define KILLSWITCH_BUTTON_BITMASK (1 << 8) // bitmask of pushbutton in "pressed" bitfield
#define OPTION_BUTTON_BITMASK (1 << 9) // bitmask of pushbutton in "pressed" bitfield

#define IR_PIN 16

const int MAX_SPEECH_LEN = 128; // Max speech len in characters - the network seems to have problems handling much longer than 128 so let's cap it: this is enough for a sentence. Also we don't want to dealock the CPU for a huge length of time.
const int MAX_RESPONSE_PACKET_LEN = 256; // Max respone len in bytes

#ifndef HIGH_QUALITY_SPEECH
#define say(x) Serial.println(x)
#define speechBegin() // nothing
#endif

// Note by default pin 36 is used by internal ESP32 wifi power management code, so we need to set the wifi mode before we can USE_it

void setBusyLED() {
  clear_gpio(BUSY_LED_PIN);
  //digitalWrite(BUSY_LED_PIN, 1);
  }
  
void clearBusyLED() {
    set_gpio(BUSY_LED_PIN);
  //digitalWrite(BUSY_LED_PIN, 0);
}

// Function declarations
uint32_t parseCommand(char * packet, int length, unsigned int responseBufferSize, char * responseBuffer);
void updateFootSwitches();
void servoTaskLoop(void *param);

AsyncServer server(1234);

#define NUM_OF(x) (sizeof(x)/sizeof(x[0]))

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define FP_SHIFT 12          // How many fixed point digits we have
#define FP_ONE (1<<FP_SHIFT) // The FP unit

static const int NUM_SERVOS = 32;

int32_t servoStart[NUM_SERVOS];
int32_t servoTarget[NUM_SERVOS];
int32_t servoProgress[NUM_SERVOS];
int32_t servoProgressSpeed[NUM_SERVOS];

int32_t servoCurrent[NUM_SERVOS]; // For interrupts, etc.
uint16_t servoSwitchInterruptMask; // which switches will halt movement. Zero for always finish movement regardless of switches.

// Last recorded tick. Set to millis on startup
static unsigned int ticksLast = 0;

// Servo multithreading - needs to run fast + realtime
TaskHandle_t servoTask;
SemaphoreHandle_t servoSemaphore = xSemaphoreCreateMutex();

// Set this to true to flash the error LED
#define SET_ERROR(x) { set_gpio(ERROR_LED_PIN); errorCondition = 20; }
int errorCondition;

// --- Input IO
volatile uint16_t pressed; // volatile: USE_RAM instead of register, for USE_inside of interrupts

// Logic for detecting an IR beacon. Beacon must be 38kHz, and pulse modulated at greater than BEACON_TIMEOUT_MS
// The self-made beacon has an on pulse of 120ms - when the signal is strong, this is solid, if it is weak or a reflection, it has many missing sections

#ifdef USE_IR
volatile unsigned int beacon_first_seen_time;
volatile unsigned int beacons_seen_in_interrupt;
unsigned int clean_pings;
unsigned int beacon_on;
unsigned const int REQUIRED_CLEAN_PINGS = 2;
#endif

#ifdef USE_VL53l0X
// Optionally smooth out the LIDAR samples, to avoid noise and transients.
double cumulativeRangeMM = 0;
int numRangeSamples = 0;
#endif

struct RobotSettings {
  uint32_t statusFrequencyMs = 8000; // How often to send status updates, battery power, RAM etc.
  uint32_t beaconTimeoutMs = 2000; // How long to wait between sightings for saying that an IR beacon is not seen.
  uint32_t rangingFrequencyMs = 1125; // How often to sample the range finder
  uint32_t rangingAveraging = 2; // Average out n readings from the range finder
  uint32_t imuFrequencyMs = 1250; // 0 if disabled
  uint32_t buttonInterruptToServoMapping[6]; // Used to map buttons to servo interrupts
};

RobotSettings settings;

// default address 0x40, we need the other
TwoWire primaryI2C = TwoWire(1); // Setup on default pins 21,22, port 1 of (0-1), defaults to GPIOs 8,9 on S3

#define PCA9685_SECONDARY_I2C_ADDRESS 0x41      /**< Secondary PCA9685 I2C Slave Address */
Adafruit_PWMServoDriver pwm[] {
  Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, primaryI2C),
  Adafruit_PWMServoDriver(PCA9685_SECONDARY_I2C_ADDRESS, primaryI2C)
};

/**
 * Set servo, addressing the correct PWM interface.
 */

const int PWM_CYCLE_LENGTH = 4096; // From the hardware

// Space out the servo control pulses so they do not happen all at once, otherwise we get a slight ripple on the power supply.
const int SERVO_PHASE_SEPARATION = 7 * (PWM_CYCLE_LENGTH-SERVOMAX-1) / NUM_SERVOS; // Spread them out by a prime number, so they do not cluster
 
void set_pwm(int servo, int val) {
  int offset = (servo * SERVO_PHASE_SEPARATION) % PWM_CYCLE_LENGTH;
  pwm[servo/16].setPWM(servo&15, offset, (val+offset) % PWM_CYCLE_LENGTH);
}


void log(const String str) {
  if (digitalRead(TOGGLE_SWITCH_PIN)) {
    Serial.println(str);
  }
  else {
    say(str.c_str());
  }
}

void log(const __FlashStringHelper * str) {
    if (digitalRead(TOGGLE_SWITCH_PIN)) {
    Serial.println(str);
  }
  else {
    say(str);
  }
}


bool lowBattery(double batteryVoltage) 
{
  return batteryVoltage < 6.0;
}

void relaxOne(int servo) {
      if( xSemaphoreTake( servoSemaphore, portMAX_DELAY ) == pdTRUE ) {
        servoTarget[servo] = servoCurrent[servo] = servoProgressSpeed[servo] = 0;
        servoProgress[servo] = FP_ONE;
        set_pwm(servo, 0);
              xSemaphoreGive( servoSemaphore );
      }
}

void relaxAll() {
    for (int i=0; i<NUM_SERVOS;i++) {
        relaxOne(i);
    }
}


void haltAll() {
        if( xSemaphoreTake( servoSemaphore, portMAX_DELAY ) == pdTRUE ) {
    for (int i=0; i<NUM_SERVOS;i++) {
        servoTarget[i] = servoCurrent[i];
        servoProgress[i] = FP_ONE;
    }
                  xSemaphoreGive( servoSemaphore );
      }
}

// --- Servo interpolation

int32_t servo_interpolate_linear(int32_t servo, int32_t position) {
    int32_t diff = (servoTarget[servo] - servoStart[servo]);
    diff = ((diff >> 3) * (position >> 2)) >> (FP_SHIFT-5);
    return diff + servoStart[servo];
}

int32_t interpolate_bezier(int32_t position) {
    const int32_t position_2 = (position * position) >> FP_SHIFT;
    const int32_t term = (3 << FP_SHIFT) - (2 * position);
    return  (position_2 * term) >> FP_SHIFT;
}

void servo_setTarget(int servo, int32_t val, int duration) {
  if (val == 0) {
    // Relax
    relaxOne(servo);
    return;
  }
    if( xSemaphoreTake( servoSemaphore, portMAX_DELAY ) == pdTRUE ) {
  
      if (servoCurrent[servo] == 0) {
          servoStart[servo] = (val<<FP_SHIFT)-1;
          servoCurrent[servo] = servoStart[servo];
      }
      else {
          servoStart[servo] = servoCurrent[servo];
      }
      servoTarget[servo] = (val << FP_SHIFT);
      servoProgress[servo] = 0;
      servoProgressSpeed[servo] = (1<<FP_SHIFT) * 1000 / duration;
      xSemaphoreGive( servoSemaphore );
    }
}


// --- TCP/IP server
AsyncClient * client;

static void clientSend(const char * sendData, unsigned int len)
{
  if (!client) {
    say(F("Bueller"));
    return;
  }
  
  if (client->space() >= len && client->canSend())
  {
    client->add(sendData, len);
    client->send();
  }
}

static void clientSendString(const char * str)
{
  clientSend(str, strlen(str)); 
}

static void handleData(void *arg, AsyncClient *clientSending, void *data, size_t len)
{
  char parseResponse[MAX_RESPONSE_PACKET_LEN];
  
  //Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());
  //Serial.write((uint8_t *)data, len);

  uint32_t totalParsed = 0;
  char* dataPtr = (char*)data;
  while (totalParsed < len) {
    uint32_t bytesParsed = parseCommand(dataPtr, len-totalParsed, sizeof(parseResponse), parseResponse);
    dataPtr += bytesParsed;
    totalParsed += bytesParsed; 
  }

  clientSendString(parseResponse);
  //Serial.print("Done parsing data.");
}

static void handleError(void *arg, AsyncClient *client, int8_t error)
{
  Serial.println(F("connection error from client")); //, client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleDisconnect(void *arg, AsyncClient *clientDisconnecting)
{
  Serial.println(F("client disconnected"));
  client = NULL; // @todo how is client freed? Memory leak?
  relaxAll();
}

static void handleNewClient(void *arg, AsyncClient *newClient);

static void handleTimeOut(void *arg, AsyncClient *timedOutClient, uint32_t time)
{
  // Seems to be fatal - restart the connection brutally
  timedOutClient->close();
  Serial.print(F("\n client ACK timeout time: ")); // Note that client info seems to be invalid.
  Serial.println(time);
  server.end();
  server.begin();
}

static void handleNewClient(void *arg, AsyncClient *newClient)
{
  if (client) {
    client->stop(); // @todo delete client?
  }
  client = newClient;
  client->setAckTimeout(WDT_TIMEOUT_S*1000); // Think it was timing out from speech and deadlocking the connection.
  client->setRxTimeout(WDT_TIMEOUT_S*1000);
  Serial.print(F("New client has been connected to server, ip: "));
  Serial.println(client->remoteIP().toString());
  log(F("Client connected."));
  // register events
  client->onData(&handleData, NULL);
  client->onError(&handleError, NULL);
  client->onDisconnect(&handleDisconnect, NULL);
  client->onTimeout(&handleTimeOut, NULL);
  clientSendString(PSTR("ONLINE\n"));
}


void sendButtonMask(int mask) {
  char buff[64];
  snprintf_P(buff, sizeof(buff), PSTR("B%d\n"), mask);
  if (client) {
    clientSendString(buff);
  }
  else {
    queueSpeech(buff);
  }
}

/**
 * @return num of bytes parsed.
 */
uint32_t parseCommand(char * packet, int length, unsigned int responseBufferSize, char * responseBuffer) {
  char * cursor = packet;
  uint8_t cmd = *(cursor);
  const unsigned int maxStringSize = responseBufferSize-1;
  uint32_t bytesParsed = length; // By default, if we don't verify length, just throw away extra bytes.

#ifdef INFO_LED_PIN
    static char ch;
    digitalWrite(INFO_LED_PIN, ch&1);
    ch++;
#endif

  packet[length] = 0;

  cursor ++;

  if(cmd=='S') {
    if (length > MAX_SPEECH_LEN) {
      snprintf_P(responseBuffer, maxStringSize, PSTR("S0 too long: %d\n"), length);
    }
    else {
#ifdef HIGH_QUALITY_SPEECH
// Do not block network task - this should be fast
        snprintf_P(responseBuffer, maxStringSize, PSTR("S1\n"));
        queueSpeech(cursor);
#endif
    }
  }
  else if(cmd=='X') {
    relaxAll();
      snprintf_P(responseBuffer, maxStringSize, PSTR(">Relaxed all\n"));
      bytesParsed = 1;
  }
  else if(cmd=='H') {
      haltAll();
      snprintf_P(responseBuffer, maxStringSize, PSTR(">Halted movement\n"));
      bytesParsed = 1;
  }
  else if(cmd=='R') {
    if (length < sizeof(settings)+1) {
      snprintf_P(responseBuffer, maxStringSize, PSTR(">Wrong length for settings\n"));
    }
    else {
      memcpy((void*)&settings, cursor, sizeof(settings));
      snprintf_P(responseBuffer, maxStringSize, PSTR(">Changed settings\n"));
      bytesParsed = sizeof(settings);
    }
  }
  else if(cmd=='P') {
      snprintf_P(responseBuffer, maxStringSize, PSTR(">PING to %s:%d\n"), client->remoteIP().toString().c_str(), client->remotePort());
      bytesParsed = 1;
  }
    else if(cmd=='M') {
      uint32_t timestamp = *((uint32_t*)cursor);
      cursor += sizeof(uint32_t);
      uint32_t servoMask = *((uint32_t*)cursor);
      cursor += sizeof(uint32_t);
      servoSwitchInterruptMask = *((uint32_t*)cursor); 
      cursor += sizeof(uint32_t);
      
      uint16_t * cursor16 = (uint16_t*)cursor;
      for (int servo=0; servo<NUM_SERVOS; servo++) {
        if (servoMask&1) {
          const uint16_t val = *(cursor16++);
          const uint16_t duration = *(cursor16++);

          if (val == 0) {
            relaxOne(servo);
          }
          else if (servo < NUM_SERVOS && (val >= SERVOMIN && val <= SERVOMAX)) {
            servo_setTarget(servo, val, duration);
          }
          else {
          }
        }
        servoMask >>= 1;
      }
      
      snprintf_P(responseBuffer, maxStringSize, PSTR("M%d,%d\n"), timestamp, servoSwitchInterruptMask);
  }
  else {
      snprintf_P(responseBuffer, maxStringSize, PSTR(">UNKNOWN\n"));
  }

  return bytesParsed;
}


void IRAM_ATTR switch7() {
  pressed |= KILLSWITCH_BUTTON_BITMASK;
}

void IRAM_ATTR switchOption() {
  pressed |= OPTION_BUTTON_BITMASK;
}

#ifdef USE_IR
void IRAM_ATTR irTriggered() {
  if (beacon_first_seen_time == 0) {
   beacon_first_seen_time = millis();
  }
  
  beacons_seen_in_interrupt++;
}
#endif

void runI2CScan(TwoWire & target) {
    int nDevices = 0;
      for(byte address = 0x1; address < 0x80; address++ ) {
    target.beginTransmission(address);
    byte error = target.endTransmission();
    if (error == 0) {
      log(F("Device found at hex "));
      log(String(address, HEX));
      Serial.flush();
      nDevices++;
    }
    else if (error==4) {
      log(F("Unknown error at hex"));
      log(String(address, HEX));
    }
  }
  if (nDevices == 0) {
    log(F("No I2C devices found"));
  }
  else {
    log(F("done"));
  }
}

void setup_gpios() {
  pinMode(FLASH_SWITCH_PIN, INPUT_PULLUP);  // We can USE_flashing switch for UI
  
#ifdef STATUS_LED_PIN
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif 

#ifdef ERROR_LED_PIN
  pinMode(ERROR_LED_PIN, OUTPUT);
  clear_gpio(ERROR_LED_PIN);
#endif

#ifdef BUSY_LED_PIN
  pinMode(BUSY_LED_PIN, OUTPUT);
  digitalWrite(BUSY_LED_PIN, 1);
#endif

#ifdef INFO_LED_PIN
  pinMode(INFO_LED_PIN, OUTPUT);
  digitalWrite(INFO_LED_PIN, 0);
#endif



    // Foot switches
  pinMode(FOOT_1_PIN, INPUT_PULLUP); // 17 on old board used by PSRAM
  pinMode(FOOT_2_PIN, INPUT_PULLUP);
  pinMode(FOOT_3_PIN, INPUT_PULLUP); 
  pinMode(4, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
      pinMode(2, INPUT_PULLUP);
    pinMode(7, INPUT_PULLUP);
    pinMode(10, INPUT_PULLUP);

  pinMode(KILLSWITCH_PUSHBUTTON_PIN, INPUT_PULLUP); // GP push button switch for UI
  attachInterrupt(KILLSWITCH_PUSHBUTTON_PIN, switch7, FALLING); // USE_an interrupt for this responsive UI switch

  #ifdef OPTION_PUSHBUTTON_PIN
  pinMode(OPTION_PUSHBUTTON_PIN, INPUT_PULLUP); // GP push button switch for UI
   attachInterrupt(OPTION_PUSHBUTTON_PIN, switchOption, FALLING); // USE_an interrupt for this responsive UI switch
  #endif
  
  pinMode(TOGGLE_SWITCH_PIN, INPUT_PULLUP);

#ifdef USE_IR
  pinMode(IR_PIN, INPUT);
  attachInterrupt(IR_PIN, irTriggered, CHANGE);
#endif


  updateFootSwitches();
  pressed = 0;
}

void setup()
{
  int ledStatus = 0;

  Serial.begin(115200);

  delay(200);
  Serial.println(F("Starting up robot..."));
  delay(200);

  setup_gpios();

  if(psramInit() == false) {
    log(F("Warning: PS RAM missing"));
  }

  //flite.setVoice(register_cmu_us_kal(nullptr));
  //flite.setVoice(register_cmu_us_slt(nullptr));

//      cst_voice *register_cmu_us_kal(const char*voxdir=nullptr); // default
//    cst_voice *register_cmu_us_kal16(const char* voxdir=nullptr);
//    cst_voice *register_cmu_us_slt(const char *voxdir=nullptr);
//    cst_voice *register_cmu_us_rms(const char *voxdir=nullptr);
//    cst_voice *register_cmu_us_awb(const char *voxdir=nullptr);
//    cst_voice *register_cmu_time_awb(const char *voxdir=nullptr);
           
  log(F("Firmware:"));      
  log(__DATE__);      

#ifdef USE_DS2438
  // --- OneWire setup
  if (ds2438_init() == false) {
    log(F("ERROR: DS2438 failure."));
    while(1) {
      set_gpio(ERROR_LED_PIN);
      delay(200);
      clear_gpio(ERROR_LED_PIN);
      delay(100);
    }
  }
 #endif

#ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, (++ledStatus)&1); 
#endif

  log(F("Setting up I2C"));

  // --- TwoWire 0 (I2C) setup. The other I2C is setup by the PWM code.
  primaryI2C.setPins(21, 18); // WIRE_1_SDA WIRE_1_SCL
  Wire.begin(); // default of GPIOS 8 and 9 should be OK - WIRE_0
  primaryI2C.begin(); // the PWM library usually sets this up.

#ifdef I2C_SCAN
  { // I2C scan is fast, always do it
    log(F("Primary I2C scan."));
    runI2CScan(primaryI2C);

    log(F("Secondary I2C scan."));
    runI2CScan(Wire);
  }
  #else
#endif

  for (int i=0; i<NUM_OF(pwm); i++) {
    pwm[i].begin();
    Serial.println(F("PWM begun."));
    pwm[i].setOscillatorFrequency(27000000);
    pwm[i].setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    Serial.println(F("Setup PWM controller."));
  }
  delay(10);

  xTaskCreatePinnedToCore(
        servoTaskLoop, /* Function to implement the task */
        "servo", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        9,  /* Priority of the task */
        &servoTask,  /* Task handle. */
        0); /* Core where the task should run */


   speechBegin();


    if (mpu6050_init() == false) {
      settings.imuFrequencyMs = 0;
    }

    

#ifdef STATUS_LED_PIN
    digitalWrite(STATUS_LED_PIN, (++ledStatus)&1); 
#endif


#ifdef USE_VL53l0X
  if (vl53l0x_init() == false) {
    log(F("ERROR: VL53l0X failure."));
    while(1) {
      set_gpio(ERROR_LED_PIN);
      delay(200);
      clear_gpio(ERROR_LED_PIN);
      delay(100);
      set_gpio(ERROR_LED_PIN);
      delay(200);
      clear_gpio(ERROR_LED_PIN);
      delay(1000);
    }
  }
  log(F("VL53l0X okay."));

#ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, (++ledStatus)&1);
 #endif
#endif

#ifdef USE_DS2438
  // Get first sample of battery data
  ds2438_update();
  double batteryVoltage = ds2438_mainBatteryVoltage();
  double chip5vVoltage = ds2438_ServoAVoltage();
  double vccVoltage = ds2438_VccVoltage();
  if (lowBattery(batteryVoltage)) {
    say(F("Low battery."));
    set_gpio(ERROR_LED_PIN);
  }
  else if(vccVoltage < 3.2 || vccVoltage > 3.4 ) {
    say(F("VCC rail problem."));
    set_gpio(ERROR_LED_PIN);
  }
  else if(chip5vVoltage < 4.95 || chip5vVoltage > 5.15 ) {
    say(F("5 volt rail problem."));
    set_gpio(ERROR_LED_PIN);
  }
  #endif
  log(F("Power okay."));

  
#ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, (++ledStatus)&1); 
#endif
 
  relaxAll();
  log(F("Servos okay."));  

#ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, (++ledStatus)&1);
#endif

  log(F("Wifi search"));

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE); // We are using GPIO pin 36, which uses the ADC interrupt. Uses more power but better wifi performance. https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html

  {
     WiFiManager wm;
  
    if ((pressed & (KILLSWITCH_BUTTON_BITMASK | OPTION_BUTTON_BITMASK)) > 0) {
    queueSpeech(F("Wifi config open"));
     wm.resetSettings();
    }

          bool res;
      res = wm.autoConnect("Robot init"); // anonymous ap

      if (!res) {
          log(F("Wifi fail"));
            while(1) {
            #ifdef ERROR_LED_PIN
              set_gpio(ERROR_LED_PIN);
              delay(1000);
              clear_gpio(ERROR_LED_PIN);
              delay(1000);
             #endif
            }
      } else {
          char numBuff[8];
         say(F("Wifi address"));
          uint8_t lastDigit = WiFi.localIP()[3];
          itoa(lastDigit, numBuff, 10);
          say(numBuff);
      }
  }

#ifdef USE_OTA
  ArduinoOTA.onStart([]() {
   queueSpeech(F("Updating."));
    });
    
 ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static int blinky;
        #ifdef STATUS_LED_PIN
        digitalWrite(STATUS_LED_PIN, blinky&1);
        #endif
        #ifdef BUSY_LED_PIN
          digitalWrite(BUSY_LED_PIN, (blinky+1)&1);
        #endif //BUSY_LED_PIN
        blinky++;
                        });

      ArduinoOTA.onEnd([]() {
   say(F("Update complete."));
    });
                        
  ArduinoOTA.setHostname("Robot");
  ArduinoOTA.begin();

  ArduinoOTA.handle(); // If there is a crash later, give us a chance to catch an update

#endif
#ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, (++ledStatus)&1);
  #endif

        
  server.onClient(&handleNewClient, &server);
  server.begin();
  
  clearBusyLED();
  #ifdef STATUS_LED_PIN
  clear_gpio(STATUS_LED_PIN);
  #endif

  queueSpeech(F("Ready"));

  #ifdef STATUS_LED_PIN
  digitalWrite(STATUS_LED_PIN, (++ledStatus)&1);
#endif

// Speaking may cause WDT problems since it is done in the network callback. Better to have a queue, but this is a quick fix.
       #if CONFIG_ESP_TASK_WDT_TIMEOUT_S < WDT_TIMEOUT_S
         // The following function takes longer than the 5 seconds timeout of WDT
       #if ESP_IDF_VERSION_MAJOR >= 5
         esp_task_wdt_config_t wdtc;
         wdtc.idle_core_mask = 0;
       #if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
         wdtc.idle_core_mask |= (1 << 0);
       #endif
       #if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
         wdtc.idle_core_mask |= (1 << 1);
       #endif
         wdtc.timeout_ms = WDT_TIMEOUT_S*1000;
         wdtc.trigger_panic = true;
         esp_task_wdt_reconfigure(&wdtc);
       #else
         esp_task_wdt_init(15, true);
       #endif
       #endif


  ticksLast = millis();
}


/**
 * Called when a button press interrupted a servo movement
 * 
 * Sends a text packet with:
 * 'I' character
 * button bitfield of button/s which caused the interrupt
 * array of all servo positions
 * 
 */

void onServoInterrupt() {
    char buff[256]; // Big enough to hold all servos

    snprintf_P(buff, sizeof(buff), PSTR("I%d"), pressed & servoSwitchInterruptMask);
    
    if (servoSwitchInterruptMask > 0) {
      if( xSemaphoreTake( servoSemaphore, portMAX_DELAY ) == pdTRUE ) {
        uint32_t checkButton = pressed;
        uint32_t buttonIndex = 0;
        while(checkButton > 0) {
          if (checkButton&1) {
            uint32_t servosToStopMask = settings.buttonInterruptToServoMapping[buttonIndex];
            uint32_t servoNum = 0;
            while (servosToStopMask > 0) {
              if (servosToStopMask&1) {
                servoTarget[servoNum] = servoCurrent[servoNum];
                servoProgress[servoNum] = FP_ONE;
              }
              servosToStopMask >>= 1;
              servoNum ++;
            }
            servoSwitchInterruptMask &= ~(1<<servoNum); // Allow for later interrupts too
          }
          checkButton >>= 1;
          buttonIndex++;
        }

              char numBuff[16];
      for (int i=0; i<NUM_SERVOS; i++) {
        snprintf_P(numBuff, sizeof(numBuff), PSTR(",%d"), servoCurrent[i]>>FP_SHIFT);
        strcat_P(buff, numBuff);
      }
        xSemaphoreGive(servoSemaphore);
      }
    }

    strcat_P(buff, "\n");

    clientSendString(buff);
}


// Stagger these so that they don't all get called in the one tick
unsigned int statusElapsed = 0;
unsigned int sensorsUpdateElapsed = 50;
unsigned int rangingUpdateElapsed = 100;
unsigned int imuUpdateElapsed = 150;

void updateIRBeacon() {
  #ifdef USE_IR
  unsigned int now = millis();
  if (beacons_seen_in_interrupt > 0) {
    const int delta = now-beacon_first_seen_time;
    const int max_allowed_beacons = (delta/60) + 1; // Pulse of our beacon is 120ms
    // max_allowed_beacons / actually gives us strength if we care. We ignore all but the strongest beacons
    const bool got_clean_ping = (beacons_seen_in_interrupt <= max_allowed_beacons);

    if (got_clean_ping) {
        clean_pings++;
        if (clean_pings >= REQUIRED_CLEAN_PINGS) {
          if (beacon_on == 0) {
            // max_allowed_beacons / actually gives us strength if we care. We ignore all but the strongest beacons
            if (client) {
              clientSendString(PSTR("R1\n"));
            }
          }
          beacon_on = now;
        }
    }

    char buff[64];
    snprintf_P(buff, sizeof(buff), PSTR("A%d,%d\n"), beacons_seen_in_interrupt, delta);
    clientSendString(buff);
    
    beacons_seen_in_interrupt = 0;
    beacon_first_seen_time = 0;
  }
  
  if (beacon_on > 0) {
    if (now-beacon_on > settings.beaconTimeoutMs) {
        if (client) {
          clientSendString(PSTR("R0\n"));
        }
        beacon_on = 0;
        beacons_seen_in_interrupt = 0;
        beacon_first_seen_time = 0;
        clean_pings = 0;
    }
  }
  #endif
}


/**
 * Foot switches do not need interrupts, since they are only processed once per tick, and stay pressed
 * a long time.
 */
void updateFootSwitches() {
  static volatile uint16_t lastButtons = 0;

  uint16_t buttonsNow = 0;
  buttonsNow |= !digitalRead(FOOT_1_PIN) << 0;
  buttonsNow |= !digitalRead(FOOT_2_PIN) << 1;
  buttonsNow |= !digitalRead(FOOT_3_PIN) << 2;
  buttonsNow |= !digitalRead(4) << 3;
  buttonsNow |= !digitalRead(6) << 4;
  buttonsNow |= !digitalRead(2) << 5;
  buttonsNow |= !digitalRead(7) << 6;
  buttonsNow |= !digitalRead(10) << 7;

  // ui switch has interrupt, uses bit 8

  buttonsNow |= !digitalRead(FLASH_SWITCH_PIN) << 9;

  pressed |= (buttonsNow ^ lastButtons) & buttonsNow;

  lastButtons = buttonsNow;
}



// Update servo tweening in a separate high-priority thread
void servoTaskLoop(void *param) {
  static unsigned int servoTicksLast = 0;
  while(1) {
    unsigned int ticksNow = millis();

    int delta = (ticksNow - servoTicksLast);

    #ifdef ERROR_LED_PIN
    if (delta == 0) {
      delta = 1;
      SET_ERROR();
    }
    #endif

// @todo do not have semaphore around I2C access
    if( xSemaphoreTake( servoSemaphore, portMAX_DELAY ) == pdTRUE ) {
      for (int servo=0; servo<NUM_SERVOS; servo++) {
        servoProgress[servo] += delta * servoProgressSpeed[servo] / 1000;

        if (servoProgress[servo] < FP_ONE) {
            const int32_t interpolatedProgressTime = interpolate_bezier(servoProgress[servo]);
            servoCurrent[servo] = servo_interpolate_linear(servo, interpolatedProgressTime);
            set_pwm(servo, servoCurrent[servo] >> FP_SHIFT);
        }
        else if (servoTarget[servo] != 0) {
            servoCurrent[servo] = servoTarget[servo];
            set_pwm(servo, servoTarget[servo] >> FP_SHIFT);
        }
      }
            xSemaphoreGive( servoSemaphore );
    } else {
      // Could not get resource
#ifdef ERROR_LED_PIN
      SET_ERROR();
#endif
      continue;
    }

    servoTicksLast = ticksNow;
    delay(20); // 40 is 25Hz, since servos only update at 50hz
  }
}

void loop()
{
  char sendBuff[256]; // Small buffer for sending text update packets to controller
  static unsigned int numTickSamples = 0;
  static unsigned int totalTicks = 0;
  unsigned int ticksNow = millis();

#ifdef USE_OTA
  ArduinoOTA.handle();
#endif

  updateFootSwitches();

    if ((pressed & servoSwitchInterruptMask) > 0 ) {
      onServoInterrupt();
    }

    int delta = (ticksNow - ticksLast);

#ifdef ERROR_LED_PIN
    if (delta == 0) {
      delta = 1;
    SET_ERROR();
    }
    else if (delta>100) {
      delta = 100;
    SET_ERROR();
    }

    if (errorCondition > 0) {
      errorCondition -= 1;
      if (errorCondition == 0) {
          clear_gpio(ERROR_LED_PIN);
      }
    }
#endif

    numTickSamples++;
    totalTicks += delta;    

  ticksLast = ticksNow;

  updateIRBeacon();
  
  if (pressed > 0) { 
    sendButtonMask(pressed);
    if (pressed & KILLSWITCH_BUTTON_BITMASK) {
      relaxAll();
     queueSpeech(F("Kill switch pressed."));
      // On the high-level controller end, we should halt all execution also.
    }
    else if (pressed & OPTION_BUTTON_BITMASK) {
      char buff[64];
      double batteryVoltage = ds2438_mainBatteryVoltage();
      snprintf_P(buff, sizeof(buff), PSTR("Battery %.1f volts"), batteryVoltage);
      queueSpeech(buff);
    }
    pressed = 0;
  }

#ifdef USE_VL53l0X
  if (ticksNow > rangingUpdateElapsed && client) {
      if (client) {
        auto range = vl53l0x_rangeMM();
        if (range < 8000) { // Anything greater is way beyond the max range for this sensor, ie "not found"
          cumulativeRangeMM += range;
          numRangeSamples ++;
          if (numRangeSamples >= settings.rangingAveraging) {
            snprintf_P(sendBuff, sizeof(sendBuff), PSTR("L0,%d\n"), int(cumulativeRangeMM/numRangeSamples));
            clientSendString(sendBuff);
            cumulativeRangeMM = 0;
            numRangeSamples = 0;
          }
        }
      }
      rangingUpdateElapsed = ticksNow + settings.rangingFrequencyMs;
  }
#endif

#ifdef USE_IMU
  if (settings.imuFrequencyMs > 0 && ticksNow > imuUpdateElapsed && client) {
      imuUpdateElapsed = ticksNow + settings.imuFrequencyMs;
      IMUUpdate upd;
      mpu6050_update(upd);
      snprintf_P(sendBuff, sizeof(sendBuff), PSTR("C%f,%f,%f,%f,%f,%f,%f\n"), upd.a.acceleration.x, upd.a.acceleration.y, upd.a.acceleration.z, upd.g.gyro.x, upd.g.gyro.y, upd.g.gyro.z, upd.temp.temperature);
      clientSendString(sendBuff);
  }    
#endif

  if (ticksNow > sensorsUpdateElapsed && client) {
      sensorsUpdateElapsed = ticksNow + settings.statusFrequencyMs / 4;
      ds2438_update();
  }

#ifdef USE_DS2438
  if (ticksNow > statusElapsed + 200) {
    statusElapsed = ticksNow + settings.statusFrequencyMs;

    double batteryVoltage = ds2438_mainBatteryVoltage();

    if (client) {
      auto freeHeap = ESP.getFreeHeap();
      auto freePsram = ESP.getFreePsram();
      
      snprintf_P(sendBuff, sizeof(sendBuff), PSTR("U%d,%d,%d,%f,%f,%f,%f,%f,%f,%d\n"), totalTicks/numTickSamples, freeHeap, freePsram, ds2438_temp(), ds2438_currentAmps(), batteryVoltage, ds2438_ServoAVoltage(),  ds2438_ServoBVoltage(), ds2438_VccVoltage(), WiFi.RSSI());
      clientSendString(sendBuff);
    }

    totalTicks = 0;
    numTickSamples = 0;
    
    if (lowBattery(batteryVoltage)) {
      digitalWrite(ERROR_LED_PIN, 1);
      say(F("Low battery. Shutting down."));
      relaxAll();
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      while(lowBattery(batteryVoltage)) {
        delay(5000);
        say(F("Low battery"));
        ds2438_blockingUpdate();
        batteryVoltage = ds2438_mainBatteryVoltage();
      }
      delay(1000);
     say(F("Battery recovered"));
    }
  }
  #endif

  
  delay(10);
}
