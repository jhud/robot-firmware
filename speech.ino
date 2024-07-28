#ifdef HIGH_QUALITY_SPEECH


// -- Speech ---
#include "flite_arduino.h" 

  const i2s_config_t i2s_config_default = {
                .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
                .sample_rate = 8000,
                .bits_per_sample = (i2s_bits_per_sample_t)16,
                .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
                .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
                .intr_alloc_flags = 0, // default interrupt priority
                .dma_buf_count = 8,
                .dma_buf_len = 64,
                .use_apll = false
            };

#ifdef FIRST_ROBOT
 static const i2s_pin_config_t pin_config_default = {
                .bck_io_num = 48,
                .ws_io_num = 38,
                .data_out_num = 47,
                .data_in_num = I2S_PIN_NO_CHANGE
            };
#else
           static const i2s_pin_config_t pin_config_default = {
                .bck_io_num = 27,
                .ws_io_num = 25,
                .data_out_num = 26,
                .data_in_num = I2S_PIN_NO_CHANGE
            };
#endif

FliteOutputI2S *fliteOutput = new FliteOutputI2S(I2S_NUM_0, i2s_config_default, pin_config_default);

Flite flite(fliteOutput);


void say(const char * x)
{ 
  if (strlen(x) == 0) {
    Serial.println(F("Warning: cannot say empty string."));
    return;
  }
  setBusyLED();
  flite.say(x);
  fliteOutput->close(); // Stop hum. Output call in Flite will reopen this.
  clearBusyLED();
}

void say(const __FlashStringHelper * str)
{ 
  char buff[MAX_SPEECH_LEN+1];
  strncpy_P(buff, (PGM_P)str, MAX_SPEECH_LEN);
  say(buff);
}


// Speech multithreading - needs to not block everything
TaskHandle_t speakTask;
SemaphoreHandle_t speakSemaphore = xSemaphoreCreateMutex();
char speechBuffer[MAX_SPEECH_LEN+1];


static void speakTaskLoop(void * param) {
  char input[MAX_SPEECH_LEN+1];
    while(1) {
    if( xSemaphoreTake( speakSemaphore, portMAX_DELAY ) == pdTRUE ) {
        if (speechBuffer[0] != 0) {
          strncpy(input, speechBuffer, MAX_SPEECH_LEN);
          speechBuffer[0] = 0;
        }
        xSemaphoreGive( speakSemaphore );
    }
    if (input[0] != 0) {
      say(input);
      input[0] = 0;
      clientSendString(PSTR("S0\n"));
  }
    delay(50);
    }
}

void speechBegin() {
xTaskCreatePinnedToCore(
      speakTaskLoop, /* Function to implement the task */
      "speak", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      tskIDLE_PRIORITY,  /* Priority of the task */
      &speakTask,  /* Task handle. */
      1); /* Core where the task should run */
}

void queueSpeech(const char * input) {
  if (xSemaphoreTake( speakSemaphore, portMAX_DELAY ) == pdTRUE ) {
    if (speechBuffer[0] != 0) {
      Serial.println(F("ERROR: speech is already active! Discarding"));
      xSemaphoreGive( speakSemaphore );
      SET_ERROR();
      return;
    }
  
    strncpy(speechBuffer, input, MAX_SPEECH_LEN);
    xSemaphoreGive( speakSemaphore );
  }
}

void queueSpeech(const __FlashStringHelper * str)
{ 
  char buff[MAX_SPEECH_LEN+1];
  strncpy_P(buff, (PGM_P)str, MAX_SPEECH_LEN);
  queueSpeech(buff);
}

#endif
