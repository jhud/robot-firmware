#ifndef FAST_IO_H_
#define FAST_IO_H_

#if ESP_ARDUINO_VERSION_MAJOR >= 3
#include <soc/gpio_struct.h>
#else
#include <driver/rtc_io.h>
#endif

// Arduino digitalWrite is super slow. We can get a huge speed improvement by manipulating the
// ESP32 bits directly.

static inline __attribute__((always_inline)) void set_gpio(int pin) {
    if ( pin < 32 ) {
      GPIO.out_w1ts = ((uint32_t)1 << pin); // write 1 to set
    }
    else {
      GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32)); // write 1 to clear
    }
}

static inline __attribute__((always_inline)) void clear_gpio(int pin) {
      if ( pin < 32 ){
    GPIO.out_w1tc = ((uint32_t)1 << pin);
      }
      else {
        GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32));
      }
}

#endif // FAST_IO_H_
