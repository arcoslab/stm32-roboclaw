#ifndef TIMERS_H
#define TIMERS_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

typedef struct timer {
  enum rcc_periph_clken clken;
  enum rcc_periph_clken clken_timer;
  uint32_t gpio_port;
  uint32_t gpio_pin;
  uint32_t period;
  uint32_t peripheral;
  uint8_t gpio_af;
  enum tim_ic_id ic1;
  enum tim_ic_input in1;
  enum tim_ic_id ic2;
  enum tim_ic_input in2;
  uint8_t mode;
} timer;

void tim_init(timer timer_x);

#endif
