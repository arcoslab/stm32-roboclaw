#ifndef TIMERS_H
#define TIMERS_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

/* This value indicates the autoreload for the timer counter */
#define PERIOD 65535

void tim_init(void);

#endif
