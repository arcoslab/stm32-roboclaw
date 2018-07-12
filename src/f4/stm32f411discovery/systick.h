#ifndef SYSTICK_H
#define SYSTICK_H

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define SYS_TICK_AUTORELOAD 11999
#define TICKS_TIME 0.0001

void systick_init(void);

#endif
