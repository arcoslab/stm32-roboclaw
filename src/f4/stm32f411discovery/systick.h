#ifndef SYSTICK_H
#define SYSTICK_H

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define SYS_TICK_AUTORELOAD 4799
#define TICKS_TIME 0.00004

void systick_init(void);

#endif
