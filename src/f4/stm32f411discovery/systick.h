#ifndef SYSTICK_H
#define SYSTICK_H

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define VEL_TIMEOUT 5000 // 50ms
#define SYS_TICK_AUTORELOAD 1200
#define TICKS_TIME 0.00001
#define TICKS_PER_REV 3408

void systick_init(void);

#endif
