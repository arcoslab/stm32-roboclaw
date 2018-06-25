#ifndef TIMERS_H
#define TIMERS_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include "motor.h"

void tim_init(timer *timer_x);

#endif
