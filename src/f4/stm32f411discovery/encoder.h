#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <libopencm3/stm32/timer.h>
#include "systick.h"
#include "motor.h"
#include "filter.h"

void encoder_init(encoder *encoder_x);
bool encoder_update(motor *motor_x);

#endif
