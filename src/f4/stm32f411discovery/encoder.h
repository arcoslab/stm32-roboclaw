#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "systick.h"

typedef struct encoder {
  volatile uint64_t systick_counter;
  volatile uint32_t past_timer_counter;
  volatile uint32_t current_timer_counter;
  volatile int64_t past_pos;
  volatile int64_t current_pos;
  volatile int64_t past_vel;
  volatile float current_vel;
  volatile float current_accel;
  volatile bool uif;
} encoder;

void encoder_init(encoder *encoder_x);
bool encoder_update(encoder *encoder_x, uint16_t current_timer_counter, bool uif);

#endif
