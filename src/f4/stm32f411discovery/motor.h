#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"
#include "timers.h"
#include "usart.h"


typedef struct motor {
  /* Virtual make motors appears as different
   * and the contents inside this struct will manage
   * to use the right usart com port and address
  */
  uint8_t address; // address to corresponding roboclaw in that port
  bool code; // each roboclaw has two motors. Choose between them
  encoder encoder; // encoder related to this motor
  usart_port port; // usart port where the roboclaw for this motor is
  timer timer; // timer in stm for the encoder
} motor;

#endif
