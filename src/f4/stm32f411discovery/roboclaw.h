#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/usart.h>
#include "usart.h"

#define DRIVE_FWD_1 0
#define DRIVE_BWD_1 1
#define DRIVE_FWD_2 4
#define DRIVE_BWD_2 5
#define GET_FIRMWARE 21
#define GET_MAIN_BATT 24

typedef struct motor {
  /* Virtual make motors appears as different
   * and the contents inside this struct will manage
   * to use the right usart com port and address
  */
  uint32_t usart; // com port defined by the usart init
  uint8_t address; // address to corresponding roboclaw in that port
  bool motor; // each roboclaw has two motors. Choose between them
} motor;

uint16_t crc16(unsigned char *packet, int nBytes);
bool read_firmware(char* output, uint8_t address);
bool read_main_battery(float *voltage, uint8_t address);
bool drive_motor(motor motor_x, int16_t vel);
bool drive_motor_fwd_bwd(bool motor, uint8_t address, uint8_t value, bool direction);

#endif
