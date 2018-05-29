#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/usart.h>
#include "usart.h"
#include "motor.h"

#define DRIVE_FWD_1 0
#define DRIVE_BWD_1 1
#define DRIVE_FWD_2 4
#define DRIVE_BWD_2 5
#define GET_FIRMWARE 21
#define GET_MAIN_BATT 24

uint16_t crc16(unsigned char *packet, int nBytes);
bool read_firmware(char* output, motor motor_x);
bool read_main_battery(float *voltage, motor motor_x);
bool drive_motor(motor motor_x, int16_t vel);
bool drive_motor_fwd_bwd(motor motor_x, uint8_t value, bool direction);

#endif
