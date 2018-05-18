#ifndef SERIAL_ROBOCLAW_H
#define SERIAL_ROBOCLAW_H

#include <stdint.h>
#include <stdbool.h>

#define DRIVE_FWD_1 0
#define DRIVE_BWD_1 1
#define DRIVE_FWD_2 4
#define DRIVE_BWD_2 5
#define GET_FIRMWARE 21
#define GET_MAIN_BATT 24

bool read_firmware(char* output, uint8_t address);
bool move_motor(bool motor, uint8_t address, uint8_t value, bool direction);
bool read_main_battery(float *voltage, uint8_t address);
uint16_t crc16(unsigned char *packet, int nBytes);

#endif
