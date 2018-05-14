#ifndef SERIAL_ROBOCLAW_H
#define SERIAL_ROBOCLAW_H

bool read_firmware(char* output, uint8_t address);
bool move_motor(bool motor, uint8_t address, uint8_t value, bool direction);
bool read_main_battery(float *voltage, uint8_t address);
uint16_t crc16(unsigned char *packet, int nBytes);

#endif
