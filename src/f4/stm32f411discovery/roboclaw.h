#ifndef SERIAL_ROBOCLAW_H
#define SERIAL_ROBOCLAW_H

#define TIMEOUT_ERROR 1
#define SUCCESS 0

int read_firmware(char* output, uint8_t address);

#endif
