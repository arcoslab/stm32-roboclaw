#include <stdint.h>

unsigned int crc16(unsigned char *packet, int nBytes) {
  unsigned int crc;
  for (int byte = 0; byte < nBytes; byte++) {
    crc = crc ^ ((unsigned int)packet[byte] << 8);
    for (unsigned char bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

int read_firmware(char* output, uint8_t address) {
  // Mandar un address y 21 con el checksum al final                
}


