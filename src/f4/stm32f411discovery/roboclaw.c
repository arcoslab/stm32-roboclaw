#include <stdint.h>
#include <stdio.h>
#include <libopencm3/stm32/usart.h>

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
  unsigned int a = 0;
    
  usart_send_blocking(USART2, address);
  usart_send_blocking(USART2, 21);

  unsigned char *packet = output + address;
  fprintf(stdout, "B: ");
  for(int i=0; i<29; i++) {
    a = usart_recv_blocking(USART2);
    fprintf(stdout, "%u ", a);

  }
}

int read_main_battery(char* output, uint8_t address) {
  unsigned int rcv = 0;

  usart_send_blocking(USART2, address);
  usart_send_blocking(USART2, 24);

  unsigned char *packet = address + 24;
  short checksum = crc16(packet, 2);
  char high_byte = checksum >> 8;
  char low_byte = checksum;
  fprintf(stdout, "%u %u", high_byte, low_byte);
  
  for(int i=0; i<4; i++) {
    rcv = usart_recv_blocking(USART2);
    fprintf(stdout, " %u" , rcv);
  }
}




