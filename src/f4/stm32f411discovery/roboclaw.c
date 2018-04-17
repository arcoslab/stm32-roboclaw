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
  a = usart_recv_blocking(USART2);
  fprintf(stdout, "%u", a);

  usart_send_blocking(USART2, 21);
  a = usart_recv_blocking(USART2);
  //fprintf(stdout, "%u", a);

  unsigned char *packet = output + address;
  usart_send_blocking(USART2, crc16(packet, 2));
  a = usart_recv_blocking(USART2);
  //a = usart_recv_blocking(USART2);
  //fprintf(stdout, "%u", a);

}


