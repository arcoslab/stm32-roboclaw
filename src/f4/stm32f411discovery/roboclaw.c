/*
 * Copyright (C) 2018 ARCOS-Lab Universidad de Costa Rica
 * Author: Stuart Leal Quesada <stuart.leal23@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdint.h>
#include <stdio.h>
#include <libopencm3/stm32/usart.h>

unsigned int crc16(unsigned char *packet, int nBytes) {
  unsigned int crc=0;
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

  char cmd = 24; // CMD value for read main battery
  usart_send_blocking(USART2, address);
  usart_send_blocking(USART2, cmd);

  for(int i=0; i<2; i++) {
    rcv = usart_recv_blocking(USART2);
    fprintf(stdout, " %u" , rcv);
  } // first two bytes are the battery voltage

  unsigned char crc_rcv[2]; // received crc value from roboclaw
  data[1] = usart_recv_blocking(USART2);
  data[0] = usart_recv_blocking(USART2);

  fprintf(stdout, " %u", data[1]);
  fprintf(stdout, " %u". data[0]);
}




