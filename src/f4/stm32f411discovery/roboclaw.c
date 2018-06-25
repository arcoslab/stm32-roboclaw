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


#include "roboclaw.h"
#include "motor.h"

uint16_t crc16(unsigned char *packet, int nBytes) {
  uint16_t crc=0;
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

bool usart_send_blocking_wtimeout(uint32_t usart, unsigned char *data, uint32_t timeout) {
  for(uint32_t i=0; i<timeout; i++){
    if( (USART_SR(usart) & USART_SR_TXE) != 0) {
      usart_send(usart, *data);
      return true;
    }
  }
  return false;
}

bool usart_recv_blocking_wtimeout(uint32_t usart, unsigned char *data, uint32_t timeout) {
  for(uint32_t i=0; i<timeout; i++){
    if( (USART_SR(usart) & USART_SR_RXNE) != 0) {
      *data = usart_recv(usart);
      return true;
    }
  }
  return false;
}


bool drive_motor(motor *motor_x, int16_t vel) {
  /* This wrapper function will call drive motor fwd or bwd. This will support
   * negative and positie values as commands, and will manage the cases when
   * the value is not within the range. Negative values will mean the opposite
   * direction.
   */
  bool direction;
  uint8_t value;

  if(vel >= 0){
    direction = true;
  }
  else {
    direction = false;
  }

  value = (vel>=0 ? vel:-vel);

  return drive_motor_fwd_bwd(motor_x, value, direction);

}

bool drive_motor_fwd_bwd(motor *motor_x, uint8_t value, bool direction) {
  /* Returns true if the motors moves succesfully, or
     false if didn't receive 0xff. motor is 0 for motor 1
     and 1 for motor 2. 127 is for full speed, 64 is about half
     speed, and 0 is full stop. If direction is 0 then it will move
     backwards, if direction is 1 it will move forward.
  */
  bool done = false;

  unsigned char data[6]; // address, cmd, value, 2byte crc

  data[0] = motor_x->address; //first to write is address
  done = usart_send_blocking_wtimeout(motor_x->usart, &data[0], TIMEOUT); //send address

  if (!done) {
    return false;
  }

  if ((motor_x->code == 0) & (direction == 1)) {
    // move motor 1 forward
    data[1] = (unsigned char) DRIVE_FWD_1;
  }
  if ((motor_x->code == 0) & (direction == 0)) {
    // move motor 1 backward
    data[1] = (unsigned char) DRIVE_BWD_1;
  }
  if ((motor_x->code == 1) & (direction == 1)) {
    // move motor 2 forward
    data[1] = (unsigned char) DRIVE_FWD_2;
  }
  if ((motor_x->code == 1) & (direction == 0)) {
    // move motor 2 backwards
    data[1] = (unsigned char) DRIVE_BWD_2;
  }

  done = usart_send_blocking_wtimeout(motor_x->usart, &data[1], TIMEOUT); //send cmd

  if (!done) {
    return false;
  }

  if(value > 127){
    // invalid input value
    return false;
  }

  data[2] = (unsigned char) value;

  done = usart_send_blocking_wtimeout(motor_x->usart, &data[2], TIMEOUT); //send value

  if (!done) {
    return false;
  }

  uint16_t crc_chk = crc16(data, 3);
  data[3] = crc_chk >> 8; // high value byte
  data[4] = crc_chk; // low value byte

  done = usart_send_blocking_wtimeout(motor_x->usart, &data[3], TIMEOUT); //send high byte crc
  if (!done) {
    return false;
  }

  done = usart_send_blocking_wtimeout(motor_x->usart, &data[4], TIMEOUT); //send low byte crc

  if (!done) {
    return false;
  }

  done = usart_recv_blocking_wtimeout(motor_x->usart, &data[5], TIMEOUT);
  if (!done) {
    return false;
  }

  return true;

  if(data[5] == ((char)255)){
    return true; //ack code sent
  }
  else {
    return false;
  }

}

bool read_firmware(char *output, motor *motor_x) {
  /* Returns true if operation was succesful,
     False otherwise. Also if outcome is true
     the firmware version will be printed
     in the output
  */
  bool done = false; // control the timeout for each send block

  unsigned char data[50]; //two bytes for address and cmd, and up to 48 to response
  data[0] = motor_x->address;
  data[1] = (unsigned char) GET_FIRMWARE;

  //unsigned int a = 0;
  done = usart_send_blocking_wtimeout(motor_x->usart, &data[0], TIMEOUT);
  if (!done) {
    return false;
  }

  done = usart_send_blocking_wtimeout(motor_x->usart, &data[1], TIMEOUT);
  if (!done) {
    return false;
  }


  for(int i=2; i<51; i++) {// max response size is 48 bytes
    done = usart_recv_blocking_wtimeout(motor_x->usart, &data[i], TIMEOUT);
    if (!done) {
      return false;
    }
if (((uint8_t)(data[i-1]) == 10) & ((uint8_t)(data[i]) == 0)) {//if this is 10, 0
      break;
    }
  }// write all response to data[i]

  unsigned char crc_rcv[2]; // receive the crc
  done = usart_recv_blocking_wtimeout(motor_x->usart, &crc_rcv[0], TIMEOUT);

  done = usart_recv_blocking_wtimeout(motor_x->usart, &crc_rcv[1], TIMEOUT);

  int response_size = strlen(&data)+1;//size of the dat rcvd + \n
  uint16_t crc_chk = crc16(data, response_size); // calculate local checksum

  if (crc_chk == (uint16_t)(crc_rcv[0] << 8 | crc_rcv[1])) { //check local crc with roboclaw
    strcpy(output, &data[2]);
    return true;
  }//if
  else {
    return false;
  }

}

bool read_main_battery(float *voltage, motor *motor_x) {
  /* Returns true if operation was succesful,
     False otherwise """.
     If the outcome is true, the battery
     level will be stored in output.
  */

  bool done = false; // track for timeout

  unsigned char data[4]; // two bytes for address and cmd, two for value

  data[0] = motor_x->address; //first to write is address
  data[1] = (unsigned char) GET_MAIN_BATT; // second is cmd

  done = usart_send_blocking_wtimeout(motor_x->usart, &data[0], TIMEOUT); // send first address and cmd
  if (!done) {
    return false;
  }

  done = usart_send_blocking_wtimeout(motor_x->usart, &data[1], TIMEOUT);
  if (!done) {
    return false;
  }

  for(int i=2; i<4; i++) {
    done = usart_recv_blocking_wtimeout(motor_x->usart, &data[i], TIMEOUT);
    if (!done) {
      return false;
    }
  } // first two bytes rcvd are the battery voltage

  unsigned char crc_rcv[2]; // received crc values from roboclaw
  done = usart_recv_blocking_wtimeout(motor_x->usart, &crc_rcv[0], TIMEOUT);
  if (!done) {
    return false;
  }

  done = usart_recv_blocking_wtimeout(motor_x->usart, &crc_rcv[1], TIMEOUT);
  if (!done) {
    return false;
  }


  uint16_t crc_chk = crc16(data, 4); // calculate local checksum

  if (crc_chk == (uint16_t)(crc_rcv[0] << 8 | crc_rcv[1])) { //check local crc with roboclaw crc
    *voltage = (float) ((uint16_t)(data[2] << 8 | data[3])) / 10.0;
    return true;
  }//if
  else {
    return false;
  }
}




