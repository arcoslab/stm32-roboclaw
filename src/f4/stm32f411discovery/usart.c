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

#include "usart.h"

void usart_init(usart_port usart_port_x) {
  /* RX is S1, TX is S2 on the roboclaw
     o
   * PA3 is RX, PA2 is TX on the stm
   */
  rcc_periph_clock_enable(usart_port_x.clken);
  gpio_mode_setup(usart_port_x.gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, usart_port_x.gpio_pin);
  gpio_set_af(usart_port_x.gpio_port, usart_port_x.gpio_af, usart_port_x.gpio_pin);

  rcc_periph_clock_enable(usart_port_x.clken_usart);

  /* Setup usart_port_x.usart parameters. */
  usart_set_baudrate(usart_port_x.usart, usart_port_x.baudrate);
  usart_set_databits(usart_port_x.usart, 8);
  usart_set_stopbits(usart_port_x.usart, USART_STOPBITS_1);
  usart_set_mode(usart_port_x.usart, USART_MODE_TX_RX);
  usart_set_parity(usart_port_x.usart, USART_PARITY_NONE);
  usart_set_flow_control(usart_port_x.usart, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(usart_port_x.usart);
}
