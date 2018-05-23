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

#ifndef USART_H
#define USART_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#define USART_x USART2
#define GPIO_PORT_x GPIOA
#define GPIO_PIN_x GPIO2 | GPIO3
#define GPIO_AF_x GPIO_AF7
#define CLKEN_x RCC_GPIOA
#define CLKEN_USART_x RCC_USART2
#define BAUDRATE 115200

typedef struct usart_port{
  uint32_t usart;
  uint32_t baudrate;
  uint32_t gpio_port;
  uint16_t gpio_pin;
  uint8_t gpio_af;
  enum rcc_periph_clken clken;
  enum rcc_periph_clken clken_usart;
} usart_port;

void usart_init(usart_port usart_port_x);

#endif
