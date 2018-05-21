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

void usart_init(void);

#endif
