#ifndef EXTI_H
#define EXTI_H

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/* The following sets up the port to use and 
 * the exti related to that port.
 */

#define GPIO_PORT_x GPIOA
#define GPIO_MODE_x GPIO_MODE_INPUT
#define GPIO_PIN_x GPIO0
#define EXTI_x EXTI0
#define CLKEN_x RCC_GPIOA
#define IRQN_x NVIC_EXTI0_IRQ

void exti_init(void);

#endif
