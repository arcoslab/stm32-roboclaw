#include "usart.h"

void usart_init(void) {
  /* RX is S1, TX is S2 on the roboclaw
     o
   * PA3 is RX, PA2 is TX on the stm
   */
  rcc_periph_clock_enable(CLKEN_x);
  gpio_mode_setup(GPIO_PORT_x, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_x);
  gpio_set_af(GPIO_PORT_x, GPIO_AF_x, GPIO_PIN_x);

  rcc_periph_clock_enable(CLKEN_USART_x);

  /* Setup USART_x parameters. */
  usart_set_baudrate(USART_x, BAUDRATE);
  usart_set_databits(USART_x, 8);
  usart_set_stopbits(USART_x, USART_STOPBITS_1);
  usart_set_mode(USART_x, USART_MODE_TX_RX);
  usart_set_parity(USART_x, USART_PARITY_NONE);
  usart_set_flow_control(USART_x, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART_x);
}
