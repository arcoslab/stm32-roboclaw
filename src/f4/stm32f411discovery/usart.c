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
  usart_set_baudrate(usart_port_x.usart, usart_port.baudrate);
  usart_set_databits(usart_port_x.usart, 8);
  usart_set_stopbits(usart_port_x.usart, USART_STOPBITS_1);
  usart_set_mode(usart_port_x.usart, USART_MODE_TX_RX);
  usart_set_parity(usart_port_x.usart, USART_PARITY_NONE);
  usart_set_flow_control(usart_port_x.usart, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(usart_port_x.usart);
}
