#include "exti.h"

void exti_init(void) {
  /*
   * Function to enable exti, for hardware interruptions
   * based on each click from the encoder in the pin.
   * In other words, this enables that the code in extix_isr
   * runs the code in exti0 receives a signal, which is configured
   * to hear the pin in GPIO_PORT_x, which is the user button
   * Look for NVIC in the reference manual. 
   * The counter is only a pointer to a variable
   * That contains the actual counter info
   */

  /* This line is commented because this clock was enabled above*/
  rcc_periph_clock_enable(CLKEN_x);
  
  gpio_mode_setup(GPIO_PORT_x, GPIO_MODE_x, GPIO_PUPD_NONE, GPIO_PIN_x);

  rcc_periph_clock_enable(RCC_SYSCFG); // enable SYSCFG clock
  nvic_enable_irq(IRQN_x); // enable exti1 interrupt

  exti_select_source(EXTI_x, GPIO_PORT_x); //select the source for exti1
  exti_set_trigger(EXTI_x, EXTI_TRIGGER_RISING); //set the trigger to rising
  exti_enable_request(EXTI_x); //enable the requests 
}

