#include "timers.h"

void tim_init(void) {
  /* Timer 5 is available to use in PA0 and PA1 for other encoder
   * Timer 4 can be used mapping TIM4_CH3 in PB8 to TI1 and TIM4_CH2 in PB7 to TI2
   * Timer 2 can be used with CH2 to TI2 in PA1 and CH1 to TI1 in PA15
   * NOTE: CH1, CH2, and CH3 might be mapped to TI1
   * TI2 can only be mapped with CH2 245 @ rm0383.pdf
   *
   */
  
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5); // pin a6 & a7 to Alternate Function
  gpio_set_af(GPIOB, GPIO_AF2, GPIO4 | GPIO5); // ref pag 47 @ ds10314 datasheet table 9 AF02 column
  
  rcc_periph_clock_enable(RCC_TIM3);
  timer_set_period(TIM3, PERIOD); // set maximun period before auto reload
  timer_slave_set_mode(TIM3, 0x3); // encoder 
  timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1); // map ch1 to TI1 -> ref pag 243 @ rm0383.pdf
  timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2); // map ch2 to TI2 '' '' ''
  timer_enable_counter(TIM3);
  
}