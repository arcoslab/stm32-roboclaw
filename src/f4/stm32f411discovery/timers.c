#include "timers.h"
#include "timers.h"

void tim_init(timer timer_x) {
  /* Timer 5 is available to use in PA0 and PA1 for other encoder
   * Timer 4 can be used mapping TIM4_CH3 in PB8 to TI1 and TIM4_CH2 in PB7 to TI2
   * Timer 2 can be used with CH2 to TI2 in PA1 and CH1 to TI1 in PA15
   * NOTE: CH1, CH2, and CH3 might be mapped to TI1
   * TI2 can only be mapped with CH2 245 @ rm0383.pdf
   *
   */

  rcc_periph_clock_enable(timer_x.clken);
  gpio_mode_setup(timer_x.gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, timer_x.gpio_pin); // pin a6 & a7 to Alternate Function
  gpio_set_af(timer_x.gpio_port, timer_x.gpio_af, timer_x.gpio_pin); // ref pag 47 @ ds10314 datasheet table 9 AF02 column

  rcc_periph_clock_enable(timer_x.clken_timer);
  timer_set_period(timer_x.peripheral, timer_x.period); // set maximun period before auto reload
  timer_slave_set_mode(timer_x.peripheral, timer_x.mode); // encoder
  timer_ic_set_input(timer_x.peripheral, timer_x.ic1, timer_x.in1); // map ch1 to iI1 -> ref pag 243 @ rm0383.pdf
  timer_ic_set_input(timer_x.peripheral, timer_x.in2, timer_x.in2); // map ch2 to TI2 '' '' ''
  timer_enable_counter(timer_x.peripheral);

}
