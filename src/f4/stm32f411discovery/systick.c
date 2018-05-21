#include "systick.h"

void systick_init(void) {
  /* Proper configuration of systick is the following
   * Set Reload Register STK_LOAD reg
   * Clear current value STK_VAL reg
   * Program control and status reg: STK_CTRL
   * Control reg STK_CTRL contains countflag,
   * Clksource, tick int and enable
   * Reload value, should be N-1, because systick 
   * lasts one clock cycle in logic
   * EG you want subroutine to run each 100,000 cycles
   * then you choose Reload value to 99,999
   */
  systick_set_reload(SYS_TICK_AUTORELOAD); // clock rate is 84Mhz. repeated each 100us
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); 
  systick_counter_enable();
  systick_interrupt_enable();
}
