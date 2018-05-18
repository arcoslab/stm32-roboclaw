/*
 * Copyright (C) 2018 ARCOS-Lab Universidad de Costa Rica
 * Author: Federico Ruiz Ugalde <memeruiz@gmail.com> 
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3-plus/newlib/syscall.h>
#include "stm32-roboclaw.h"
#include <libopencm3-plus/cdcacm_one_serial/cdcacm.h>
#include <stdio.h>
#include <libopencm3-plus/utils/misc.h>
#include <libopencm3-plus/stm32f4discovery/leds.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <libopencm3/stm32/usart.h>
#include "roboclaw.h"
#include <libopencm3/cm3/systick.h>

volatile unsigned int counter;
volatile unsigned int pos_o;
volatile unsigned int pos_f;
volatile float vel_o;
volatile float vel_f;
volatile float accel_o;
volatile float accel_f;
volatile unsigned int uif;
volatile float systick_time;

void leds_init(void) {
  rcc_periph_clock_enable(RCC_GPIOE);
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12| GPIO13| GPIO14| GPIO15);
}

void usart_init(void) {
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2| GPIO3);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
  
  rcc_periph_clock_enable(RCC_USART2);
  
  /* Setup USART2 parameters. */
  usart_set_baudrate(USART2, BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  
  /* Finally enable the USART. */
  usart_enable(USART2);
}

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
  systick_time = TICKS_T; // or the same as 100us, 0.1ms
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); 
  systick_counter_enable();
  systick_interrupt_enable();
}

void encoder_init(void) {
  /* This only resets the variables used to
   * store the encoder pos, vel and accel during
   the init
  */
  counter = 0;
  pos_o = 0;
  pos_f = 0;
  vel_o = 0.0;
  vel_f = 0.0;
  accel_o = 0.0;
  accel_f = 0.0;
  uif = 0;
}

void sys_tick_handler(void) {
  /* This function will be called when systick fires, every 100us. 
     Note that the slowest rate of the motor reaches
     5.53 ms per tick. If you count 30 ticks, the lag between new 
     vel values will be 166 ms 
   */
    
  counter++; // this is counting how many systick handlers are called
  uif = timer_get_flag(TIM3, TIM_SR_UIF); // get the update flag from the counter

  //pos_f = timer_get_counter(TIM3);
  if(uif==1){
    // the difference is unknown
    counter=0;
    pos_o = timer_get_counter(TIM3);
    pos_f = pos_o; // reset pos final
    timer_clear_flag(TIM3, TIM_SR_UIF); // clear the flag
  }
  else{
    // theres no call to autoreload
    pos_f = timer_get_counter(TIM3);
    if(abs(pos_f - pos_o) == TICKS) { //ticks is defined in the header
      //amount of ticks found
      pos_o = pos_f;
      vel_f = (float)TICKS / (systick_time * (float)counter * (float) TICKS_PER_REV ); // in ticks / second
      vel_o = vel_f;
      counter = 0; // reset counter
    }
    if(counter > COUNTER_RELOAD) {
      counter = 0;
      vel_f = 0.0;
    }//counter reload
  }
}

void system_init(void) {
  /* This setup is using a STM32F411-disco, other versions may vary */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
  leds_init();
  cdcacm_init();
  usart_init();
  tim_init();
  systick_init();
  encoder_init();
}

int main(void)
{
  system_init();
  int i;
  int c=0;
  int value = 0;
  bool motor = 0;
  bool dir = 0;
  int motor_pos = 0;  
  int flag = 0;
  
  setvbuf(stdin,NULL,_IONBF,0); // Sets stdin in unbuffered mode (normal for usart com)
  setvbuf(stdout,NULL,_IONBF,0); // Sets stdin in unbuffered mode (normal for usart com)
  
  while (poll(stdin) > 0) {
    printf("Cleaning stdin\n");
    getc(stdin);
  }

  while (1){
    /* If you want to test this code with an lying 
     you should connect the tx and rx port
    to the usart 2 for a stm32f4-disco11. This would
    be PA2 and PA3 pins.*/

    motor_pos = timer_get_counter(TIM3);
    flag = timer_get_flag(TIM3, TIM_SR_UIF); // get the udpate interrupt flag
    fprintf(stdout, "Pos: %d | Vel: %f | Accel: %f\n", pos_f, vel_f, accel_f);
    //fprintf(stdout, "Motor: %d  || UIF: %d \n", motor_pos, flag);
    //printf("Test\n");


    if ((poll(stdin) > 0)) {
      i=0;
      c=0;
      while (c!='\r') {
	c=getc(stdin);
	i++;
	//putc(c, stdout);
        //fprintf(stdout, " %u\n", c);
        // read firmware test
        char output;
	bool success = read_firmware(&output, ADDRESS);
        if (success) {
          fprintf(stdout, "%s", &output);
        }// if success
        // read battery test
        float voltage;
        success = false;
	success = read_main_battery(&voltage, ADDRESS);
        if (success) {
          fprintf(stdout, " %f\n", voltage);
        }
        if (c == 112){//WARNING dont change direction while moving fast
          success = false;
          dir = !dir;
          success = move_motor(0, ADDRESS, value, dir);
        }
        if (c == 119){//move forward with w
          success = false;
          value += 1;
          success = move_motor(0, ADDRESS, value, dir);
        }
        if (c == 115){//move backward with s
          success = false;
          value -= 1;
          success = move_motor(0, ADDRESS, value, dir);
        }
        if(success){
          fprintf(stdout, "ACK\n");
        }
        else{
          fprintf(stdout, "FAILED\n");
        }
        //move motor
        success = false;

      }
    }
  }
  
}
