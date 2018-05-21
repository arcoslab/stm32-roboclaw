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
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3-plus/newlib/syscall.h>
#include <libopencm3-plus/cdcacm_one_serial/cdcacm.h>
#include <libopencm3-plus/utils/misc.h>
#include <libopencm3-plus/stm32f4discovery/leds.h>
#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include "stm32-roboclaw.h"
#include "roboclaw.h"
#include "exti.h"
#include "timers.h"
#include "usart.h"
#include "systick.h"

volatile uint64_t counter=0;
volatile unsigned int ticks;
volatile int64_t pos_o;
volatile int64_t pos_f;
volatile float vel_o;
volatile float vel_f;
volatile float accel_o;
volatile float accel_f;
volatile unsigned int uif;
volatile float systick_time=TICKS_TIME;

void leds_init(void) {
  rcc_periph_clock_enable(RCC_GPIOE);
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12| GPIO13| GPIO14| GPIO15);
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

void exti0_isr(void) {
  /* Called on each hardware interrupt. Read exti_init
   * for more info
   */
  exti_reset_request(EXTI_x); // reset the interruption flag
  ticks++;

}

void sys_tick_handler(void) {
  /* This function will be called when systick fires, every 100us. 
   * Note that the slowest rate of the motor reaches
   * 5.53 ms per tick. If you count 30 ticks, the lag between new 
   * vel values will be 166 ms 
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
    if(abs(pos_f - pos_o) >= TICKS) { //ticks is defined in the header
      //amount of ticks found
      pos_o = pos_f;
      vel_f = 1.0 / (systick_time * (float)counter * (float) TICKS_PER_REV ); // in ticks / second
      vel_o = vel_f;
      counter = 0; // reset counter
    }
  }
}

void system_init(void) {
  /* This setup is using a STM32F411-disco, other versions may vary */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
  leds_init();
  cdcacm_init();
  usart_init();
  exti_init();
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
    fprintf(stdout, "Pos: %d | Vel: %f | Accel: %f | Counter: %u \n", pos_f, vel_f, accel_f, counter);
    //fprintf(stdout, "Motor: %d  || UIF: %d \n", motor_pos, flag);
    //fprintf(stdout, "Test | counter value: %d \n", counter);
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
        //bool success = false;
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
