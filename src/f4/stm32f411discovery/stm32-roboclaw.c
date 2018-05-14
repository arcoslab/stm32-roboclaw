/*
 * Copyright (C) 2013 ARCOS-Lab Universidad de Costa Rica
 * Author: Federico Ruiz Ugalde <memeruiz@gmail.com>
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
#include <stdbool.h>
#include <libopencm3/stm32/usart.h>
#include "roboclaw.h"

#define ADDRESS 128

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
  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  
  /* Finally enable the USART. */
  usart_enable(USART2);
}

void tim_init(void) {
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7); // pin a6 & a7 to Alternate Function
  gpio_set_af(GPIOA, GPIO_AF2, GPIO6 | GPIO7); // ref pag 47 @ ds10314 datasheet table 9 AF02 column
  
  rcc_periph_clock_enable(RCC_TIM3);
  timer_set_period(TIM3, 65535); // set maximun period before auto reload
  timer_slave_set_mode(TIM3, 0x3); // encoder 
  timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1); // map ch1 to TI1 -> ref pag 243 @ rm0383.pdf
  timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2); // map ch2 to TI2 '' '' ''
  timer_enable_counter(TIM3);
  
}

void system_init(void) {
  /* This setup is using a STM32F411-disco, other versions may vary */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
  leds_init();
  cdcacm_init();
  usart_init();
  tim_init();
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
    fprintf(stdout, "Motor: %d \n", motor_pos);
    //printf("Test\n");


    if ((poll(stdin) > 0)) {
      i=0;
      c=0;
      while (c!='\r') {
	c=getc(stdin);
	i++;
	putc(c, stdout);
        fprintf(stdout, " %u\n", c);
        // read firmware test
        char output;
	bool success = read_firmware(&output, ADDRESS);
        if (success) {
          //fprintf(stdout, "%s", &output);
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
          value += 10;
          success = move_motor(0, ADDRESS, value, dir);
        }
        if (c == 115){//move backward with s
          success = false;
          value -= 10;
          success = move_motor(0, ADDRESS, value, dir);
        }
        //move motor
        success = false;

      }
    }
  }
  
}
