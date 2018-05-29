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
#include <libopencm3/stm32/usart.h>
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
#include "encoder.h"

#define FILTER_SIZE 2

volatile uint64_t counter;
volatile int16_t past_counter;
volatile int64_t past_pos;
volatile int64_t current_pos;
volatile float past_vel;
volatile float current_vel;
volatile float current_accel;
volatile unsigned int uif;

void leds_init(void) {
  rcc_periph_clock_enable(RCC_GPIOE);
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12| GPIO13| GPIO14| GPIO15);
}

void exti0_isr(void) {
  /* Called on each hardware interrupt. Read exti_init
   * for more info
   */
  exti_reset_request(EXTI_x); // reset the interruption flag
  //ticks++;

}

void sys_tick_handler(void) {
  /* This function will be called when systick fires, every 100us.
   * Note that the slowest rate of the motor reaches
   * 5.53 ms per tick. If you count 30 ticks, the lag between new
   * vel values will be 166 ms
   */

  counter++; // this is counting how many systick handlers are called
  current_pos = timer_get_counter(TIM3);

  if (counter > 5000) {
    // when more than 50ms has passed
    current_vel = 0;
  }

  if (past_pos == current_pos) {
    return;
  }

  uif = timer_get_flag(TIM3, TIM_SR_UIF);
  if(uif == 1){

  }

  current_vel = (float) (past_pos - current_pos)/(((float) counter) * TICKS_TIME);

  past_vel = current_vel;
  past_pos = current_pos;
  counter = 0;

  //uif = timer_get_flag(TIM3, TIM_SR_UIF); // get the update flag from the counter

}

void system_init(void) {
  /* This setup is using a STM32F411-disco, other versions may vary */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);
  leds_init();
  cdcacm_init();
  usart_port roboclaw_port;
  roboclaw_port.usart = USART2;
  roboclaw_port.baudrate = 115200;
  roboclaw_port.gpio_port = GPIOA;
  roboclaw_port.gpio_pin = GPIO2 | GPIO3;
  roboclaw_port.gpio_af = GPIO_AF7;
  roboclaw_port.clken = RCC_GPIOA;
  roboclaw_port.clken_usart = RCC_USART2;
  usart_init(roboclaw_port);
  //exti_init();
  tim_init();
  systick_init();
  encoder motor_fl_encoder;
  encoder_init(motor_fl_encoder);
}

int main(void)
{
  static motor motorfl_motor; // static is necesary to mantain this values
  motorfl_motor.usart = USART2;
  motorfl_motor.address = 128;
  motorfl_motor.motor = 0;


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
    /* If you want to test this code with a stm32f4
     *you should connect the tx and rx port
     *to the usart 2 for a stm32f4-disco11. This would
     *be PA2 and PA3 pins.*/

    //fprintf(stdout, "Pos: %d | Vel: %f | Accel: %f | Counter: %u \n", current_pos, current_vel, current_accel, counter);
    //fprintf(stdout, "Past Pos: %ld | Current Pos: %ld | TICKS TIME: %f | Counter: %ld | Current Vel: %f \n", (long)past_pos, (long)current_pos, TICKS_TIME, (long)counter, current_vel);
    //fprintf(stdout, "test\n");
    fprintf(stdout, "POS: %lld | VALUE: %d | MOTRO: %d \n", current_pos, value, motorfl_motor.motor);

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
        if (c == 49){
          success = false;
          value += 1;
          success = drive_motor(motorfl_motor, value);
        }
        if (c == 50){
          success = false;
          value-=1;
          success = drive_motor(motorfl_motor, value);
        }

        if (c == 112){//WARNING dont change direction while moving fast
          success = false;
          dir = !dir;
          success = drive_motor_fwd_bwd(0, ADDRESS, value, dir);
        }
        if (c == 119){//move forward with w

          success = false;
          value += 1;
          success = drive_motor_fwd_bwd(0, ADDRESS, value, dir);
        }
        if (c == 115){//move backward with s
          success = false;
          value -= 1;
          success = drive_motor_fwd_bwd(0, ADDRESS, value, dir);
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
