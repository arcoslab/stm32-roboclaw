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
#include "motor.h"

static motor motorfl; // static is necesary to mantain this values
static motor motorfr;

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

  encoder_update(&motorfl.encoder, timer_get_counter(motorfl.timer.peripheral), timer_get_flag(motorfl.timer.peripheral, TIM_SR_UIF));


}

void usart_config(void) {
  motorfl.port.usart = USART2;
  motorfl.port.baudrate = 115200;
  motorfl.port.gpio_port = GPIOA;
  motorfl.port.gpio_pin = GPIO2 | GPIO3;
  motorfl.port.gpio_af = GPIO_AF7;
  motorfl.port.clken = RCC_GPIOA;
  motorfl.port.clken_usart = RCC_USART2;
  usart_init(motorfl.port);
}

void encoder_config(void) {
  encoder_init(&motorfl.encoder);
}

void timers_config(void) {
  motorfl.timer.clken = RCC_GPIOB;
  motorfl.timer.clken_timer = RCC_TIM3;
  motorfl.timer.gpio_port = GPIOB;
  motorfl.timer.gpio_pin = GPIO4 | GPIO5;
  motorfl.timer.period = 65535;
  motorfl.timer.peripheral = TIM3;
  motorfl.timer.gpio_af = GPIO_AF2;
  motorfl.timer.ic1 = TIM_IC1;
  motorfl.timer.in1 = TIM_IC_IN_TI1;
  motorfl.timer.ic2 = TIM_IC2;
  motorfl.timer.in2 = TIM_IC_IN_TI2;
  motorfl.timer.mode = 0x3;
  tim_init(motorfl.timer);
}

void motors_config(void) {
  motorfl.address = 128;
  motorfl.code = 0;
  //motorfl.port = roboclaw_port; // add the usart port here
  //motorfl.encoder_timer = &motorfl.timer;
  //motorfl.encoder = motorfl_encoder;
  motorfr.address = 128;
  motorfr.code = 1;
  motorfr.timer = motorfl.timer;
  motorfr.port = motorfl.port;
  //motorfr.port = roboclaw_port;
}

void system_init(void) {
  /* This setup is using a STM32F411-disco, other versions may vary */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);

  usart_config(); // this config functions can be used with other ports and functions
  encoder_config();
  motors_config();
  timers_config();;

  leds_init();
  cdcacm_init();
  systick_init();
}

int main(void)
{
  system_init();

  int i;
  int c=0;
  int value = 0;
  bool dir = 0;

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
    fprintf(stdout, "POS: %lld | VALUE: %d | MOTRO: %d \n", motorfl.encoder.current_pos, value, motorfl.code);

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
        bool success = read_firmware(&output, &motorfl);
        if (success) {
          fprintf(stdout, "%s", &output);
        }// if success
        // read battery test
        float voltage;
        success = false;
        success = read_main_battery(&voltage, &motorfl);
        if (success) {
          fprintf(stdout, " %f\n", voltage);
        }
        if (c == 49){
          success = false;
          value += 1;
          success = drive_motor(&motorfl, value);
          success = drive_motor(&motorfr, value);
        }
        if (c == 50){
          success = false;
          value-=1;
          success = drive_motor(&motorfl, value);
          success = drive_motor(&motorfr, value);
        }

        if (c == 112){//WARNING dont change direction while moving fast
          success = false;
          dir = !dir;
          success = drive_motor_fwd_bwd(&motorfl, value, dir);
        }
        if (c == 119){//move forward with w

          success = false;
          value += 1;
          success = drive_motor_fwd_bwd(&motorfl, value, dir);
        }
        if (c == 115){//move backward with s
          success = false;
          value -= 1;
          success = drive_motor_fwd_bwd(&motorfl, value, dir);
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
