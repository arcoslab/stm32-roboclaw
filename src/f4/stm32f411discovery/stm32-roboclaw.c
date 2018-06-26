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
#include "filter.h"

static motor *motorrl; // static is necesary to mantain this values
static motor *motorrr;
static motor *motorfr;
static motor *motorfl;
static motor *test;

void sys_tick_handler(void) {
  /* This function will be called when systick fires, every 100us.
     It's purpose is to update the encoder pos, vel, and accel,
     and to generate the pid control signal
   */

  // pid control for motorfl
  encoder_update(test);
  cmd_vel(test);

}

void usart_config(void) {
  // configuration for usart6, roboclaw of front axel
  motorfl->port = malloc(sizeof(usart_port));
  motorfl->port->usart = USART6;
  motorfl->port->baudrate = 115200;
  motorfl->port->gpio_port = GPIOC;
  motorfl->port->gpio_pin = GPIO6 | GPIO7;
  motorfl->port->gpio_af = GPIO_AF8;
  motorfl->port->clken = RCC_GPIOC;
  motorfl->port->clken_usart = RCC_USART6;
  usart_init(motorfl->port);
  motorfl->usart = motorfl->port->usart;
  free(motorfl->port); // free memory

  // pass the configuration for the left front motor
  motorfr->usart = motorfl->usart;

  // configuration for usart 2, rear axel
  motorrr->port = malloc(sizeof(usart_port));
  motorrr->port->usart = USART2;
  motorrr->port->baudrate = 115200;
  motorrr->port->gpio_port = GPIOA;
  motorrr->port->gpio_pin = GPIO2 | GPIO3;
  motorrr->port->gpio_af = GPIO_AF7;
  motorrr->port->clken = RCC_GPIOA;
  motorrr->port->clken_usart = RCC_USART2;
  usart_init(motorrr->port);
  motorrr->usart = motorrr->port->usart;
  free(motorrr->port); // free memory

  // pass the configuration for the right rear motor
  motorrl->usart = motorrr->usart;
}

void encoder_config(void) {

  // front right motor encoder
  motorfl->encoder = malloc(sizeof(encoder));
  motorfl->encoder->autoreload = 10000;
  motorfl->encoder->filter.max_size = 15;
  filter_init(&motorfl->encoder->filter);
  encoder_init(motorfl->encoder);

  // front left motor encoder
  motorfr->encoder = malloc(sizeof(encoder));
  motorfr->encoder->autoreload = 10000;
  motorfr->encoder->filter.max_size = 15;
  filter_init(&motorfr->encoder->filter);
  encoder_init(motorfr->encoder);

  // rear right encoder init
  motorrr->encoder = malloc(sizeof(encoder));
  motorrr->encoder->autoreload = 10000;
  motorrr->encoder->filter.max_size = 15;
  filter_init(&motorrr->encoder->filter);
  encoder_init(motorrr->encoder);

  // rear left encoder init
  motorrl->encoder = malloc(sizeof(encoder));
  motorrl->encoder->autoreload = 10000;
  motorrl->encoder->filter.max_size = 15;
  filter_init(&motorrl->encoder->filter);
  encoder_init(motorrl->encoder);

}

void timers_config(void) {
  // front right motor timer
  motorfl->timer = malloc(sizeof(timer));
  motorfl->timer->clken_ch1 = RCC_GPIOD;
  motorfl->timer->clken_ch2 = RCC_GPIOD;
  motorfl->timer->clken_timer = RCC_TIM4;
  motorfl->timer->gpio_port_ch1 = GPIOD;
  motorfl->timer->gpio_port_ch2 = GPIOD;
  motorfl->timer->gpio_pin_ch1 = GPIO12;
  motorfl->timer->gpio_pin_ch2 = GPIO13;
  motorfl->timer->period = 65535;
  motorfl->timer->peripheral = TIM4;
  motorfl->timer->gpio_af = GPIO_AF2;
  motorfl->timer->ic1 = TIM_IC1;
  motorfl->timer->in1 = TIM_IC_IN_TI1;
  motorfl->timer->ic2 = TIM_IC2;
  motorfl->timer->in2 = TIM_IC_IN_TI2;
  motorfl->timer->mode = 0x3;
  tim_init(motorfl->timer);
  motorfl->peripheral = motorfl->timer->peripheral; //save peripheral name
  motorfl->period = motorfl->timer->period;
  free(motorfl->timer);

  // front left motor timer
  motorfr->timer = malloc(sizeof(timer));
  motorfr->timer->clken_ch1 = RCC_GPIOA;
  motorfr->timer->clken_ch2 = RCC_GPIOB;
  motorfr->timer->clken_timer = RCC_TIM2;
  motorfr->timer->gpio_port_ch1 = GPIOA;
  motorfr->timer->gpio_port_ch2 = GPIOB;
  motorfr->timer->gpio_pin_ch1 = GPIO15;
  motorfr->timer->gpio_pin_ch2 = GPIO3;
  motorfr->timer->period = 65535;
  motorfr->timer->peripheral = TIM2;
  motorfr->timer->gpio_af = GPIO_AF1;
  motorfr->timer->ic1 = TIM_IC1;
  motorfr->timer->in1 = TIM_IC_IN_TI1;
  motorfr->timer->ic2 = TIM_IC2;
  motorfr->timer->in2 = TIM_IC_IN_TI2;
  motorfr->timer->mode = 0x3;
  tim_init(motorfr->timer);
  motorfr->peripheral = motorfr->timer->peripheral;
  motorfr->period = motorfr->timer->period;
  free(motorfr->timer);

  // rear right motor timer
  motorrr->timer = malloc(sizeof(timer));
  motorrr->timer->clken_ch1 = RCC_GPIOB;
  motorrr->timer->clken_ch2 = RCC_GPIOB;
  motorrr->timer->clken_timer = RCC_TIM3;
  motorrr->timer->gpio_port_ch1 = GPIOB;
  motorrr->timer->gpio_port_ch2 = GPIOB;
  motorrr->timer->gpio_pin_ch1 = GPIO4;
  motorrr->timer->gpio_pin_ch2 = GPIO5;
  motorrr->timer->period = 65535;
  motorrr->timer->peripheral = TIM3;
  motorrr->timer->gpio_af = GPIO_AF2;
  motorrr->timer->ic1 = TIM_IC1;
  motorrr->timer->in1 = TIM_IC_IN_TI1;
  motorrr->timer->ic2 = TIM_IC2;
  motorrr->timer->in2 = TIM_IC_IN_TI2;
  motorrr->timer->mode = 0x3;
  tim_init(motorrr->timer);
  motorrr->peripheral = motorrr->timer->peripheral;
  motorrr->period = motorrr->timer->period;
  free(motorrr->timer);

  // rear left motor timer
  motorrl->timer = malloc(sizeof(timer));
  motorrl->timer->clken_ch1 = RCC_GPIOA;
  motorrl->timer->clken_ch2 = RCC_GPIOA;
  motorrl->timer->clken_timer = RCC_TIM5;
  motorrl->timer->gpio_port_ch1 = GPIOA;
  motorrl->timer->gpio_port_ch2 = GPIOA;
  motorrl->timer->gpio_pin_ch1 = GPIO0;
  motorrl->timer->gpio_pin_ch2 = GPIO1;
  motorrl->timer->period = 65535;
  motorrl->timer->peripheral = TIM5;
  motorrl->timer->gpio_af = GPIO_AF2;
  motorrl->timer->ic1 = TIM_IC1;
  motorrl->timer->in1 = TIM_IC_IN_TI1;
  motorrl->timer->ic2 = TIM_IC2;
  motorrl->timer->in2 = TIM_IC_IN_TI2;
  motorrl->timer->mode = 0x3;
  tim_init(motorrl->timer);
  motorrl->peripheral = motorrl->timer->peripheral;
  motorrl->period = motorrl->timer->period;
  free(motorrl->timer);

}

void pid_config(void) {
  // new pid using malloc
  motorfl->pid = malloc(sizeof(pid));
  motorfl->pid->kp = 0.02; // 0->6 tune for good reference response-> Big overshoot
  motorfl->pid->ki = 0.35;
  motorfl->pid->kd = 0.001;
  motorfl->pid->reference = 0.0;
  motorfl->pid->current_error = 0;
  motorfl->pid->past_error = 0;
  motorfl->pid->error_sum = 0;
  motorfl->pid->current_action = 0;
  motorfl->pid->past_action = 0;
  motorfl->pid->action_limit = 127;
  motorfl->pid->wait_time = 0;
  motorfl->pid->response_time = 0.001; // 1 ms

  // use fr pid config
  motorfr->pid = motorfl->pid;
  motorrr->pid = motorfl->pid;
  motorrl->pid = motorfl->pid;
}

void motors_config(void) {
  // motor front rear configuration
  motorfl->address = 128;
  motorfl->code = 0;
  motorfl->clicks_per_rev = 3408;
  motorfl->wheel_radius = 0.05;

  // motor front left configuration
  motorfr->address = 128;
  motorfr->code = 1;
  motorfr->clicks_per_rev = 3408;
  motorfr->wheel_radius = 0.05;

  // motor rear right configuration
  motorrr->address = 128;
  motorrr->code = 1;
  motorrr->clicks_per_rev = 3408;
  motorrr->wheel_radius = 0.05;

  // motor rear left configuration
  motorrl->address = 128;
  motorrl->code = 0;
  motorrl->clicks_per_rev = 3408;
  motorrl->wheel_radius = 0.05; // in meters

}

void system_init(void) {
  /* This setup is using a STM32F411-disco, other versions may vary */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);

  // init the motor structs
  motorfl = malloc(sizeof(motor));
  motorfr = malloc(sizeof(motor));
  motorrr = malloc(sizeof(motor));
  motorrl = malloc(sizeof(motor));

  usart_config(); // this config functions can be used with other ports and functions
  timers_config();
  pid_config();
  motors_config();

  cdcacm_init();
  encoder_config(); // this comsumes a lot of memory because of filters
  systick_init();

  test = motorrl; // test front motor
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
    //fprintf(stdout, "Past Pos: %lld | Past timer Pos: %ld | TICKS TIME: %f | Counter: %lld | Current Vel: %f \n", motorfr->encoder->past_pos, test->encoder->current_timer_counter, TICKS_TIME, test->encoder->systick_counter, test->encoder->current_vel);
    // fprintf(stdout, "test\n");
    fprintf(stdout, "POS 1: %lld | POS 2: %lld | VALUE: %d | MOTRO: %d \n", test->encoder->current_pos, test->encoder->current_pos, value, test->code);
    //fprintf(stdout,
    //        "Act: %f | AvgVel: %f | Ref: %f | Kp: %f | Ki: %f | Kd: %f | E: %f | Esum: %f | Change %f \n",
     //       test->pid->current_action,
       //     test->encoder->current_vel/(float) test->clicks_per_rev,
           // test->pid->reference,
         //   test->pid->kp,
           // test->pid->ki,
           // test->pid->kd,
           // test->pid->current_error,
           // test->pid->error_sum,
           // (test->pid->current_error - test->pid->past_error)) ;
    //fprintf(stdout, "Current Vel: %f | Avg Vel: %f | Pos: %lld | Counter: %ld\n", test->encoder->current_vel, test->encoder->avg_ticks, test->encoder->current_pos, test->encoder->used_timer_counter);

    if ((poll(stdin) > 0)) {
      test->pid->updating = true;
      i=0;
      c=0;
      while (c!='\r') {
        c=getc(stdin);

        i++;
        putc(c, stdout);
        fprintf(stdout, "%f", test->pid->reference);
        //fprintf(stdout, " %u\n", c);
        // read firmware test
        char output;
        //bool success = false;
        bool success = read_firmware(&output, test);
        if (success) {
          fprintf(stdout, "%s", &output);
        }// if success
        // read battery test
        float voltage;
        success = false;
        success = read_main_battery(&voltage, test);
        if (success) {
          fprintf(stdout, " %f\n", voltage);
        }
        if (c == 122) { // z will raise the reference
          //success = false;
          test->pid->reference += 0.1;
        }
        if (c == 120) { // x will lower the reference
          test->pid->reference -= 0.1;
        }
        if (c == 110) { // n will increase kp by 0.1
          test->pid->ki += 0.00001;
        }

        if (c == 109) { // m will decrease kp by 0.1
          test->pid->ki -= 0.00001;
        }

        if (c == 49){
          success = false;
          value += 1;
          success = drive_motor(test, value);
          //success = drive_motor(&test, value);
        }
        if (c == 50){
          success = false;
          value-=1;
          success = drive_motor(test, value);
          //success = drive_motor(&test, value);
        }

        if (c == 112){//WARNING dont change direction while moving fast
          success = false;
          dir = !dir;
          success = drive_motor_fwd_bwd(test, value, dir);
        }
        if (c == 119){//move forward with w

          success = false;
          value += 1;
          success = drive_motor_fwd_bwd(test, value, dir);
        }
        if (c == 115){//move backward with s
          success = false;
          value -= 1;
          success = drive_motor_fwd_bwd(test, value, dir);
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
    test->pid->updating = false;
    }
  }

}
