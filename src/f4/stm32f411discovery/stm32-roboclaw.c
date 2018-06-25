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
static motor *motorfl;
static motor *motorfr;

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

  // pid control for motorrl
  //encoder_update(&motorrl);
  //cmd_vel(&motorrl);

  // pid control for rear right motor
 // encoder_update(motorrr);

}

void usart_config(void) {
  // configuration for usart1, front axel
  //
  motorfr->port = malloc(sizeof(usart_port));

  motorfr->port->usart = USART6;
  motorfr->port->baudrate = 115200;
  motorfr->port->gpio_port = GPIOC;
  motorfr->port->gpio_pin = GPIO6 | GPIO7;
  motorfr->port->gpio_af = GPIO_AF8;
  motorfr->port->clken = RCC_GPIOC;
  motorfr->port->clken_usart = RCC_USART6;
  usart_init(motorfr->port);

  // pass the configuration for the left front motor
  //motorfl->port = motorfr->port;

  // configuration for usart 2, rear axel
  //motorrl->port->usart = USART2;
  //motorrl->port->baudrate = 115200;
  //motorrl->port->gpio_port = GPIOA;
  //motorrl->port->gpio_pin = GPIO2 | GPIO3;
  //motorrl->port->gpio_af = GPIO_AF7;
  //motorrl->port->clken = RCC_GPIOA;
  //motorrl->port->clken_usart = RCC_USART2;
  //usart_init(motorrl->port);

  // pass the configuration for the right rear motor
  //motorrr->port = motorrl->port;
}

void encoder_config(void) {
  motorfr->encoder = malloc(sizeof(encoder));

  // front right motor encoder
  motorfr->encoder->autoreload = 10000;
  motorfr->encoder->filter.max_size = 70;
  //filter_init(&motorfr->encoder->filter);
  encoder_init(motorfr->encoder);

   // front left motor encoder
  //motorfl->encoder->autoreload = 10000;
  //motorfl->encoder->filter.max_size = 70;
  //filter_init(&motorfl->encoder->filter);
  //encoder_init(&motorfl->encoder);

  // rear left encoder init
  //motorrl->encoder->autoreload = 10000;
  //motorrl->encoder->filter.max_size = 70;
  //filter_init(&motorrl->encoder->filter);
  //encoder_init(&motorrl->encoder);

  // rear right encoder init
  //motorrr->encoder->autoreload = 10000;
  //motorrr->encoder->filter.max_size = 70;
  //filter_init(&motorrr->encoder->filter);
  //encoder_init(motorrr->encoder);
  //filter_init(&motorrr->encoder->filter);

}

void timers_config(void) {
  // front right motor timer
  motorfr->timer->clken_ch1 = RCC_GPIOD;
  motorfr->timer->clken_ch2 = RCC_GPIOD;
  motorfr->timer->clken_timer = RCC_TIM4;
  motorfr->timer->gpio_port_ch1 = GPIOD;
  motorfr->timer->gpio_port_ch2 = GPIOD;
  motorfr->timer->gpio_pin_ch1 = GPIO12;
  motorfr->timer->gpio_pin_ch2 = GPIO13;
  motorfr->timer->period = 65535;
  motorfr->timer->peripheral = TIM4;
  motorfr->timer->gpio_af = GPIO_AF2;
  motorfr->timer->ic1 = TIM_IC1;
  motorfr->timer->in1 = TIM_IC_IN_TI1;
  motorfr->timer->ic2 = TIM_IC2;
  motorfr->timer->in2 = TIM_IC_IN_TI2;
  motorfr->timer->mode = 0x3;
  tim_init(motorfr->timer);

  motorfr->timer->peripheral = TIM4;

  // front left motor timer
  motorfl->timer->clken_ch1 = RCC_GPIOA;
  motorfl->timer->clken_ch2 = RCC_GPIOB;
  motorfl->timer->clken_timer = RCC_TIM2;
  motorfl->timer->gpio_port_ch1 = GPIOA;
  motorfl->timer->gpio_port_ch2 = GPIOB;
  motorfl->timer->gpio_pin_ch1 = GPIO15;
  motorfl->timer->gpio_pin_ch2 = GPIO3;
  motorfl->timer->period = 65535;
  motorfl->timer->peripheral = TIM2;
  motorfl->timer->gpio_af = GPIO_AF1;
  motorfl->timer->ic1 = TIM_IC1;
  motorfl->timer->in1 = TIM_IC_IN_TI1;
  motorfl->timer->ic2 = TIM_IC2;
  motorfl->timer->in2 = TIM_IC_IN_TI2;
  motorfl->timer->mode = 0x3;
  tim_init(motorfl->timer);

  // rear right motor timer
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

  // rear left motor timer
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
}

void pid_config(void) {
//  motorrl->pid->kp = 0.02; // 0->6 tune for good reference response-> Big overshoot
//  motorrl->pid->ki = 0.35;
//  motorrl->pid->kd = 0.001;
//  motorrl->pid->reference = 0.0;
//  motorrl->pid->current_error = 0;
//  motorrl->pid->past_error = 0;
//  motorrl->pid->error_sum = 0;
//  motorrl->pid->current_action = 0;
//  motorrl->pid->past_action = 0;
//  motorrl->pid->action_limit = 127;
//  motorrl->pid->wait_time = 0;
//  motorrl->pid->response_time = 0.001; // 1 ms

  motorfr->pid = malloc(sizeof(pid));

  motorfr->pid->reference = 0.0;

//  motorrr->pid = motorrl->pid;
//  motorfr->pid = motorrl->pid;
//  motorfl->pid = motorrl->pid;
}

void motors_config(void) {
  // motor rear left configuration
  //motorrl->address = 128;
  //motorrl->code = 0;
  //motorrl->clicks_per_rev = 3408;
  //motorrl->wheel_radius = 0.05; // in meters

  // motor rear right configuration
  //motorrr->address = 128;
  //motorrr->code = 1;
  //motorrr->clicks_per_rev = 3408;
  //motorrr->wheel_radius = 0.05;

  // motor front rear configuration
  motorfr->address = 128;
  motorfr->code = 0;
  motorfr->clicks_per_rev = 3408;
  motorfr->wheel_radius = 0.05;

  // motor front left configuration
  //motorfl->address = 128;
  //motorfl->code = 1;
  //motorfl->clicks_per_rev = 3408;
  //motorfl->wheel_radius = 0.05;

}

void system_init(void) {
  /* This setup is using a STM32F411-disco, other versions may vary */
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);

  // init the motor structs
  motorfr = malloc(sizeof(motor));
  //motorfl = malloc(sizeof(motor));
  //motorrr = malloc(sizeof(motor));
  //motorrl = malloc(sizeof(motor));

  usart_config(); // this config functions can be used with other ports and functions
  //encoder_config();
  //timers_config();
  //pid_config();
  motors_config();

  //leds_init();
  cdcacm_init();
  //systick_init();
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


    fprintf(stdout, "hi");
    //fprintf(stdout, "Pos: %d | Vel: %f | Accel: %f | Counter: %u \n", current_pos, current_vel, current_accel, counter);
    //fprintf(stdout, "Past Pos: %lld | Past timer Pos: %ld | TICKS TIME: %f | Counter: %lld | Current Vel: %f \n", motorfr->encoder->past_pos, motorfr->encoder->current_timer_counter, TICKS_TIME, motorfr->encoder->systick_counter, motorfr->encoder->current_vel);
    // fprintf(stdout, "test\n");
    //fprintf(stdout, "POS 1: %lld | POS 2: %lld | VALUE: %d | MOTRO: %d \n", motorfr->encoder->current_pos, motorfr->encoder->current_pos, value, motorfr->code);
    //fprintf(stdout,
    //        "Act: %f | AvgVel: %f | Ref: %f | Kp: %f | Ki: %f | Kd: %f | E: %f | Esum: %f | Change %f \n",
     //       motorfr->pid->current_action,
       //     motorfr->encoder->current_vel/(float) motorfr->clicks_per_rev,
           // motorfr->pid->reference,
         //   motorfr->pid->kp,
           // motorfr->pid->ki,
           // motorfr->pid->kd,
           // motorfr->pid->current_error,
           // motorfr->pid->error_sum,
           // (motorfr->pid->current_error - motorfr->pid->past_error)) ;
    //fprintf(stdout, "Current Vel: %f | Avg Vel: %f | Pos: %lld | Counter: %ld\n", motorfr->encoder->current_vel, motorfr->encoder->avg_ticks, motorfr->encoder->current_pos, motorfr->encoder->used_timer_counter);

    if ((poll(stdin) > 0)) {
      motorfr->pid->updating = true;
      i=0;
      c=0;
      while (c!='\r') {
        c=getc(stdin);

        i++;
        putc(c, stdout);
        fprintf(stdout, "%f", motorfr->pid->reference);
        //fprintf(stdout, " %u\n", c);
        // read firmware test
        char output;
        //bool success = false;
        bool success = read_firmware(&output, motorfr);
        if (success) {
          fprintf(stdout, "%s", &output);
        }// if success
        // read battery test
        float voltage;
        success = false;
        success = read_main_battery(&voltage, motorfr);
        if (success) {
          fprintf(stdout, " %f\n", voltage);
        }
        if (c == 122) { // z will raise the reference
          //success = false;
          motorfr->pid->reference += 0.1;
        }
        if (c == 120) { // x will lower the reference
          motorfr->pid->reference -= 0.1;
        }
        if (c == 110) { // n will increase kp by 0.1
          motorfr->pid->ki += 0.00001;
        }

        if (c == 109) { // m will decrease kp by 0.1
          motorfr->pid->ki -= 0.00001;
        }

        if (c == 49){
          success = false;
          value += 1;
          success = drive_motor(motorfr, value);
          //success = drive_motor(&motorfr, value);
        }
        if (c == 50){
          success = false;
          value-=1;
          success = drive_motor(motorfr, value);
          //success = drive_motor(&motorfr, value);
        }

        if (c == 112){//WARNING dont change direction while moving fast
          success = false;
          dir = !dir;
          success = drive_motor_fwd_bwd(motorfr, value, dir);
        }
        if (c == 119){//move forward with w

          success = false;
          value += 1;
          success = drive_motor_fwd_bwd(motorfr, value, dir);
        }
        if (c == 115){//move backward with s
          success = false;
          value -= 1;
          success = drive_motor_fwd_bwd(motorfr, value, dir);
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
    motorfr->pid->updating = false;
    }
  }

}
