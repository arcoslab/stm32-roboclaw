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

#define LX 0.16
#define LY 0.1495
#define R 0.03
#define RAD_TO_REV 0.159155
#define REV_TO_RAD 6.28319
#define GLOBAL_POS_UPDATE_TIME 0.001 // this is 1ms

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

float global_pos[3] = {0.0,0.0,0.0};
float instant_vels[3] = {0.0,0.0,0.0};
float time_elapsed=0;

void sys_tick_handler(void) {
  /* This function will be called when systick fires, every 100us.
     It's purpose is to update the encoder pos, vel, and accel,
     and to generate the pid control signal
   */

  // pid control for motorfl
  encoder_update(motorfl);
  cmd_vel(motorfl);

  // pid control for motorfr
  encoder_update(motorfr);
  cmd_vel(motorfr);

  // pid control for motorrr
  encoder_update(motorrr);
  cmd_vel(motorrr);

  // pid control for motorrl
  encoder_update(motorrl);
  cmd_vel(motorrl);

  // calculate real pos of the robot
  if(time_elapsed > GLOBAL_POS_UPDATE_TIME) {
    // time elapsed reset
    time_elapsed = 0;

    // since update time is now, update global pos
    // calculate vx
    instant_vels[0] = (motorfl->encoder->current_vel +
                motorfr->encoder->current_vel +
                motorrl->encoder->current_vel +
                motorrr->encoder->current_vel) * (R/4.0) * REV_TO_RAD;

    // calculate vy
    instant_vels[1] = (-motorfl->encoder->current_vel +
               motorfr->encoder->current_vel +
               motorrl->encoder->current_vel -
               motorrr->encoder->current_vel) * (R/4.0) * REV_TO_RAD;

    // calculate angular vel
    instant_vels[2] = (-motorfl->encoder->current_vel +
               motorfr->encoder->current_vel -
               motorrl->encoder->current_vel +
               motorrr->encoder->current_vel) * (R/(4.0*(LX+LY))) * REV_TO_RAD;

    // finally update global pos
    global_pos[0] += instant_vels[0]*GLOBAL_POS_UPDATE_TIME;
    global_pos[1] += instant_vels[1]*GLOBAL_POS_UPDATE_TIME;
    global_pos[2] += instant_vels[2]*GLOBAL_POS_UPDATE_TIME;

  } // if time elapsed

  // add to time elapsed
  time_elapsed += TICKS_TIME;

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
  motorfl->encoder->filter.max_size = 8;
  filter_init(&motorfl->encoder->filter);
  encoder_init(motorfl->encoder);

  // front left motor encoder
  motorfr->encoder = malloc(sizeof(encoder));
  motorfr->encoder->autoreload = 10000;
  motorfr->encoder->filter.max_size = 8;
  filter_init(&motorfr->encoder->filter);
  encoder_init(motorfr->encoder);

  // rear right encoder init
  motorrr->encoder = malloc(sizeof(encoder));
  motorrr->encoder->autoreload = 10000;
  motorrr->encoder->filter.max_size = 8;
  filter_init(&motorrr->encoder->filter);
  encoder_init(motorrr->encoder);

  // rear left encoder init
  motorrl->encoder = malloc(sizeof(encoder));
  motorrl->encoder->autoreload = 10000;
  motorrl->encoder->filter.max_size = 8;
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
  motorfl->pid->kp = 0.025;
  motorfl->pid->ki = 0.35;
  motorfl->pid->kd = 0;
  motorfl->pid->action_limit = 127;
  motorfl->pid->wait_time = 0;
  motorfl->pid->response_time = 0.001; // 1 ms
  pid_reset(motorfl); // reset pid values

  // front right encoder
  motorfr->pid = malloc(sizeof(pid));
  motorfr->pid->kp = 0.025;
  motorfr->pid->ki = 0.35;
  motorfr->pid->kd = 0;
  motorfr->pid->action_limit = 127;
  motorfr->pid->wait_time = 0;
  motorfr->pid->response_time = 0.001; // 1 ms
  pid_reset(motorfr);

  // rear right encoder config
  motorrr->pid = malloc(sizeof(pid));
  motorrr->pid->kp = 0.025;
  motorrr->pid->ki = 0.35;
  motorrr->pid->kd = 0;
  motorrr->pid->action_limit = 127;
  motorrr->pid->wait_time = 0;
  motorrr->pid->response_time = 0.001; // 1 ms
  pid_reset(motorrr);

  // rear left encoder config
  motorrl->pid = malloc(sizeof(pid));
  motorrl->pid->kp = 0.025;
  motorrl->pid->ki = 0.35;
  motorrl->pid->kd = 0;
  motorrl->pid->action_limit = 127;
  motorrl->pid->wait_time = 0;
  motorrl->pid->response_time = 0.001; // 1 ms
  pid_reset(motorrl);

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
}

void convert_vel(float *vels) {
  /* Receive the velocities, convert to each motor vel*/

  motorfl->pid->reference = (1/R)*(vels[0] - vels[1] - (LX+LY)*vels[2])*RAD_TO_REV;
  motorfr->pid->reference = (1/R)*(vels[0] + vels[1] + (LX+LY)*vels[2])*RAD_TO_REV;
  motorrl->pid->reference = (1/R)*(vels[0] + vels[1] - (LX+LY)*vels[2])*RAD_TO_REV;
  motorrr->pid->reference = (1/R)*(vels[0] - vels[1] + (LX+LY)*vels[2])*RAD_TO_REV;

}

void read_instruction(char *c, float *vels) {
  /* Read stdin and command instruction */

  // read the instruction
  scanf("%c\n", c);

  // switch for different commands
  switch(*c) {

  case 'm' :
    // send back 0 for instruction ack
    printf("%d \n", 0);

    // expects 4 float values
    scanf("%f %f %f \n", &vels[0], &vels[1], &vels[2]);

    // command velocity
    convert_vel(vels);

    // finally send ack
    printf("%d \n", 10);

    // end this case
    break;

  case 'o' :
    // send back 1 for instruction ack
    //putc(1, stdout);
    printf("%d \n", 1);

    // send back 3 float values of the pos
    printf("%f %f %f\n", global_pos[0], global_pos[1], global_pos[2]);

    // finally send ack
    //putc(10, stdout);
    printf("%d \n", 10);

    break;

  case 'v' :
    // send back 2 for instruction ack
    //putc(2, stdout);
    printf("%d", 2);

    // send back 3 float values of the pos
    printf("%f %f %f", instant_vels[0], instant_vels[1], instant_vels[2]);

    // send back ack
   // putc(10, stdout);
    printf("%d", 10);

    break;

  case 'r' :
    // send back 3 for instruction ack
    //putc(3, stdout);
    printf("%d \n", 3);

    // reset pid settings
    pid_reset(motorfr);
    pid_reset(motorfl);
    pid_reset(motorrr);
    pid_reset(motorrl);

    // gloabl pos
    global_pos[0] = 0.0;
    global_pos[1] = 0.0;
    global_pos[2] = 0.0;

    // instant vels
    instant_vels[0] = 0.0;
    instant_vels[1] = 0.0;
    instant_vels[2] = 0.0;

    // send back ack
   // putc(10, stdout);
    printf("%d \n", 10);

    break;

  default:
    printf("%d \n", -1);

  }
}

int main(void)
{
  system_init();

  char c=0;
  float vels[3] = {0,0,0};

  setvbuf(stdin,NULL,_IONBF,0); // Sets stdin in unbuffered mode (normal for usart com)
  setvbuf(stdout,NULL,_IONBF,0); // Sets stdin in unbuffered mode (normal for usart com)

  // clean stdin
  while (poll(stdin) > 0) {
    //printf("Cleaning stdin\n");
    getc(stdin);
  }

  // read input forever
  while(1) {
    if (poll(stdin)>0) {
      // pause pid action
      motorfr->pid->updating = true;
      motorfl->pid->updating = true;
      motorrr->pid->updating = true;
      motorrl->pid->updating = true;

      // instruction character reset
      c=0;

      // read instruction function
      read_instruction(&c, vels);

      // renew pid action
      motorfr->pid->updating = false;
      motorfl->pid->updating = false;
      motorrr->pid->updating = false;
      motorrl->pid->updating = false;
    }
  }
}
