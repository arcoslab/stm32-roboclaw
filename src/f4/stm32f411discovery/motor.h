#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include "systick.h"

typedef struct filter {
  float *array;
  uint64_t insert_pos;
  uint64_t max_size;
} filter;

typedef struct encoder {
  volatile uint64_t systick_counter;
  volatile float avg_ticks;
  volatile uint32_t past_timer_counter;
  volatile uint32_t current_timer_counter;
  volatile uint32_t used_timer_counter;
  volatile int64_t past_pos;
  volatile int64_t current_pos;
  volatile float past_vel;
  volatile float current_vel;
  volatile float current_accel;
  volatile bool uif;
  filter filter;
  uint64_t autoreload;
} encoder;

typedef struct timer {
  enum rcc_periph_clken clken_ch1;
  enum rcc_periph_clken clken_ch2;
  enum rcc_periph_clken clken_timer;
  uint32_t gpio_port_ch1;
  uint32_t gpio_port_ch2;
  uint32_t gpio_pin_ch1;
  uint32_t gpio_pin_ch2;
  uint32_t period;
  uint32_t peripheral;
  uint8_t gpio_af;
  enum tim_ic_id ic1;
  enum tim_ic_input in1;
  enum tim_ic_id ic2;
  enum tim_ic_input in2;
  uint8_t mode;
} timer;

typedef struct usart_port{
  uint32_t usart;
  uint32_t baudrate;
  uint32_t gpio_port;
  uint16_t gpio_pin;
  uint8_t gpio_af;
  enum rcc_periph_clken clken;
  enum rcc_periph_clken clken_usart;
} usart_port;

typedef struct pid {
  float kp;
  float ki;
  float kd;
  float reference;
  float current_error;
  float past_error;
  float error_sum;
  float current_action;
  float past_action;
  float action_limit;
  float response_time; // to avoid too many changes in a row
  float wait_time; // how much time to wait until calculation
  bool updating; // disable drive while updating value
} pid;

typedef struct motor {
  /* Virtual make motors appears as different
   * and the contents inside this struct will manage
   * to use the right usart com port and address
  */
  uint8_t address; // address to corresponding roboclaw in that port
  bool code; // each roboclaw has two motors. Choose between them
  uint64_t clicks_per_rev; // amount of encoder events for one complete output shaft revolution
  float wheel_radius; // wheel radius to calculate circunference
  encoder *encoder; // encoder related to this motor
  usart_port *port; // usart port where the roboclaw for this motor is
  timer *timer; // timer in stm for the encoder
  pid *pid; // variables for pid controller
  uint32_t peripheral; // save the timer to use
  uint32_t period; // save the timer period
  uint32_t usart; // save the usart port for roboclaw
} motor;

void pid_reset(motor *motor_x);
bool cmd_vel(motor *motor_x);

#endif
