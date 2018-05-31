#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

typedef struct filter {
  float *array;
  uint64_t insert_pos;
  uint64_t max_size;
} filter;

typedef struct encoder {
  volatile uint64_t systick_counter;
  volatile uint32_t past_timer_counter;
  volatile uint32_t current_timer_counter;
  volatile uint32_t used_timer_counter;
  volatile int64_t past_pos;
  volatile int64_t current_pos;
  volatile float past_vel;
  volatile float avg_vel;
  volatile float current_vel;
  volatile float current_accel;
  volatile bool uif;
  filter filter;
  uint64_t autoreload;
} encoder;

typedef struct timer {
  enum rcc_periph_clken clken;
  enum rcc_periph_clken clken_timer;
  uint32_t gpio_port;
  uint32_t gpio_pin;
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
  float error_sum_limit;
  int32_t current_action;
  int32_t past_action;
  int32_t action_limit;
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
  encoder encoder; // encoder related to this motor
  usart_port port; // usart port where the roboclaw for this motor is
  timer timer; // timer in stm for the encoder
  pid pid; // variables for pid controller
} motor;

bool cmd_vel(motor *motor_x);

#endif
