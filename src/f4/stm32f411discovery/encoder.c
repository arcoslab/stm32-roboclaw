#include "encoder.h"

void encoder_init(encoder *encoder_x){
  encoder_x->systick_counter = 0;
  encoder_x->past_timer_counter = 0;
  encoder_x->current_timer_counter = 0;
  encoder_x->past_pos = 0;
  encoder_x->current_pos = 0;
  encoder_x->past_vel = 0;
  encoder_x->current_vel = 0.0;
  encoder_x->current_accel = 0.0;
  encoder_x->uif = 0;
}

bool encoder_update(motor *motor_x){
  /* Input parameters:
   *encoder_x: the struct containing all information for this encoder
   *current_timer_counter: the current internal register counter of the timer
   *uif: flag of update, raised only is there's an autoreload in the register of the timer
   * The function will return 1 is there was an update of pos.
   */

  motor_x->encoder->systick_counter++;

  motor_x->encoder->current_timer_counter = timer_get_counter(motor_x->timer->peripheral);

  if (motor_x->encoder->systick_counter > motor_x->encoder->autoreload) {
    motor_x->encoder->current_vel = 0.0;
  }

  if (motor_x->encoder->past_timer_counter == motor_x->encoder->current_timer_counter) {
    return 0;
  }

  if (timer_get_flag(motor_x->timer->peripheral, TIM_SR_UIF) == 1) {
    if (motor_x->encoder->past_timer_counter >= ( (float) motor_x->timer->period / 2.0) ) {
      motor_x->encoder->current_pos += 1;
    }
    else {
      motor_x->encoder->current_pos -= 1;
    }
    timer_clear_flag(motor_x->timer->peripheral, TIM_SR_UIF);
  }

  else {
    // no flag was raised
    if (motor_x->encoder->current_timer_counter > motor_x->encoder->past_timer_counter) {
      motor_x->encoder->current_pos += motor_x->encoder->current_timer_counter - motor_x->encoder->past_timer_counter;
    }
    else {
      motor_x->encoder->current_pos -= motor_x->encoder->past_timer_counter - motor_x->encoder->current_timer_counter;
    }
  }

  // use filter for times, not for vel. With vel the error is increased.
  filter_push(&motor_x->encoder->filter, motor_x->encoder->systick_counter);
  motor_x->encoder->avg_ticks = filter_average(&motor_x->encoder->filter);

  // current vel - no need to do the substract because it's always 1.
  motor_x->encoder->current_vel = (motor_x->encoder->current_pos-motor_x->encoder->past_pos) / (motor_x->encoder->avg_ticks * TICKS_TIME);

  // current vel - old vel
  // motor_x->encoder->current_vel = (float) (motor_x->encoder->current_pos - motor_x->encoder->past_pos)/( ((float) motor_x->encoder->systick_counter) * TICKS_TIME);

  motor_x->encoder->current_accel = motor_x->encoder->current_vel - motor_x->encoder->past_vel;

  // update past values
  motor_x->encoder->past_pos = motor_x->encoder->current_pos;
  motor_x->encoder->past_vel = motor_x->encoder->current_vel;
  motor_x->encoder->past_timer_counter = motor_x->encoder->current_timer_counter;

  motor_x->encoder->used_timer_counter = motor_x->encoder->systick_counter;
  motor_x->encoder->systick_counter=0;
  return 1;
}
