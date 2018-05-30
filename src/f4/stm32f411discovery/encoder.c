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

  motor_x->encoder.systick_counter++;

  motor_x->encoder.current_pos = timer_get_counter(motor_x->timer.peripheral);

  if (motor_x->encoder.systick_counter > motor_x->encoder.autoreload) {
    motor_x->encoder.current_vel = 0.0;
  }

  if (motor_x->encoder.past_pos == motor_x->encoder.current_pos) {
    return 0;
  }

  if (timer_get_flag(motor_x->timer.peripheral, TIM_SR_UIF) == 1) {

  }

  motor_x->encoder.current_vel = (float) (motor_x->encoder.past_pos - motor_x->encoder.current_pos)/( ((float) motor_x->encoder.systick_counter) * TICKS_TIME);

  motor_x->encoder.past_vel = motor_x->encoder.current_vel;
  motor_x->encoder.past_pos = motor_x->encoder.current_pos;
  motor_x->encoder.systick_counter=0;
  return 1;

}
