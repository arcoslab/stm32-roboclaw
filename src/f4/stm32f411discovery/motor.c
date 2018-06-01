#include "motor.h"

bool cmd_vel(motor *motor_x) {
  if (motor_x->pid.response_time > motor_x->pid.wait_time) {
    motor_x->pid.wait_time += TICKS_TIME;
    return 0;
  }

  // reset wait time
  motor_x->pid.wait_time = 0.0;

  // calculate current error
  motor_x->pid.current_error = motor_x->pid.reference -
    (motor_x->encoder.current_vel / (float) motor_x->clicks_per_rev);

  // command calculate action
  motor_x->pid.current_action = lroundf(motor_x->pid.current_error * motor_x->pid.kp
                                        + motor_x->pid.error_sum * motor_x->pid.ki
                                        + (motor_x->pid.current_error -
                                           motor_x->pid.past_error) * motor_x->pid.kd
                                        + motor_x->pid.past_action);

  // don't command speed if nothing has changed.
  if(motor_x->pid.current_action == motor_x->pid.past_action) {
    return 0; // do nothing if the action hasn't changed
  }

  // don't allow error sum too increase too much.
  if(abs(motor_x->pid.error_sum) > motor_x->pid.error_sum_limit) {
    motor_x->pid.error_sum = motor_x->pid.error_sum_limit;
  }

  // don't allow actions greater than action limit
  if(abs(motor_x->pid.current_action) > motor_x->pid.action_limit) {
    return 0;
  }

  motor_x->pid.past_action = motor_x->pid.current_action;

  drive_motor(motor_x, motor_x->pid.current_action);

  return 1;
}

