#include "motor.h"

bool cmd_vel(motor *motor_x) {
  motor_x->pid.current_error = motor_x->pid.reference - (motor_x->encoder.current_vel / (float) motor_x->clicks_per_rev);
  motor_x->pid.current_action = lroundf(motor_x->pid.current_error * motor_x->pid.kp);
  if(motor_x->pid.current_action == motor_x->pid.past_action) {
    return 0;
  }
  motor_x->pid.past_action = motor_x->pid.current_action;
  drive_motor(motor_x, motor_x->pid.current_action);
  return 1;
}

