#include "motor.h"

void pid_reset(motor *motor_x) {
  /* Set all variables to 0 */

  motor_x->pid->reference = 0.0;
  motor_x->pid->current_error = 0;
  motor_x->pid->past_error = 0;
  motor_x->pid->error_sum = 0;
  motor_x->pid->current_action = 0;
  motor_x->pid->past_action = 0;

}

bool cmd_vel(motor *motor_x) {
  // wait until response time has passed
  if (motor_x->pid->response_time > motor_x->pid->wait_time) {

    // save past action
    //motor_x->pid->past_action = motor_x->pid->current_action;
    motor_x->pid->wait_time += TICKS_TIME;
    return 1;
  }

  // while not updating, don't do anything
  if (motor_x->pid->updating){
    return 1;
  }

  // past error
  motor_x->pid->past_error = motor_x->pid->current_error;

  // calculate current error
  motor_x->pid->current_error = motor_x->pid->reference -
    (motor_x->encoder->current_vel / (float) motor_x->clicks_per_rev);
  motor_x->pid->error_sum += motor_x->pid->current_error * motor_x->pid->wait_time;

  // save past action
  motor_x->pid->past_action = motor_x->pid->current_action;

  // command calculate action
  motor_x->pid->current_action += (motor_x->pid->current_error * motor_x->pid->kp
                                        + motor_x->pid->error_sum * motor_x->pid->ki
                                        + motor_x->pid->kd* (motor_x->pid->current_error -
                                           motor_x->pid->past_error)/TICKS_TIME);

  // don't allow actions greater than action limit
  if(abs(motor_x->pid->current_action) > motor_x->pid->action_limit) {
    if(signbit(motor_x->pid->current_action)) {
      // negative value
      motor_x->pid->current_action = -1 * motor_x->pid->action_limit;
    }
    else {
      motor_x->pid->current_action = motor_x->pid->action_limit;
    }
  }

  // reset wait time
  motor_x->pid->wait_time = 0.0;

  // recalculate history

  //if (lroundf(motor_x->pid->current_action) == lroundf(motor_x->pid->past_action)){
  //    return 0;
  //}
  return drive_motor(motor_x, (int16_t) lroundf(motor_x->pid->current_action));

}

