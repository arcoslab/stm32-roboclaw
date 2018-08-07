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
  /*
    This function will correctly update internal values of encoder
    struct for each motor, which contains information about
    encoder amout of ticks, and current velocity of ticks.

    Input: motor struct, which is defined at motor.h

   */

  // update systick counter, which is the amout of ticks
  // systick has made. Each tick occurs at TICK_TIME, which
  // is defined at systick.h
  motor_x->encoder->systick_counter++;

  // get the counter from TIM_X
  motor_x->encoder->current_timer_counter = timer_get_counter(motor_x->peripheral);

  // check if systick counter is bigger than autoreload
  // this is in the case that a long time has passed since there has been
  // any update in the counter of timer. If that happens, then set the vel to 0,
  // because the wheel is actually not moving
  if (motor_x->encoder->systick_counter > motor_x->encoder->autoreload) {
    motor_x->encoder->current_vel = 0.0;
  }


  if ( fabs(motor_x->encoder->past_timer_counter -
            motor_x->encoder->current_timer_counter) <= motor_x->encoder->n_ticks) {
    return 0;
  }


  // check if UIF flags has being raised. This flag indicates that the counter has reached
  // the end. Update Interrupt flag is set when the repetition counter is reloaded.
  if (timer_get_flag(motor_x->peripheral, TIM_SR_UIF) == 1) {
    if (motor_x->encoder->past_timer_counter >= ( (float) motor_x->period / 2.0) ) {
      motor_x->encoder->current_pos += 1;
    }
    else {
      motor_x->encoder->current_pos -= 1;
    }
    timer_clear_flag(motor_x->peripheral, TIM_SR_UIF);
  }

  else {
    // no flag was raised
    if (motor_x->encoder->current_timer_counter > motor_x->encoder->past_timer_counter) {
      motor_x->encoder->current_pos += (motor_x->encoder->current_timer_counter -
                                        motor_x->encoder->past_timer_counter);
    }
    else {
      motor_x->encoder->current_pos -= (motor_x->encoder->past_timer_counter -
                                        motor_x->encoder->current_timer_counter);
    }
  }

  // use filter for times, not for vel. Avg ticks is then the amout of systicks
  // between one encoder tick, and another. The time elapsed between on encoder tick
  // and another is then, avg ticks * the amout of time on systick event elapses.
  filter_push(&motor_x->encoder->filter, motor_x->encoder->systick_counter);
  motor_x->encoder->avg_ticks = filter_average(&motor_x->encoder->filter);

  // current vel - no need to do the substract because it's always 1.
  motor_x->encoder->current_vel =
    (motor_x->encoder->current_pos-motor_x->encoder->past_pos) /
    (motor_x->encoder->avg_ticks * TICKS_TIME);

  //motor_x->encoder->current_accel = motor_x->encoder->current_vel - motor_x->encoder->past_vel;

  // update past values
  motor_x->encoder->past_pos = motor_x->encoder->current_pos;
  //motor_x->encoder->past_vel = motor_x->encoder->current_vel;
  motor_x->encoder->past_timer_counter = motor_x->encoder->current_timer_counter;

  //motor_x->encoder->used_timer_counter = motor_x->encoder->systick_counter;
  motor_x->encoder->systick_counter=0;
  return 1;
}
