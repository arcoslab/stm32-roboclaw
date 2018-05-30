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

bool encoder_update(encoder *encoder_x, uint16_t current_timer_counter, bool uif){
  /* Input parameters:
   *encoder_x: the struct containing all information for this encoder
   *current_timer_counter: the current internal register counter of the timer
   *uif: flag of update, raised only is there's an autoreload in the register of the timer
   * The function will return 1 is there was an update of pos.
   */
  encoder_x->systick_counter++;

  encoder_x->current_pos = current_timer_counter;
  //fprintf(stdout, "\n%d\n", current_timer_counter);
  return 1;

  if (encoder_x->systick_counter > 5000) {
    encoder_x->current_vel = 0.0;
  }

  if (encoder_x->past_pos == encoder_x->current_pos) {
    return 0;
  }

  if (uif == 1) {

  }

  encoder_x->current_vel = (float) (encoder_x->past_pos - encoder_x->current_pos)/( ((float) encoder_x->systick_counter) * TICKS_TIME);

  encoder_x->past_vel = encoder_x->current_vel;
  encoder_x->past_pos = encoder_x->current_pos;
  encoder_x->systick_counter=0;
  return 1;

}
