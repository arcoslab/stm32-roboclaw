#include "encoder.h"

void encoder_init(encoder encoder_x){
  encoder_x.systick_counter = 0;
  encoder_x.past_timer_counter = 0;
  encoder_x.current_timer_counter = 0;
  encoder_x.past_pos = 0;
  encoder_x.current_pos = 0;
  encoder_x.past_vel = 0;
  encoder_x.current_vel = 0.0;
  encoder_x.current_accel = 0.0;
  encoder_x.uif = 0;
}

bool encoder_update(encoder encoder_x, uint64_t systick_counter, uint16_t current_timer_counter, bool uif){
  /* Input parameters:
   *encoder_x: the struct containing all information for this encoder
   *systick_counter: the counter for how many times systick has
   *current_timer_counter: the current interal register counter of the timer
   *uif: flag of update, raised only is there's an autoreload in the register of the timer
   * The function will return 1 is there was an update of pos, meaning that
   * the systick counter should be reset
   */

  if ()

}













