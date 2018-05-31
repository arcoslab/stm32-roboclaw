#include "filter.h"

void filter_init(filter *filter_x) {
  filter_x->array = malloc(filter_x->max_size * sizeof(float));
  filter_x->insert_pos = 0;
}

void filter_push(filter *filter_x, float value) {
  if (filter_x->insert_pos == filter_x->max_size) {
    filter_x->insert_pos = 0;
  }
  filter_x->array[filter_x->insert_pos++] = value;
}

float filter_average(filter *filter_x) {
  uint64_t i;
  float sum = 0.0;
  for (i = 0; i<filter_x->max_size; i++) {
    sum += filter_x->array[i];
  }
  return sum / filter_x->max_size;
}

