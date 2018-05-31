#ifndef __FILTER_H_
#define __FILTER_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "motor.h"

void filter_init(filter *filter_x);
void filter_push(filter *filter_x, float value);
float filter_average(filter *filter_x);

#endif // __FILTER_H_
