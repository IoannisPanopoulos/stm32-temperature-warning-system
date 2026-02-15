#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"

// Thresholds and temperatures
extern volatile float lower_threshold;
extern volatile float upper_threshold;
extern float current_temp;
extern float max_temp;
extern float min_temp;

// Thread function
void sensor_task(void);

// Min/Max tracking
void reset_hour_stats(float temp);
void update_max_min(float temp);

// Button interrupts
void configure_buttons(void);
void isr_lower_down(void);
void isr_lower_up(void);
void isr_upper_down(void);
void isr_upper_up(void);

#endif
