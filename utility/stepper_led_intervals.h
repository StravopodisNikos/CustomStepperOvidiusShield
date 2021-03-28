// Custom Stepper Library Led Intervals 
// Can be used for visual debugging
#ifndef stepper_led_intervals_h
#define stepper_led_intervals_h

#include <avr/pgmspace.h>
// led indicators
// can be accessed in libraries/meta_controller_configuration

// led intervals
const PROGMEM int16_t UPDATE_IMU_MEAS_INTERVAL 	        =   250;
const PROGMEM int16_t UPDATE_FORCE_MEAS_INTERVAL        =	250;
const PROGMEM int16_t UPDATE_POS_MEAS_INTERVAL 	        =   250;
const PROGMEM int16_t UPDATE_VEL_MEAS_INTERVAL 	        =   250;
const PROGMEM int16_t UPDATE_CURRENT_MEAS_INTERVAL 	    =   250;

#endif