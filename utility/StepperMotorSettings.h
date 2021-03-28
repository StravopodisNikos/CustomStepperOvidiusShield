// Stepper Motor and Driver Settings
#ifndef StepperMotorSettings_h
#define StepperMotorSettings_h

#define RED_LED_PIN                	    10            // these are digital pins with pwm capabilities DUE:2-13
#define GREEN_LED_PIN               	4                               
#define BLUE_LED_PIN               	    11            // GRIPPER_SERVO_PIN = 12 (!) defined in <utility/OvidiusSensors_config.h>
#define ACCEL_WIDTH_DENOM		        5
#define MIN_STEP_DELAY_MICROS           25            // because Stepper driver's frequency 20kHz
#define MOTION_TIMEOUT_MILLIS           3000            
#define PAUSE_FOR_DXL_COM_MILLIS        1 
#define HOMING_TIMEOUT_MILLIS		    5000	
#include <avr/pgmspace.h>
// CAUTION! -> PINS 0,1,2,7,8 ARE USED FOR SERIAL PORT COMMUNICATION AND MUST NOT BE CONFIGURED!!!

// Ovidius Manipulator Structure Properties
const PROGMEM unsigned char nDoF        =	4;
const PROGMEM unsigned char DXL_MOTORS	=   3;

// MOTOR ID: STP1_ID Joint1 axis Stepper Nema34

const PROGMEM uint8_t HOME_TRIGGER_SWITCH = 3;                // This is the Homing trigger switch Previous name:HALL_SWITCH_PIN1
// IF DXL SHIELD ->
const PROGMEM uint8_t HALL_SWITCH_PIN2    = 18;				  // This is the Hall effect-MIN limit switch	(4 when no interrupt used)		
const PROGMEM uint8_t HALL_SWITCH_PIN3    = 19;			      // This is the Hall effect-MAX limit switch    (5 when no interrupt used)
// IF DXL SHIELD <-
const PROGMEM float   ETDF = 1.50f;
const PROGMEM uint8_t STP_MOVING_STATUS_THRESHOLD   = 1;
const PROGMEM uint8_t STEP_Pin                      = 6;
const PROGMEM uint8_t DIR_Pin        	            = 9;
const PROGMEM uint8_t ENABLE_Pin      	            = 5;
const PROGMEM uint8_t GEAR_FACTOR_PLANETARY  	    = 40;       // Gear Reducer used
const PROGMEM uint8_t GEAR_FACTOR_PLANETARY_TEST    = 1;        // Only for testing experimental arrangement

const PROGMEM int32_t FT_CLOSED_LOOP                = 200000;   // Stepper Driver Frequency [1/micros]->[*10^-6 * 1/sec]

const PROGMEM uint16_t SPR_1                        = 3200;     // Steps per Revolution from Dip Switch Driver Configuration [steps/rev]
const PROGMEM uint16_t STP_HOMING_DELAY		        = 500;
const PROGMEM uint16_t STP_FIXED_DELAY			    = 500;
const PROGMEM uint16_t BREAK_FREE_STEPS		        = 500;

const PROGMEM uint8_t PROF_STEPS_SIZE	            = 4;		// sizeof array that contains vel prof total steps/phase	

#endif
