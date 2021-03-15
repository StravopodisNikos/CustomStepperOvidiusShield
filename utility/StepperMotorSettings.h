// Stepper Motor and Driver Settings

// CAUTION! -> PINS 0,1,2,7,8 ARE USED FOR SERIAL PORT COMMUNICATION AND MUST NOT BE CONFIGURED!!!

// Ovidius Manipulator Structure Properties
#define nDoF						4
#define DXL_MOTORS					3

// MOTOR ID: STP1_ID Joint1 axis Stepper Nema34

#define HOME_TRIGGER_SWITCH        	    3                // This is the Homing trigger switch Previous name:HALL_SWITCH_PIN1
// IF DXL SHIELD ->
#define HALL_SWITCH_PIN2        	    18				 // This is the Hall effect-MIN limit switch	(4 when no interrupt used)		
#define HALL_SWITCH_PIN3        	    19				 // This is the Hall effect-MAX limit switch    (5 when no interrupt used)
// IF DXL SHIELD <-
// IF OPENCR1.0 ->
//#define HALL_SWITCH_PIN2        	42				 // EXTI_Pin2 = 5			
//#define HALL_SWITCH_PIN3        	45               // EXTI_Pin3 = 6
//#define EXTI_Pin2			5
//#define EXTI_Pin3			6
// IF OPENCR1.0 <-

#define STEP_Pin               		    6
#define DIR_Pin                		    9
#define ENABLE_Pin               	    5

#define RED_LED_PIN                	    A0            // the number of the RED LED pin
#define GREEN_LED_PIN               	A1                               
#define BLUE_LED_PIN               	    A2                               

#define SPR1                   		    3200          // Steps per Revolution from Dip Switch Driver Configuration [steps/rev]
#define GEAR_FACTOR_PLANETARY  	     	40            // Gear Reducer used
#define GEAR_FACTOR_PLANETARY_TEST	    1             // Only for testing experimental arrangement
#define FT_CLOSED_LOOP                  200000        // Stepper Driver Frequency [1/micros]->[*10^-6 * 1/sec]

#define STP_MOVING_STATUS_THRESHOLD     1

#define STP_HOMING_DELAY		        500
#define STP_FIXED_DELAY			        500
#define BREAK_FREE_STEPS		        500

#define ACCEL_WIDTH_DENOM		        5
#define PROF_STEPS_SIZE			        4				// sizeof array that contains vel prof total steps/phase	

