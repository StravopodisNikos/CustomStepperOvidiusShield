 /*
  * CustomStepperOvidiusShield.cpp - Library for controlling Steppers of the Ovidius Metamorphic Manipulator
  * Created by N.A. Stravopodis, December, 2020.
  */

#include "Arduino.h"
#include <EEPROM.h>
#include "CustomStepperOvidiusShield.h"
#include <StepperMotorSettings.h>

using namespace std;

enum ROT_DIR stepperDirRotation;
bool return_function_state;

// Constructor
CustomStepperOvidiusShield::CustomStepperOvidiusShield(int stepID, int stepPin, int dirPin, int enblPin, int homeTriggerPin, int limitSwitchPin2, int limitSwitchPin3, int RED_LED_pin,int GREEN_LED_pin,int BLUE_LED_pin, int spr, int GEAR_FACTOR, int ft )
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enblPin, OUTPUT);
    pinMode(homeTriggerPin, INPUT_PULLUP);       // HOME TRIGGER SWITCH 
    pinMode(limitSwitchPin2, INPUT);             // LIMIT HALL SWITCH
    pinMode(limitSwitchPin3, INPUT);             // LIMIT HALL SWITCH

    pinMode(RED_LED_pin, OUTPUT);
    pinMode(GREEN_LED_pin, OUTPUT);
    pinMode(BLUE_LED_pin, OUTPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(enblPin, LOW);
    digitalWrite(dirPin, LOW);

    _stepID         = stepID;
    _stepPin        = stepPin;
    _dirPin         = dirPin;
    _enblPin        = enblPin;
    _homeTriggerPin    = homeTriggerPin;
    _limitSwitchPin2   = limitSwitchPin2;
    _limitSwitchPin3   = limitSwitchPin3;

    _RED_LED_pin    = RED_LED_pin;
    _GREEN_LED_pin  = GREEN_LED_pin;
    _BLUE_LED_pin   = BLUE_LED_pin;

    _spr            = spr;                                  // Steps per revolution [Found from driver dip switch configuration]
    _GEAR_FACTOR    = GEAR_FACTOR;                          // Gearbox Reduction of stepper motor [depends on Gearbox attached to motor]
    _ft             = ft;                                   // Frequency of stepper driver [Found in Stepper-Motor Driver Specifications]

    _a              = ( 2 * pi ) / (  _spr );               // Stepper Motor Step Angle(Angular Position of Motor shaft)[rad]
    _ag             = ( 2 * pi ) / ( _GEAR_FACTOR * _spr ); // Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
    _accel_width    = 0.10;                                 // Acceleration Phase width

}

// =========================================================================================================== //

// singleStepVarDelay
void CustomStepperOvidiusShield::singleStepVarDelay(unsigned long delayTime) {
  // Custom Stepping Function for Velocity-Acceleration Profiles

    unsigned long time_now_micros = micros();
    digitalWrite(_stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(_stepPin, LOW);
} // END function singleStepVarDelay

// =========================================================================================================== //

void CustomStepperOvidiusShield::read_STP_EEPROM_settings(volatile byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp)
{
	/*
	 *	This function is executed at setup() to read form EEPROM and Initialize the global variables of Joint1 Stepper
	 */
  EEPROM.get(CP_JOINT1_STEPPER_EEPROM_ADDR, *currentAbsPos_double);

	*currentDirStatus = EEPROM.read(CD_JOINT1_STEPPER_EEPROM_ADDR);

	EEPROM.get(VL_JOINT1_STEPPER_EEPROM_ADDR, *VelocityLimitStp);    			

	EEPROM.get(AL_JOINT1_STEPPER_EEPROM_ADDR, *AccelerationLimitStp);    			

  EEPROM.get(MAX_POS_JOINT1_STEPPER_EEPROM_ADDR, *MaxPosLimitStp); 

} // END FUNCTION: readEEPROMsettings

void CustomStepperOvidiusShield::save_STP_EEPROM_settings(volatile byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp)
{
	/*
	 *	Executed before exit setup\action loops - NEVER INSIDE LOOP
	 */

		// 1. Save the dirPin status only if value has changed
		EEPROM.update(CD_JOINT1_STEPPER_EEPROM_ADDR, *currentDirStatus);

		// 2. Save current absolute position in which stepper motor finished job
		EEPROM.put(CP_JOINT1_STEPPER_EEPROM_ADDR, *currentAbsPos_double);

		// 3. Save stepper Velocity/Acceleration Limits
		EEPROM.put(VL_JOINT1_STEPPER_EEPROM_ADDR, *VelocityLimitStp);

		EEPROM.put(AL_JOINT1_STEPPER_EEPROM_ADDR, *AccelerationLimitStp);

    // 4. Save Joint1 Limit Angle
    EEPROM.put(MAX_POS_JOINT1_STEPPER_EEPROM_ADDR, *MaxPosLimitStp);

} // END FUNCTION: saveEEPROMsettings

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperHomePositionSlow(unsigned long *currentAbsPos, volatile byte *currentDirStatus,  volatile bool *kill_motion_triggered,  int *stp_error){
/*
 *  HOMES MOTOR - currentDirStatus changes value inside external interrupt arduino functions
 *  Returns true if error_code = 0
 */

  unsigned long homing_stepping_delay = STP_HOMING_DELAY;
  
  *stp_error = 1;                             
  int times_limit_hall_activated = 0;

  while( (digitalRead(_homeTriggerPin)) )
  {   
      //move motor
      digitalWrite(_dirPin, *currentDirStatus);

      CustomStepperOvidiusShield::singleStepVarDelay(homing_stepping_delay);                  

      // Everything worked
      *stp_error = 0;
  }

  // sets global variable to new position(HOME) value
  *currentAbsPos = 0; 

if (*stp_error == 0)
{
  *kill_motion_triggered = false;
  return true;
}
else
{
  return false;
}
} // END OF FUNCTION

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperHomePositionFast(double * currentAbsPos_double, uint32_t * currentAbsPos, volatile byte * currentDirStatus){

  unsigned long homing_stepping_delay = STP_HOMING_DELAY;                                // micros

  // 1.Read currentAbsPos_double and currentDirStatus from EEPROM
  
  //float currentAbsPos_double = 0.00f;

  EEPROM.get(CP_JOINT1_STEPPER_EEPROM_ADDR, *currentAbsPos_double);    			// @setup OR after every Action Task finishes: float f = 123.456f; EEPROM.put(eeAddress, f);
  *currentAbsPos = abs( 	round( *currentAbsPos_double / _ag)    );
  *currentDirStatus = EEPROM.read(CD_JOINT1_STEPPER_EEPROM_ADDR);
  
  // 2.Calculate relative steps
  // Relative steps for homing is the absolute number of steps calculated

  // 3.  Define direction of motion
  if ( *currentAbsPos_double >= 0)
  {
    *currentDirStatus = LOW;						// move CCW
  }
  else
  {
    *currentDirStatus = HIGH;						// move CW
  }
  
  digitalWrite(_dirPin, *currentDirStatus);

  // 4. execute homing  

  int motor_step = 0;
  // steps motor for pre-calculated number of steps and for the period that hall pin is not triggered
  while( (motor_step <= *currentAbsPos) && (digitalRead(_homeTriggerPin) == 0 )){
          
      CustomStepperOvidiusShield::singleStepVarDelay(homing_stepping_delay);

      motor_step++;
  }

  // 5. sets global variable to new position(HOME) value
  *currentAbsPos = 0; 

return true;
} // END OF FUNCTION

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperGoalDirection(double currentAbsPos_double, double goalAbsPos_double, volatile byte * currentDirStatus){

  // Calculate Delta_q1
  double Delta_q1 = goalAbsPos_double - currentAbsPos_double;

  if (Delta_q1 < 0 )
  {
    stepperDirRotation = CCW;
    * currentDirStatus = stepperDirRotation;
  }
  else if (Delta_q1 > 0 )
  {
    stepperDirRotation = CW;
    * currentDirStatus = stepperDirRotation;  
  }
  else
  {
    // does nothing
  }
  
  
return true;
}

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperGoalPositionFixedStep(double * currentAbsPos_double, double * goalAbsPos_double, volatile byte * currentDirStatus, volatile bool *kill_motion_triggered,  int *stp_error){
  /*
   *  Executes sequence for joint-space operation
   */
  
  uint32_t relative_steps_2_move;

  // Determine Direction of Rotation
  return_function_state = CustomStepperOvidiusShield::setStepperGoalDirection(*currentAbsPos_double, *goalAbsPos_double, currentDirStatus);
  if (return_function_state)
  {
    *stp_error = 0;
  }
  else
  {
    *stp_error = 1;
  }
  
  // Calculate relative steps to move
  relative_steps_2_move = CustomStepperOvidiusShield::calculateRelativeSteps2Move(currentAbsPos_double, goalAbsPos_double);

  // Move Motor to goal position - kills motion if limit switch is ON!
  return_function_state = CustomStepperOvidiusShield::moveStp2Position(relative_steps_2_move, currentDirStatus, kill_motion_triggered, stp_error);
  if (return_function_state)
  {
    *stp_error = 0;
  }
  else
  {
    *stp_error = stp_error;
  }
  
  if (*stp_error == 0)
  {
    * currentAbsPos_double = * goalAbsPos_double;
    return true;
  }
  else
  {
    return false;
  }  

}

// =========================================================================================================== //

bool CustomStepperOvidiusShield::moveStp2Position(uint32_t relative_steps_2_move, volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, int *stp_error)
{
  /*
   *  Executes the specified number of steps
   */
  unsigned long fixed_stepping_delay = STP_FIXED_DELAY;
  uint32_t STP_CNT = 0;

  // Writes direction of rotation
  digitalWrite(_dirPin, *currentDirStatus);

  // executes motion
  while ( (!(*kill_motion_triggered)) && (STP_CNT <= relative_steps_2_move))
  {
    STP_CNT++;
    CustomStepperOvidiusShield::singleStepVarDelay(fixed_stepping_delay);
    *stp_error = 555;
  }

  // if KILL_MOTION=true inside the ISR:
  // I.  previous while exits
  // II. and 1.stop 1sec 2. change dir. 3. move away of danger for 1000. 4. set error_code that requests homing
  if (*kill_motion_triggered)
  {
    delay(1000);

    digitalWrite(_dirPin, !(*currentDirStatus));

    for (size_t i = 0; i < 1000; i++)
    {
      CustomStepperOvidiusShield::singleStepVarDelay(fixed_stepping_delay);
    }
    
    *stp_error = 666;

  }
  else
  {
    *stp_error = 0;
  }
  
  if (*stp_error == 0)
  {
    return true;
  }
  else
  {
    return false;
  }  

}

// =========================================================================================================== //

uint32_t CustomStepperOvidiusShield::calculateRelativeSteps2Move(double * currentAbsPos_double, double * goalAbsPos_double)
{
    double Delta_q1 = goalAbsPos_double - currentAbsPos_double;

    uint32_t relative_steps_2_move = (abs(Delta_q1) * SPR1)/ (2*PI);
}

// =========================================================================================================== //

uint32_t CustomStepperOvidiusShield::convertRadian2StpPulses(double position_in_radians)
{
    uint32_t position_in_stp_pulses;

    if (position_in_radians == 0)
    {
        position_in_stp_pulses = 0;
    }
    else 
    {
        position_in_stp_pulses = (abs(position_in_radians) * SPR1)/ (2*PI);
    }

return position_in_stp_pulses;
}
  
// =========================================================================================================== //

double CustomStepperOvidiusShield::convertStpPulses2Radian(uint32_t position_in_stp_pulses)
{
    double position_in_radians;
    
    if (position_in_stp_pulses == 0)
    {
        position_in_radians = (double) position_in_stp_pulses;
    }
    else
    {
        position_in_radians = (double) (position_in_stp_pulses * 2 * PI) / SPR1 ;
    }
    
    return position_in_radians;
}

// =========================================================================================================== //