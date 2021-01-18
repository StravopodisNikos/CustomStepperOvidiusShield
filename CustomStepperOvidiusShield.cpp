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
//bool return_function_state;

const double pi              = 3.14159265359;

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
    _accel_width    = 0.20;                                 // Acceleration Phase width

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

// multiStepVarDelay
void CustomStepperOvidiusShield::multiStepVarDelay(unsigned long delayTime, uint32_t numSteps2Move) {
  // Custom Stepping Function for Velocity-Acceleration Profiles

  for (size_t i = 0; i < numSteps2Move; i++)
  {
    unsigned long time_now_micros = micros();
    digitalWrite(_stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(_stepPin, LOW);
  }

} // END function multiStepVarDelay

// =========================================================================================================== //

void CustomStepperOvidiusShield::read_STP_EEPROM_settings(volatile byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp)
{
	/*
	 *	FUN_CODE = 1 used for stp_error assignment, each error is given the value: 1*... for example: 11, 12, 13
   *  This function is executed at setup() to read form EEPROM and Initialize the global variables of Joint1 Stepper
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
  *	FUN_CODE = 2 used for stp_error assignment, each error is given the value: 1*... for example: 11, 12, 13
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

bool CustomStepperOvidiusShield::setStepperHomePositionSlow(uint32_t *currentAbsPos, double * currentAbsPos_double, volatile byte *currentDirStatus,  volatile bool *kill_motion_triggered,  int *stp_error){
/*
 *	FUN_CODE = 3 used for stp_error assignment, each error is given the value: 1*... for example: 11, 12, 13
 *  HOMES MOTOR - currentDirStatus changes value inside external interrupt arduino functions
 *  Returns true if error_code = 0
 */

  unsigned long homing_stepping_delay = STP_HOMING_DELAY;
  uint32_t breakFreeSteps             = BREAK_FREE_STEPS;

  *stp_error = 1;                             
  int times_limit_hall_activated = 0;

  digitalWrite(_dirPin, *currentDirStatus);       // start moving with the pre-existing direction value

  while( (digitalRead(_homeTriggerPin)) )         // move motor
  {         
      if (*kill_motion_triggered == true)
      {
        digitalWrite(_dirPin, *currentDirStatus); // change direction, since the currentDirStatus value was changed inside ISR function
        
        CustomStepperOvidiusShield::multiStepVarDelay(homing_stepping_delay, breakFreeSteps); // break free from magnetic field

        *kill_motion_triggered = false; // set the volatile value back to false
      }
      
      CustomStepperOvidiusShield::singleStepVarDelay(homing_stepping_delay);                  

      // Everything worked
      *stp_error = 0;
  }

  // sets global variable to new position(HOME) value
  *currentAbsPos = 0; 
  *currentAbsPos_double = 0;

if (*stp_error == 0)
{
  
  return true;
}
else
{
  return false;
}
} // END OF FUNCTION

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperHomePositionFast(double * currentAbsPos_double, volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool * kill_motion_triggered,  int * stp_error){
	// *	FUN_CODE = 4 used for stp_error assignment, each error is given the value: 1*... for example: 11, 12, 13
  
  bool return_function_state;
  unsigned long homing_stepping_delay = STP_HOMING_DELAY;                                // micros
  uint32_t relative_steps_2_move;
  uint32_t breakFreeSteps             = BREAK_FREE_STEPS;

  // 1.Read currentAbsPos_double and currentDirStatus from EEPROM
  
  //float currentAbsPos_double = 0.00f;
  EEPROM.get(CP_JOINT1_STEPPER_EEPROM_ADDR, *currentAbsPos_double);    			// @setup OR after every Action Task finishes: float f = 123.456f; EEPROM.put(eeAddress, f);
  *currentDirStatus = EEPROM.read(CD_JOINT1_STEPPER_EEPROM_ADDR);
  
  // 2.Calculate relative steps
  double goalAbsPos_double = 0;
  relative_steps_2_move = CustomStepperOvidiusShield::calculateRelativeSteps2Move(currentAbsPos_double, &goalAbsPos_double, stp_error);
  *relative_movement_in_steps = relative_steps_2_move;
  if (*stp_error == 92)
  {
    *stp_error = 492;
  }
  else if (*stp_error == 91)
  {
    *stp_error = 491;
  }
  else
  {
    *stp_error = 0;
  }

  return_function_state = CustomStepperOvidiusShield::setStepperGoalDirection(currentAbsPos_double, &goalAbsPos_double, currentDirStatus);
  if (return_function_state)
  {
    *stp_error = 0;
  }
  else
  {
    *stp_error = 41;
  }
/*
  // 3.  Define direction of motion
  if ( *currentAbsPos_double >= 0)
  {
    *currentDirStatus = CCW;						// move CCW
  }
  else
  {
    *currentDirStatus = CW;						// move CW
  }
  
  digitalWrite(_dirPin, *currentDirStatus);
*/

  // 4. execute homing  
  int motor_step = 0;
  // steps motor for pre-calculated number of steps and for the period that hall pin is not triggered
  while( (motor_step <= *relative_movement_in_steps) && (digitalRead(_homeTriggerPin) )) {
          
      CustomStepperOvidiusShield::singleStepVarDelay(homing_stepping_delay);

      motor_step++;

      if (*kill_motion_triggered == true)
      {
        digitalWrite(_dirPin, *currentDirStatus); // change direction, since the currentDirStatus value was changed inside ISR function
        
        CustomStepperOvidiusShield::multiStepVarDelay(homing_stepping_delay, breakFreeSteps); // break free from magnetic field

        *kill_motion_triggered = false; // set the volatile value back to false
      }

      *stp_error == 0;
  }

  // 5. sets global variable to new position(HOME) value
  if (*stp_error == 0)
  {
    (*currentAbsPos_double) = goalAbsPos_double;
    return true;
  }
  else
  {
    return false;
  } 
} // END OF FUNCTION

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperGoalDirection(double * currentAbsPos_double, double * goalAbsPos_double, volatile byte * currentDirStatus){
  // FUN_CODE = 5 used for stp_error assignment

  // Calculate Delta_q1
  double Delta_q1 = *goalAbsPos_double - *currentAbsPos_double;

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
  
  digitalWrite(_dirPin, (*currentDirStatus));
  
return true;
}

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperGoalPositionFixedStep(double * currentAbsPos_double, double * goalAbsPos_double, volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered,  int *stp_error)
{
  /*
   *  FUN_CODE = 6 used for stp_error assignment
   *  Executes sequence for joint-space operation
   */
  
  uint32_t relative_steps_2_move;
  bool return_function_state;

  // Determine Direction of Rotation
  return_function_state = CustomStepperOvidiusShield::setStepperGoalDirection(currentAbsPos_double, goalAbsPos_double, currentDirStatus);
  if (return_function_state)
  {
    *stp_error = 0;
  }
  else
  {
    *stp_error = 1;
  }
  
  // Calculate relative steps to move
  relative_steps_2_move = CustomStepperOvidiusShield::calculateRelativeSteps2Move(currentAbsPos_double, goalAbsPos_double, stp_error);
  *relative_movement_in_steps = relative_steps_2_move;
  if (*stp_error == 92)
  {
    *stp_error = 92;
  }
  else if (*stp_error == 91)
  {
    *stp_error = 91;
  }
  else
  {
   *stp_error = 0;
  }

  // Move Motor to goal position - kills motion if limit switch is ON!
  return_function_state = CustomStepperOvidiusShield::moveStp2Position(&relative_steps_2_move, currentDirStatus, kill_motion_triggered, stp_error);
  if (*stp_error == 81)
  {
    *stp_error = 81;
  }
  else if (*stp_error == 82)
  {
    *stp_error = 82;
  }
  else
  {
   *stp_error = 0;
  }
  
  if (*stp_error == 0)
  {
    (*currentAbsPos_double) = (*goalAbsPos_double);
    return true;
  }
  else
  {
    return false;
  }  

}

// =========================================================================================================== //

bool CustomStepperOvidiusShield::testP2Pparams_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double *Ta, int *stp_error) {

  /*
   *  FUN_CODE = 7 used for stp_error assignment
   *  This fn assumes that a known fixed relation between V,A exists
   *  1. Accepts current&goal joint position, p2p execution time and computes V,A|exec
   *  2. If V,A|exec <= V,A|max then sets (Tsync=Texec) 
   *  3. If V,A|exec > V,A|max then sends error code that requests to increase Texec and aborts
   */
  double *VelocityLimitStp;
  double *AccelerationLimitStp;

  // Calculate 
  double Delta_q1 = *goalAbsPos_double - *currentAbsPos_double;	// [rad]

  // Type of trajectory: Trajectory with assigned duratins T,Ta-p.69 Melchiorri
	*Vexec = abs( Delta_q1 ) / ( (1 - _accel_width) * (*Texec) )  ;  // [rad/sec]
	*Aexec = abs( Delta_q1 ) / ( _accel_width * ( 1 - _accel_width) * pow((*Texec),2.0) ) ;  // [rad/sec^2]
  *Ta = _accel_width * (*Texec);

  // Read Vmax,Amax for Stepper from EEPROM
  EEPROM.get(VL_JOINT1_STEPPER_EEPROM_ADDR, *VelocityLimitStp);    			

	EEPROM.get(AL_JOINT1_STEPPER_EEPROM_ADDR, *AccelerationLimitStp);  

  if ( ( (*Vexec) <= (*VelocityLimitStp) ) && ( (*Aexec) <= (*AccelerationLimitStp) ))
  {
    *stp_error = 71;
    // doesn't change Vexec,Axec,Texec,Ta,accel_width!
  }
  else
  {
    *stp_error = 72;
    // aborts execution
    // sets V,A to max available
    /*
    *Vexec =  *VelocityLimitStp;
    *Aexec =  *AccelerationLimitStp;
    // Recomputes Texec from eq. (3.8.3) p.65 Melchiorri-New type of trajectory: Trajectory with preassigned acceleration & velocity
    *Texec = (abs( Delta_q1 )*(*Aexec) + pow((*Vexec),2.0) ) / ( (*Aexec) * (*Vexec) );
    *Ta = (*Vexec) / (*Aexec);
    */
  }
  
  if ((*stp_error) == 71)
  {
    *stp_error = 71;
    return true;
  }
  /*
  else if ((*stp_error) == 72)
  {
    *stp_error = 72;
    return true;
  }
  */
  else
  {
    *stp_error = 72;
    return false;
  }
} 

// =========================================================================================================== //

bool CustomStepperOvidiusShield::moveStp2Position(uint32_t * relative_steps_2_move, volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, int *stp_error)
{
  /*
   *  FUN_CODE = 8 used for stp_error assignment
   *  Executes the specified number of steps
   */
  unsigned long fixed_stepping_delay = STP_FIXED_DELAY;
  uint32_t STP_CNT = 0;

  // Writes direction of rotation -> moved inside setStepperGoalDirection
  // digitalWrite(_dirPin, (*currentDirStatus));

  // executes motion
  while ( (!(*kill_motion_triggered)) && (STP_CNT <= (*relative_steps_2_move)))
  {
    STP_CNT++;
    CustomStepperOvidiusShield::singleStepVarDelay(fixed_stepping_delay);
    *stp_error = 81;
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
    
    *stp_error = 82;

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

uint32_t CustomStepperOvidiusShield::calculateRelativeSteps2Move(double * currentAbsPos_double, double * goalAbsPos_double, int * stp_error)
{
    // FUN_CODE = 9 used for stp_error assignment

    double Delta_q1_double = (*goalAbsPos_double) - (*currentAbsPos_double);

    int ERROR_RETURNED;

    uint32_t relative_steps_2_move =  CustomStepperOvidiusShield::convertRadian2StpPulses(Delta_q1_double, &ERROR_RETURNED);

    *stp_error = ERROR_RETURNED;

    if (*stp_error==101)
    {
      *stp_error = 101;
    }
    else if (*stp_error==102)
    {
      *stp_error = 102;
    }
    else
    {
      *stp_error = 93;
    }
    
    return relative_steps_2_move;
}

// =========================================================================================================== //

uint32_t CustomStepperOvidiusShield::convertRadian2StpPulses(double position_in_radians, int *stp_error)
{
    // FUN_CODE = 10* used for stp_error assignment

    uint32_t position_in_stp_pulses;

    if (position_in_radians == 0)
    {
        position_in_stp_pulses = 0;
        *stp_error = 101;
    }
    else 
    {
        // _ag = ( 2 * pi ) / ( _GEAR_FACTOR * _spr )
        position_in_stp_pulses = (uint32_t) (abs(position_in_radians) * GEAR_FACTOR_PLANETARY * SPR1) / (2*pi);
        //position_in_stp_pulses = (uint32_t) abs(position_in_radians) / _ag;
        *stp_error = 102;
    }

return position_in_stp_pulses;
}
  
// =========================================================================================================== //

double CustomStepperOvidiusShield::convertStpPulses2Radian(uint32_t position_in_stp_pulses, int *stp_error)
{
  // FUN_CODE = 11* used for stp_error assignment
    double position_in_radians;
    
    if (position_in_stp_pulses == 0)
    {
        position_in_radians = 0;
        *stp_error = 111;
    }
    else
    {   // _ag = ( 2 * pi ) / ( _GEAR_FACTOR * _spr )
        position_in_radians = (double) (position_in_stp_pulses * 2 * pi) / (SPR1 * GEAR_FACTOR_PLANETARY);
        //position_in_radians = (double) (position_in_stp_pulses) * _ag;
        *stp_error = 112;
    }
    
    return position_in_radians;
}

// =========================================================================================================== //

bool CustomStepperOvidiusShield::segmentExists_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double,  double & Vexec, double * Aexec, double & Texec, double & Ta,  bool * segmentExists, int *stp_error){

  // FUN_CODE = 12* used for stp_error assignment
  /*
   *  Given Vexec,Aexec checks if linear segment exists
   */

  double Delta_q1_double = *goalAbsPos_double - *currentAbsPos_double;	// [rad]

  double h_cond = pow((Vexec),2.0) / (*Aexec) ;				        // Condition factor for linear segment existence in Trapezoidal Velocity Profile

  if( Delta_q1_double >= h_cond)
  {
      *stp_error = 121;
      (*segmentExists) = true;
  }
  else
  {
      Ta = sqrt( abs(Delta_q1_double) / (*Aexec) );
      Texec = 2.0 * Ta;
      Vexec = (*Aexec) * Ta;

      *stp_error = 122;
      (*segmentExists) = false;
  }

  if (*stp_error==121)
  {
    return true;
  }
  else if (*stp_error==122)
  {
    return true;
  }
  else
  {
    *stp_error = 123;
    return false;
  }
  
}

// =========================================================================================================== //

double CustomStepperOvidiusShield::calculateInitialStepDelay(double * Aexec)
{
  // FUN_CODE = 13* used for stp_error assignment
  // Determine initial step delay time for real-time profile generation: c0 with ignored inaccuracy factor [sec]

  double sqrt_for_c0_calc =  (2 * _ag) /  (*Aexec);
  double c0              =  0.676 * _ft * pow(10.0,-6.0) * sqrt(sqrt_for_c0_calc);							                  

return c0;
}
// =========================================================================================================== //

bool CustomStepperOvidiusShield::returnSteps_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double, double & Vexec, double * Aexec, double * Texec, double * Ta,  bool * segmentExists, int * stp_error, uint32_t *profile_steps)
{
  // FUN_CODE = 14* used for stp_error assignment
  /*
   * RETURNS THE STEPS OF EACH P2P MOTION SEGMENT in an uint32_t array: profile_steps[] = {relative_steps_2_move, nmov_Ta, nmov_linseg, nmov_Td};
   */

    double Delta_q1_double = (*goalAbsPos_double) - (*currentAbsPos_double);

    int ERROR_RETURNED;
    uint32_t relative_steps_2_move;
    double Delta_q1_accel_double,Delta_q1_lin_seg_double;
    uint32_t nmov_Ta,nmov_Td,nmov_linseg;

    // first calculate the relative steps of all motion
    relative_steps_2_move = CustomStepperOvidiusShield::calculateRelativeSteps2Move(currentAbsPos_double, goalAbsPos_double, stp_error);
    if (*stp_error == 141)
    {
      *stp_error = 141;
    }
    else if (*stp_error == 142)
    {
      *stp_error = 142;
    }
    else
    {
      *stp_error = 0;
    }

    // segmentExists_StpTrapzVelProfile execution will be executed outside of this function

    // calculate steps of each segment
    if((*segmentExists))
    {
        // Determine Profile Step Variables based on Melchiorri
        Delta_q1_accel_double   = 0.5 * (*Aexec) * pow((*Ta),2.0);                                                        // eq.3.9.1 Melchiorri p.66
        
        Delta_q1_lin_seg_double = (*Aexec) * _accel_width * pow((*Texec),2.0) * ( 1 - 2 * _accel_width);                  // eq.3.9.2 Melchiorri p.66
        
        nmov_Ta     = CustomStepperOvidiusShield::convertRadian2StpPulses(Delta_q1_accel_double, &ERROR_RETURNED);        // Steps of Acceleration Phase; 
        nmov_linseg = CustomStepperOvidiusShield::convertRadian2StpPulses(Delta_q1_lin_seg_double, &ERROR_RETURNED);      // Steps of liner segment                                                               
        nmov_Td     = relative_steps_2_move - ( nmov_Ta + nmov_linseg ) ;									                                // Steps of Decceleration Phase; 
        *stp_error = 144;
    }
    else
    {
        // In this case: accel_width changes value! Vmax is recalculated! Texec is recalculated!
        Delta_q1_accel_double  = 0.5 * (*Aexec) * pow((*Ta),2.0);

        nmov_Ta      = CustomStepperOvidiusShield::convertRadian2StpPulses(Delta_q1_accel_double, &ERROR_RETURNED);        // Steps of Acceleration Phase;
        nmov_linseg  = 0;
        nmov_Td      = relative_steps_2_move - nmov_Ta ;                                                                   // Steps of Decceleration Phase;   
        
        *stp_error = 145;
    }
    
    *stp_error = ERROR_RETURNED;

    if (*stp_error==91)
    {
      *stp_error = 146;
    }
    else if (*stp_error==92)
    {
      *stp_error = 147;
    }
    else
    {
      *stp_error = 0;
    }

  // Returned array
  if (*stp_error==0)
  {
    profile_steps[0] = relative_steps_2_move;
    profile_steps[1] = nmov_Ta;
    profile_steps[2] = nmov_linseg;
    profile_steps[3] = nmov_Td;
    return true;
  }
  else
  {
    return false;
  }

}

// =========================================================================================================== //

bool CustomStepperOvidiusShield::execute_StpTrapzProfile(uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, int * stp_error)
{
	/*
   *  FUN_CODE = 15* used for stp_error assignment
	 *  INPUT: segmentExists, profile_steps = {relative_steps_2_move, nmov_Ta, nmov_linseg, nmov_Td}, 
   *         delta_t = c0(initial step delay), Texec=Motion Execution Time, delta_t = Initian step delay
	 */

    const float RAMP_FACTOR       = 1;                        // Velocity Profile ramp slope factor
    const float ETDF              = 1.50;                     // Execution Time Deviation Factor (experimental calculation)

    // Initialize counters for Profile Execution
    long StpPresentPosition = 0;													
    long accel_count = 0; 
    long ctVel_count = 0;
    long decel_count = -profile_steps[3]; 

    // Initialize variable for timing/derivative calculations
    unsigned long time_duration;
    double time_duration_double;
    double new_delta_t;
    double rest = 0;
    unsigned long motor_movement_start = millis();

  // Writes direction of rotation -> moved inside setStepperGoalDirection
  // digitalWrite(_dirPin, (*currentDirStatus));

    // Stepping loop
    do
    {
        StpPresentPosition++;
    		// Move stepper using Trapz Vel Profile by locating position in ramp
        if((*segmentExists))                                                                                                     // Linear Segment exists
        {
              if(StpPresentPosition < profile_steps[1]){
                accel_count++;                                                                                                // Acceleration Phase: delta_t -> minimizes
                new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*accel_count+1) );                                // c_n [sec]
              }else if( (StpPresentPosition > profile_steps[1]) && (StpPresentPosition < profile_steps[1]+profile_steps[2]) ) // Linear Segment: delta_t -> constant
              {
                ctVel_count++;
                new_delta_t = delta_t;  
              }
              else
              {
                decel_count++;                                                                        
                new_delta_t =  delta_t - RAMP_FACTOR * ( (2*delta_t+rest)/(4*decel_count+1) );                                // Deceleration Phase: delta_t -> maximizes [sec] 
              }
              *stp_error = 151;
        }
        else
        {                                                                                             // Linear Segment doesn't exist
              if(StpPresentPosition < profile_steps[1])                                               // Acceleration Phase: delta_t -> minimizes
              {
                accel_count++;
                new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*accel_count+1) );          // c_n [sec]
              }                                   
              else
              {                                                                                       // Deceleration Phase: delta_t -> maximizes
                decel_count++;                                                                        // Negative Value!
                new_delta_t = delta_t - RAMP_FACTOR * ((2*delta_t+rest)/(4*decel_count+1) );          // Deceleration Phase: delta_t -> maximizes [sec] 
              }
              *stp_error = 152;                                                                       
        }

        unsigned long new_delta_t_micros = (new_delta_t*1000000);

        CustomStepperOvidiusShield::singleStepVarDelay(new_delta_t_micros);                           // Executes Motor Step 

        // Calculate Actual Angular Velocity    ->Needs timer
        // Calculate Actual Angular Acceleration->Needs timer
        
        delta_t = new_delta_t;                                                                        // Changes Motor Step Delay

        time_duration = millis() - motor_movement_start;                                              // Calculates Stepper Motion Execution Time 
        time_duration_double = time_duration / 1000.0;
         
    //}while( ( time_duration_double < ( ETDF * (*Texec)) ) && (abs(profile_steps[0] - StpPresentPosition) != 0) );
    }while(  (abs(profile_steps[0] - StpPresentPosition) != 0) );
    //}while(  ( time_duration_double < (*Texec) ) != 0) );

    if ( time_duration_double <= ( ETDF * (*Texec)) ) // if synced motion was successful
    {
      *Texec = time_duration_double;
      *stp_error = 0;
    }
    else
    {
      *Texec = time_duration_double;
      *stp_error = 154;
    }


    if (*stp_error == 0)
    {
      return true;
    }
    else
    {
      return false;
    }

} // END OF FUNCTION

// =========================================================================================================== //

bool CustomStepperOvidiusShield::setStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta,  volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered, bool * segment_exists, uint32_t * profile_steps,   int *stp_error)
{
  // FUN_CODE = 16* used for stp_error assignment
  /*
   * Follows flowchart of setStepperGoalPositionFixedStep but now a var step is used in each segment of motion
   * This function executes a Trapezoidal Velocity Profile. It can be used for synced motion with Dynamixels.
   */
    bool return_function_state;

    // I . Determine Direction of Rotation
    return_function_state = CustomStepperOvidiusShield::setStepperGoalDirection(currentAbsPos_double, goalAbsPos_double, currentDirStatus);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = 161;
    }
    
    // II . In fixed step here only the number of relative steps to move was calculated. Here the function flowchart is different:
    // II.1 Given (currentAbsPos_double,goalAbsPos_double,Texec) from user the Vexec,Axec are calculated -> testP2Pparams_StpTrapzVelProfile -> returns: Vexec,Aexec,Texec(if changed)
    // II.2 Given (currentAbsPos_double,goalAbsPos_double,Texec) and extracted (Vexec,Aexec,Texec')      -> segmentExists_StpTrapzVelProfile -> returns if segment exists and new Vexec,Texec(if changed)
    // II.3 Given II.2 the conditions for the Trapezoidal Profile are evaluted -> returnSteps_StpTrapzVelProfile -> An array of the steps of each segment is returned
    // II.4 Executes Motion with Trapezoidal Profile
    int ERROR_RETURNED;
    return_function_state = CustomStepperOvidiusShield::testP2Pparams_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, Vexec, Aexec, Texec, Ta, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    bool SEGMENT_EXISTS; 
    return_function_state = CustomStepperOvidiusShield::segmentExists_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double,  *Vexec, Aexec, *Texec, *Ta, &SEGMENT_EXISTS, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    //uint32_t PROFILE_STEPS[4];
    return_function_state = CustomStepperOvidiusShield::returnSteps_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, *Vexec, Aexec, Texec, Ta,  &SEGMENT_EXISTS, stp_error, profile_steps);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = 163;
    }

    // *this fn will be removed for sync motion with dxl! The final Texec must be given to the Dxl's. Dxl setGoalPosition must be sent and then EXECUTE stepper motion
    // here for testing the stepper timing only!
    double c0 = CustomStepperOvidiusShield::calculateInitialStepDelay(Aexec);

    return_function_state = CustomStepperOvidiusShield::execute_StpTrapzProfile(profile_steps, &SEGMENT_EXISTS,  Texec,  c0, currentDirStatus, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    if (*stp_error == 0)
    {
      (*currentAbsPos_double) = (*goalAbsPos_double);
      return true;
    }
    else
    {
      return false;
    }
    
}

// =========================================================================================================== //

bool CustomStepperOvidiusShield::syncPreSetStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta,  volatile byte * currentDirStatus, bool * segment_exists, uint32_t * profile_steps,   int *stp_error)
{
  // FUN_CODE = 17* used for stp_error assignment
  /*
   * Follows function 16. This function IS used for synced motion with Dynamixels. To execute Stepper motion
   * syncSetStepperGoalPositionVarStep must be called after txPacket for Dxl GP has been sent!
   * Final Texec,Ta must be passed to calculateProfAccel_preassignedVelTexec for syncing motion with Dxls!
   */
    bool return_function_state;

    // I . Determine Direction of Rotation
    return_function_state = CustomStepperOvidiusShield::setStepperGoalDirection(currentAbsPos_double, goalAbsPos_double, currentDirStatus);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = 171;
    }
    
    // II . In fixed step here only the number of relative steps to move was calculated. Here the function flowchart is different:
    // II.1 Given (currentAbsPos_double,goalAbsPos_double,Texec) from user the Vexec,Axec are calculated -> testP2Pparams_StpTrapzVelProfile -> returns: Vexec,Aexec,Texec(if changed)
    // II.2 Given (currentAbsPos_double,goalAbsPos_double,Texec) and extracted (Vexec,Aexec,Texec')      -> segmentExists_StpTrapzVelProfile -> returns if segment exists and new Vexec,Texec(if changed)
    // II.3 Given II.2 the conditions for the Trapezoidal Profile are evaluted -> returnSteps_StpTrapzVelProfile -> An array of the steps of each segment is returned
    // II.4 Executes Motion with Trapezoidal Profile
    int ERROR_RETURNED;
    return_function_state = CustomStepperOvidiusShield::testP2Pparams_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, Vexec, Aexec, Texec, Ta, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    bool SEGMENT_EXISTS; 
    return_function_state = CustomStepperOvidiusShield::segmentExists_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double,  *Vexec, Aexec, *Texec, *Ta, &SEGMENT_EXISTS, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    //uint32_t PROFILE_STEPS[4];
    return_function_state = CustomStepperOvidiusShield::returnSteps_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, *Vexec, Aexec, Texec, Ta,  &SEGMENT_EXISTS, stp_error, profile_steps);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = 173;
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

bool CustomStepperOvidiusShield::syncSetStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, double * Texec,  volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, bool * segment_exists, uint32_t * profile_steps,   int *stp_error)
{
  // FUN_CODE = 18* used for stp_error assignment

  /*
   * Executed after function 17 has completed. In main code, previously, syncSetDynamixelsGoalPosition must be called!
   */

  bool return_function_state;
  int ERROR_RETURNED;

  double c0 = CustomStepperOvidiusShield::calculateInitialStepDelay(Aexec);

  return_function_state = CustomStepperOvidiusShield::execute_StpTrapzProfile(profile_steps, segment_exists,  Texec,  c0, currentDirStatus, &ERROR_RETURNED);
  if (return_function_state)
  {
    *stp_error = 0;
  }
  else
  {
    *stp_error = ERROR_RETURNED;
  }

  if (*stp_error == 0)
  {
    (*currentAbsPos_double) = (*goalAbsPos_double);
    return true;
  }
  else
  {
    return false;
  }

}
