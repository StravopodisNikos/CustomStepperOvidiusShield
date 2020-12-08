 /*
  * CustomStepperOvidiusShield.cpp - Library for controlling Steppers of the Ovidius Metamorphic Manipulator
  * Created by N.A. Stravopodis, December, 2020.
  */

#include "Arduino.h"
#include <EEPROM.h>
#include "CustomStepperOvidiusShield.h"
#include <StepperMotorSettings.h>

using namespace std;

// Constructor
CustomStepperMetamorphicManipulator::CustomStepperMetamorphicManipulator(int stepID, int stepPin, int dirPin, int enblPin, int homeTriggerPin, int limitSwitchPin2, int limitSwitchPin3, int RED_LED_pin,int GREEN_LED_pin,int BLUE_LED_pin, int spr, int GEAR_FACTOR, int ft )
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

    _spr            = spr;                                  // Steps per revolution [Found from driver dip switch configuration]
    _GEAR_FACTOR    = GEAR_FACTOR;                          // Gearbox Reduction of stepper motor [depends on Gearbox attached to motor]
    _ft             = ft;                                   // Frequency of stepper driver [Found in Stepper-Motor Driver Specifications]

    _a              = ( 2 * pi ) / (  _spr );               // Stepper Motor Step Angle(Angular Position of Motor shaft)[rad]
    _ag             = ( 2 * pi ) / ( _GEAR_FACTOR * _spr ); // Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
    _accel_width    = 0.10;                                 // Acceleration Phase width

}

// =========================================================================================================== //

// singleStepVarDelay
void CustomStepperMetamorphicManipulator::singleStepVarDelay(unsigned long delayTime) {
  // Custom Stepping Function for Velocity-Acceleration Profiles

    unsigned long time_now_micros = micros();
    digitalWrite(_stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(_stepPin, LOW);
} // END function singleStepVarDelay

// =========================================================================================================== //

void CustomStepperMetamorphicManipulator::read_STP_EEPROM_settings( byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp)
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

void CustomStepperMetamorphicManipulator::save_STP_EEPROM_settings(byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp)
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

// setStepperHomePositionSlow
bool CustomStepperMetamorphicManipulator::setStepperHomePositionSlow(){

  unsigned long homing_stepping_delay = 500;

  Serial.print("[ Stepper ID: "); Serial.print(_stepID); Serial.println("] HOMING ... ");
  
  digitalWrite(_ledPin, HIGH);                                                                          
  
  while( (digitalRead(_homeTriggerPin) == 0 ) )
  {   
      //move motor                                                                     
      CustomStepperMetamorphicManipulator::singleStepVarDelay(homing_stepping_delay);                  

      //either MIN or MAX limit switches triggered
      if( (digitalRead(_limitSwitchPin2) == HIGH) || (digitalRead(_limitSwitchPin3) == HIGH) )
      {
          // Change DIR Pin status
          digitalWrite(_dirPin, !currentDirStatus);
      }
  }

  // sets global variable to new position(HOME) value
  long currentAbsPos = 0; 

  Serial.print("[ Stepper ID: "); Serial.print(_stepID); Serial.println("] HOMING FINISHED ");
  
  digitalWrite(_ledPin, LOW);                        

return true;
} // END OF FUNCTION

// =========================================================================================================== //

// setStepperHomePositionFast
bool CustomStepperMetamorphicManipulator::setStepperHomePositionFast(double * currentAbsPos_double, unsigned long * currentAbsPos, byte * currentDirStatus){

  unsigned long homing_stepping_delay = 500;                                // micros

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
  Serial.print("[ Stepper ID: "); Serial.print(_stepID); Serial.println("] HOMING ... ");
  
  digitalWrite(_ledPin, HIGH);                                                                          
  
  int motor_step = 0;
  // steps motor for pre-calculated number of steps and for the period that hall pin is not triggered
  while( (motor_step <= *currentAbsPos) && (digitalRead(_homeTriggerPin) == 0 )){
          
		  time_now_micros = micros();

			digitalWrite(_stepPin, HIGH);
    	while(micros() < time_now_micros + homing_stepping_delay){}          //wait approx. [μs]
    	digitalWrite(_stepPin, LOW);

      motor_step++;
  }

  // 5. sets global variable to new position(HOME) value
  *currentAbsPos = 0; 

  Serial.print("[ Stepper ID: "); Serial.print(_stepID); Serial.println("] HOMING FINISHED ");
  
  digitalWrite(_ledPin, LOW);                        

return true;
} // END OF FUNCTION

// =========================================================================================================== //