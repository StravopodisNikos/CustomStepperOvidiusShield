 /*
  * CustomStepperOvidiusShield.cpp - Library for controlling Steppers of the Ovidius Metamorphic Manipulator
  * Created by N.A. Stravopodis, December, 2020.
  */

#include "Arduino.h"
#include <EEPROM.h>
#include "CustomStepperOvidiusShield.h"
#include <utility/StepperMotorSettings.h>
#include <utility/stepper_led_intervals.h>

#include <OvidiusSensors.h>
#include <utility/OvidiusSensors_config.h>
#include <utility/OvidiusSensors_debug.h>

#include <Dynamixel2Arduino.h>
#include "DynamixelProPlusOvidiusShield.h"

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

    _a              = (double) ( ( 2.0 * pi ) / (  _spr ) );               // Stepper Motor Step Angle(Angular Position of Motor shaft)[rad]
    _ag             = (double) ( ( 2.0 * pi ) / ( _GEAR_FACTOR * _spr ) ); // Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
    _accel_width    = (double) ( 1.0 / ACCEL_WIDTH_DENOM );                                 // Acceleration Phase width
    
    _accel_count    = 0;
    _ctVel_count    = 0;
    _decel_count    = 0;

    _return_fn_state = false;

    // STATES INTERVALS
    _update_FORCE_interval     = UPDATE_FORCE_MEAS_INTERVAL;
    _update_joint_vel_interval = UPDATE_VEL_MEAS_INTERVAL;
    _update_joint_pos_interval = UPDATE_POS_MEAS_INTERVAL;

    // STEPPER MONITORING
    _currentAngVel_rps      = 0;
    _currentAngAccel_rps2   = 0;
    _prevAbsPos_rad         = 0;
    _prevAngVel_rps         = 0;
    _prevAngAccel_rps2      = 0;

    // STATE UPDATES INITIALIZATON
    _STEP_PIN_STATE         = HIGH;
    _last_STEP_STATE_update = 0;
    _last_LED_update        = 0;
    _last_FORCE_update      = 0;
    _last_IMU_update        = 0;

    //Serial.print("SPR = "); Serial.println(_spr);
}

// =========================================================================================================== //
//
//  P R I V A T E -- C L A S S -- M E M B E R S
//
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

// updateSingleStepVarDelay
void CustomStepperOvidiusShield::updateSingleStepVarDelay(unsigned long delayTime, long * StpPresentPosition, bool * updateDelayTime) {
  // Stepping Function to implement Velocity-Acceleration Profiles using
  // state machine multitasking - Here no pausing inside stepping occurs
  // Only half step pulse is generated! Total+steps/phase must be multiplied x 2! 

    _update_STEP_STATE_interval = delayTime;
    if (micros() - _last_STEP_STATE_update > _update_STEP_STATE_interval)
    {
      digitalWrite(_stepPin, !_STEP_PIN_STATE);

      _last_STEP_STATE_update = micros();

      _STEP_PIN_STATE = !_STEP_PIN_STATE;

      (*StpPresentPosition)++;  

      *updateDelayTime = true;
    }
    else
    {
      //*StpPresentPosition = *StpPresentPosition;
      //(*StpPresentPosition)++;

      *updateDelayTime = false;
    }
    
} 

// =========================================================================================================== //

void CustomStepperOvidiusShield::updateDelayTime(double * new_delayTime_sec, double * prev_delayTime_sec, long StpPresentPosition, bool * segment_exists, uint32_t * profile_steps, int *stp_error)
{
    // This function is executed only if updateSingleStepVarDelay sets the corresponding flag to true
    // New delay time calculated in [sec] is returned with "delayTime_sec"

    const float RAMP_FACTOR       = 1.0;                        // Velocity Profile ramp slope factor

    // Initialize counters for Profile Execution
    double rest = 0;

    if((*segment_exists))                                                                                                     // Linear Segment exists
    {
      if(StpPresentPosition < profile_steps[1])
      {
        _accel_count++;                                                                                                       // Acceleration Phase: delta_t -> minimizes
        //Serial.print("EXISTS : ACCEL PHASE"); Serial.println(_accel_count);
        (*new_delayTime_sec) =  *prev_delayTime_sec - RAMP_FACTOR * ( (2*(*new_delayTime_sec)+rest)/(4*_accel_count+1) );     // c_n [sec]
      }
      else if( (StpPresentPosition > profile_steps[1]) && (StpPresentPosition < profile_steps[1]+profile_steps[2]) )          // Linear Segment: delta_t -> constant
      {
        _ctVel_count++;
        //Serial.print("EXISTS : CTVEL PHASE"); Serial.println(_ctVel_count);
        (*new_delayTime_sec) = (*prev_delayTime_sec);  
      }
      else
      {
        _decel_count++; 
        //Serial.print("EXISTS : DECEL PHASE"); Serial.println(_decel_count);                                                                           
        (*new_delayTime_sec) =  (*prev_delayTime_sec) - RAMP_FACTOR * ( (2*(*new_delayTime_sec)+rest)/(4*_decel_count+1) );    // Deceleration Phase: delta_t -> maximizes [sec] 
      }
      *stp_error = 151;
    }
    else
{                                                                                                                               // Linear Segment doesn't exist
      if(StpPresentPosition < profile_steps[1])                                                                                 // Acceleration Phase: delta_t -> minimizes
      {
        _accel_count++;
        //Serial.print("NOT EXISTS : ACCEL PHASE"); Serial.println(_accel_count);
        (*new_delayTime_sec) = (*prev_delayTime_sec) - RAMP_FACTOR * ((2*(*new_delayTime_sec)+rest)/(4*_accel_count+1) );       // c_n [sec]
      }                                   
      else
    {                                                                                                                           // Deceleration Phase: delta_t -> maximizes
        _decel_count++;                                                                                                         // Negative Value!
        //Serial.print("NOT EXISTS : DECEL PHASE"); Serial.println(_decel_count); 
        (*new_delayTime_sec) = (*prev_delayTime_sec) - RAMP_FACTOR * ((2*(*new_delayTime_sec)+rest)/(4*_decel_count+1) );       // Deceleration Phase: delta_t -> maximizes [sec] 
      }
      *stp_error = 152;                                                                       
    }

  //Serial.print("delayTime_sec in updateDelayTime = "); Serial.println(*new_delayTime_sec,6);

  (*prev_delayTime_sec) = (*new_delayTime_sec);

  (*new_delayTime_sec) = (*new_delayTime_sec) / 2.0 ;       // Because half step pulse is generated each time!
}
// =========================================================================================================== //
void CustomStepperOvidiusShield::updateForceMeasurements(sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , bool &update_force , debug_error_type * force_error)
{
  // this function is executed inside execute_StpTrapzProfile2 if user sets flag to true at method call
  // Updates force measurements while motor is stepping applying state machine multitasking. The force 
  // sensor must have been calibrated BEFORE CALLING this function. (setupForceSensor of OvidiusSensors)
  // must have been called at setup().

    //ForceSensor = ptr2ForceSensor;

    if (millis() - _last_FORCE_update > _update_FORCE_interval)
    {
      //for (size_t i = 0; i < 1; i++)
      //{
        //_return_fn_state =  ForceSensor[i].measureForceKilos(&(ForceSensor->ForceSensorAxis), (UPDATED_FORCE_MEASUREMENTS_KGS+i), *force_error);
        //_return_fn_state =  ForceSensor->measureForceKilos(&ptr2ForceSensor->ForceSensorAxis, UPDATED_FORCE_MEASUREMENTS_KGS, force_error);
        _return_fn_state = ptr2ForceSensor->measureForceKilos(ptr2ForceSensorAxis, UPDATED_FORCE_MEASUREMENTS_KGS, force_error);

        _last_FORCE_update = millis();
        /*
        if (_return_fn_state)
        {
          Serial.println("MEASURED FORCE");
          Serial.print("MEASUREMENT = "); Serial.println(*UPDATED_FORCE_MEASUREMENTS_KGS);
        }
        else
        {
          Serial.println("NOT MEASURED FORCE"); Serial.print("FORCE ERROR = "); Serial.println(*force_error);
          Serial.print("MEASUREMENT = "); Serial.println(*UPDATED_FORCE_MEASUREMENTS_KGS);
        }
        */
      //}
      update_force = true;
    }
    else
    {
      update_force = false;
    }  

}
// =========================================================================================================== //

void CustomStepperOvidiusShield::updateIMU(sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet, sensors::imu_filter FILTER_SELECT, bool &update_imu, debug_error_type * imu_error)
{
  // this function is executed inside execute_StpTrapzProfile2 if user sets flag to true at method call
  // Updates imu measurements while motor is stepping applying state machine multitasking. The imu 
  // sensor must have been initialized BEFORE CALLING this function. (corresponding setup method of OvidiusSensors
  // must have been called at setup ).

    // must read private var _FILTER_INTERVAL_MILLIS and give it to _update_IMU_rate_hz
    int update_IMU_rate_hz;

    ptr2IMU->getFilterInterval(&update_IMU_rate_hz);

    if ( millis() - _last_IMU_update > (1000 / update_IMU_rate_hz) )
    {
      _return_fn_state = ptr2IMU->measure_with_filter_IMU2(ptr2imu_packet, FILTER_SELECT, imu_error);

      _last_IMU_update = millis();

      if (_return_fn_state)
      {
        //Serial.print("ROLL  = "); Serial.println(ptr2imu_packet->roll_c);
        //Serial.print("PITCH = "); Serial.println(ptr2imu_packet->pitch_c);
        Serial.print("YAW   = "); Serial.println(ptr2imu_packet->yaw_c);
      }
      else
      {
        Serial.println("NOT MEASURED IMU"); Serial.print("IMU ERROR = "); Serial.println(*imu_error);
      }

      update_imu = true;
    }
    else
    {
      update_imu = false;
    }
    
}

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

void CustomStepperOvidiusShield::updateStpAngVelStp(double &current_ang_vel, double half_step_delay_sec)
{
  // this fn works only with state machine functions. Since in state machine
  // half pulses are generated for the calculated delay time, here wmega is 
  // extracted for half step info.

  //set_ag();                       // executed at ocject creation in the constructor fn!

  double half_step_rad = _ag / 2.0;

  current_ang_vel = half_step_rad / half_step_delay_sec;

  return;
}

void CustomStepperOvidiusShield::updateStpAbsPos_rad(double &realTimeStpAbsPos, uint32_t StpPresentPulse, int *stp_error)
{
  // StpPresentPulse is the counter of pulses for relative p2p stepper movement!
  // _prevAbsPos_rad MUST be initialized right before p2p execution loop!

  // first find dq from loop execution pulse counter
  double Delta_q_rad = convertStpPulses2Radian(StpPresentPulse, stp_error);

  realTimeStpAbsPos = _prevAbsPos_rad + (_sign_of_Dq * Delta_q_rad);

  return;
}

// =========================================================================================================== //

void CustomStepperOvidiusShield::set_ag()
{
  // Deprecated function. Was used to debug constructor error.
  // Constructor was not able to see the StepperMotorSettings.h
  // when it was placed on different folder! [7-3-21]

  _ag = 2.0 * pi;
  _ag = (double) _ag / _spr;
}

void CustomStepperOvidiusShield::getJointsAbsPosition_rad(double * ROBOT_ABS_POS_RAD, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, uint32_t StpPresentPulse, bool &update_joint_pos, int *error_code)
{
    // implemented for state machine only - not in ino file!
    // RETURN nDoFx1 array ROBOT_ABS_POS_RAD (~= currentConfiguration in ino file)

    if ( millis() - _last_joint_pos_update > _update_joint_pos_interval )
    {
      // stepper
      updateStpAbsPos_rad(_currentAbsPos_rad, StpPresentPulse, error_code);
      ROBOT_ABS_POS_RAD[0] = _currentAbsPos_rad;
      // dxl
      // call dxl library function that reads position
      ptr2custom_dxl->syncGetDynamixelsPresentPosition(ptr2_dxl_pp_pck->Dxl_ids, ptr2_dxl_pp_pck->Dxl_ids_size, ptr2_dxl_pp_pck->dxl_pp, ptr2_dxl_pp_pck->SR_pp, error_code, ptr2_dxl_pp_pck->dxl2ard_obj);
      // convert dxl pulses -> radians
      ROBOT_ABS_POS_RAD[1] = ptr2custom_dxl->convertDxlPulses2Radian(ptr2_dxl_pp_pck->dxl_pp[0]);
      ROBOT_ABS_POS_RAD[2] = ptr2custom_dxl->convertDxlPulses2Radian(ptr2_dxl_pp_pck->dxl_pp[1]);
      ROBOT_ABS_POS_RAD[3] = ptr2custom_dxl->convertDxlPulses2Radian(ptr2_dxl_pp_pck->dxl_pp[2]);

      _last_joint_pos_update = millis();

      update_joint_pos = true;
    }
    else
    {
      update_joint_pos = false;
    }
}

void CustomStepperOvidiusShield::getJointsAngVelocity_rs(double * ROBOT_ANG_VEL_RS, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PV_PACKET *ptr2_dxl_pv_pck, double half_step_delay_sec, bool &update_joint_vel, int *error_code)
{
    // implemented for state machine only - not in ino file!
    if ( millis() - _last_joint_vel_update > _update_joint_vel_interval )
    {
      // stepper
      updateStpAngVelStp(_currentAngVel_rps, half_step_delay_sec);
      ROBOT_ANG_VEL_RS[0] = _currentAngVel_rps;

      // dxl
      // call dxl library function that reads velocity
      ptr2custom_dxl->syncGetDynamixelsPresentVelocity(ptr2_dxl_pv_pck->Dxl_ids,ptr2_dxl_pv_pck->Dxl_ids_size, ptr2_dxl_pv_pck->dxl_pv, ptr2_dxl_pv_pck->SR_pv, error_code, ptr2_dxl_pv_pck->dxl2ard_obj);
      // convert dxl vel units -> radians/sec
      ROBOT_ANG_VEL_RS[1] = ptr2custom_dxl->convertDxlVelUnits2RadPsec(ptr2_dxl_pv_pck->dxl_pv[0]);
      ROBOT_ANG_VEL_RS[2] = ptr2custom_dxl->convertDxlVelUnits2RadPsec(ptr2_dxl_pv_pck->dxl_pv[1]);
      ROBOT_ANG_VEL_RS[3] = ptr2custom_dxl->convertDxlVelUnits2RadPsec(ptr2_dxl_pv_pck->dxl_pv[2]);

      _last_joint_vel_update = millis();

      update_joint_vel = true;
    }
    else
    {
      update_joint_vel = false;
    }
    
} 
// =========================================================================================================== //
//
//  P U B L I C -- C L A S S -- M E M B E R S
//
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

bool CustomStepperOvidiusShield::setStepperHomePositionSlow(double * currentAbsPos_double, volatile byte *currentDirStatus,  volatile bool *kill_motion_triggered,  int *stp_error){
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
  *currentAbsPos_double = 0;
  // set private var to current abs pos in rad
  _currentAbsPos_rad    = *currentAbsPos_double;

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

bool CustomStepperOvidiusShield::setStepperHomePositionFast(double * currentAbsPos_double, volatile byte * currentDirStatus, volatile bool * kill_motion_triggered,  int * stp_error){
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

  // 3.  Define direction of motion
  return_function_state = CustomStepperOvidiusShield::setStepperGoalDirection(currentAbsPos_double, &goalAbsPos_double, currentDirStatus);
  if (return_function_state)
  {
    *stp_error = 0;
  }
  else
  {
    *stp_error = 41;
  }

  // 4. execute homing  
  int motor_step = 0;
  // steps motor for pre-calculated number of steps and for the period that hall pin is not triggered
  while( (motor_step <= relative_steps_2_move) && (digitalRead(_homeTriggerPin) )) {
          
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
    stepperDirRotation = CW;
    * currentDirStatus = stepperDirRotation;
    _sign_of_Dq = -1;
  }
  else if (Delta_q1 > 0 )
  {
    stepperDirRotation = CCW;
    * currentDirStatus = stepperDirRotation; 
    _sign_of_Dq = 1;
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

    double Delta_q1_double = abs( (*goalAbsPos_double) - (*currentAbsPos_double) );

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
        position_in_stp_pulses = (uint32_t) ( (abs(position_in_radians) * _GEAR_FACTOR * SPR1) / (2*pi) );
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
        position_in_radians = (double) ( (position_in_stp_pulses * 2 * pi) / (SPR1 * GEAR_FACTOR_PLANETARY) );
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

bool CustomStepperOvidiusShield::segmentExists_StpTrapzVelProfile2(double * currentAbsPos_double, double * goalAbsPos_double,  double & Vexec, double * Aexec, double & Texec, double & Ta,  bool * segmentExists, int *stp_error){

  // FUN_CODE = __ used for stp_error assignment
  /*
   *  Based on segmentExists_StpTrapzVelProfile2 BUT here calculateProfVelAccel_preassignedVelTexec3 was used in p2pcsp2.ino file
   */

  double Delta_q1_double = abs( *goalAbsPos_double - *currentAbsPos_double );	// [rad]

  double h_cond = abs( pow((Vexec),2.0) / (*Aexec) );				        // Condition factor for linear segment existence in Trapezoidal Velocity Profile

  if( Delta_q1_double >= h_cond)
  {
      *stp_error = 121;
      (*segmentExists) = true;
  }
  else
  {
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
    double Delta_q1_accel_double;
    double Delta_q1_lin_seg_double;
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
        
        // Returns junk value, I will use the simplest form Ta=Td
        //Delta_q1_lin_seg_double = (*Aexec) * _accel_width * pow((*Texec),2.0) * ( 1 - 2 * _accel_width);                  // eq.3.9.2 Melchiorri p.66

        nmov_Ta     = CustomStepperOvidiusShield::convertRadian2StpPulses(Delta_q1_accel_double, &ERROR_RETURNED);        // Steps of Acceleration Phase; 
        nmov_linseg = relative_steps_2_move - 2 *  nmov_Ta;                                                             
        nmov_Td     = nmov_Ta;
        *stp_error = ERROR_RETURNED;
    }
    else
    {
        // In this case: accel_width changes value! Vmax is recalculated! Texec is recalculated!
        Delta_q1_accel_double  = 0.5 * (*Aexec) * pow((*Ta),2.0);

        nmov_Ta      = CustomStepperOvidiusShield::convertRadian2StpPulses(Delta_q1_accel_double, &ERROR_RETURNED);        // Steps of Acceleration Phase;
        nmov_linseg  = 0;
        nmov_Td      = relative_steps_2_move - nmov_Ta ;                                                                   // Steps of Decceleration Phase;   
        
        *stp_error = ERROR_RETURNED; 
    }
    
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

//bool CustomStepperOvidiusShield::execute_StpTrapzProfile2(sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS ,debug_error_type *force_error, uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, bool UPDATE_FORCE, bool UPDATE_IMU, int * stp_error)
bool CustomStepperOvidiusShield::execute_StpTrapzProfile2(tools::dataLogger *ptr2logger, File *ptr2logfiles, sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet,  sensors::imu_filter FILTER_SELECT,  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, bool UPDATE_FORCE, bool UPDATE_IMU, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, int * stp_error)
{
	/*
   * Same as execute_StpTrapzProfile, but is USED FOR STATE MACHINE ONLY!
	 */
    // Log Files: 0->position, 1->velocity, 2->force, 3->current, 4->imu

    // Array for real time position/velocity
    double CURRENT_CONFIGURATION_RAD[nDoF];
    double CURRENT_JOINT_VELOCITY_RS[nDoF];
    unsigned long joint_pos_log_cnt = 0;
    unsigned long joint_vel_log_cnt = 0;
    bool updateJointPosLog;
    bool updateJointVelLog;
    // Vars for imu measurements
    bool updateImuLog;

    // Vars for force measurements
    //ptr2ForceSensor = ForceSensor;
    unsigned long force_update_duration;
    unsigned long started_force_update;
    unsigned long imu_update_duration;
    unsigned long started_imu_update;
    unsigned long force_log_cnt = 0;
    bool updateForceLog;

    const float ETDF              = 1.50;                     // Execution Time Deviation Factor (experimental calculation)

    // Initialize variable for timing/derivative calculations
    unsigned long time_duration;
    double time_duration_double;
    double prev_delta_t = delta_t; 
    double new_delta_t;                                       // Delay time in [sec] - Here the initial value is given, will change inside loop
    unsigned long new_delta_t_micros;                         // Delay time in [micros]

    uint32_t StpPresentPosition = 0;
    bool updateDelayTime = true;                              // the delay time MUST BE CALCULATED in first do-while iteration

    // Variables for Real-time Stepper Position/Velocity calculation
    _prevAbsPos_rad = _currentAbsPos_rad;
    _prevAngVel_rps = _currentAngVel_rps;
    _prevAngAccel_rps2 = _currentAngAccel_rps2;

    // profile steps must be multiplied by 2!
    uint32_t profile_steps_new[PROF_STEPS_SIZE];
    for (size_t i = 0; i < PROF_STEPS_SIZE; i++)
    {
      profile_steps_new[i] = 2 * profile_steps[i]; 
    }
    
    _accel_count    = 0;                                      // must initialize each phase steps ctr based on steps calculated
    _ctVel_count    = 0;
    _decel_count    = -profile_steps_new[3];                         

    //Serial.print("Total steps to move = "); Serial.println(profile_steps_new[0]);
    //Serial.print("Accel steps to move = "); Serial.println(profile_steps_new[1]);
    //Serial.print("CtVel steps to move = "); Serial.println(profile_steps_new[2]);
    //Serial.print("Decel steps to move = "); Serial.println(profile_steps_new[3]);
    //delay(3000);

    // Stepping loop
    unsigned long motor_movement_start = millis();
    do
    {
    		// Inside this if every desired update() function must be called
        if (updateDelayTime == true)
        {
          // 1. Always update delay time when stepper has changed state!
          CustomStepperOvidiusShield::updateDelayTime(&new_delta_t, &prev_delta_t, StpPresentPosition, segmentExists, profile_steps_new, stp_error);
          //Serial.println("delay time updated");

          // 2. Updates force sensor values -> talks to HX711 using OvidiusSensors methods
          if (UPDATE_FORCE)
          {
            //Serial.println("MPHKA UPDATE FORCE");
            started_force_update = micros();
            updateForceMeasurements(ptr2ForceSensor, ptr2ForceSensorAxis, UPDATED_FORCE_MEASUREMENTS_KGS, updateForceLog, sensor_error);
            force_update_duration = micros() - started_force_update;
            //Serial.print("UPDATED_FORCE_MEASUREMENTS_KGS = "); Serial.println(*UPDATED_FORCE_MEASUREMENTS_KGS);
          }
          
          // 4. updates IMU values
          if (UPDATE_IMU)
          {
            //Serial.println("MPHKA UPDATE IMU");
            started_imu_update = micros();
            updateIMU(ptr2IMU, ptr2imu_packet, FILTER_SELECT, updateImuLog, sensor_error);
            imu_update_duration = micros() - started_imu_update;
            //Serial.print("imu_update_duration = "); Serial.println(imu_update_duration);
          }

          // 5. read current joint positions
          // updateStpAbsPos_rad(_currentAbsPos_rad, StpPresentPosition, stp_error); // stepper only
          getJointsAbsPosition_rad(CURRENT_CONFIGURATION_RAD, ptr2custom_dxl, ptr2_dxl_pp_pck, StpPresentPosition, updateJointPosLog, stp_error);
          
          // 6. read current joint velocities
          // updateStpAngVelStp(_currentAngVel_rps, new_delta_t); // stepper only
          getJointsAngVelocity_rs(CURRENT_JOINT_VELOCITY_RS, ptr2custom_dxl, ptr2_dxl_pv_pck, new_delta_t, updateJointVelLog ,  stp_error);
          
          // 7. read current motor amp

          // 8. data logs:
          // 8.1. Joint position
          if (updateJointPosLog)
          {
              joint_pos_log_cnt++;
              for (size_t i = 0; i < nDoF; i++)
              {
                ptr2logger->writeData(CURRENT_CONFIGURATION_RAD[i], millis(), joint_pos_log_cnt, (ptr2logfiles + POS_LOG_ID), sensor_error);
              }
          }
          // 8.2. Joint Velocity
          if (updateJointVelLog)
          {
              joint_vel_log_cnt++;
              for (size_t i = 0; i < nDoF; i++)
              {
                ptr2logger->writeData(CURRENT_JOINT_VELOCITY_RS[i], millis(), joint_vel_log_cnt, (ptr2logfiles + VEL_LOG_ID), sensor_error);
              }
          }
          // 8.3. Force
          if (updateForceLog)
          {
              force_log_cnt++;
              ptr2logger->writeData((double) *UPDATED_FORCE_MEASUREMENTS_KGS, millis(), force_log_cnt, (ptr2logfiles+ FOR_LOG_ID), sensor_error);
          }
          
          // 8.4. Current

          // 8.5. IMU       
        
        }
        new_delta_t_micros = (new_delta_t*1000000);

        // send step pulse unsigned long delayTime, long & StpPresentPosition, bool & updateDelayTime
        CustomStepperOvidiusShield::updateSingleStepVarDelay(new_delta_t_micros, &StpPresentPosition, &updateDelayTime);  // Updates Motor Step 
        
        //Serial.print("StpPresentPosition = "); Serial.println(StpPresentPosition);
        //Serial.print("force_update_duration = "); Serial.println(force_update_duration);
    }while(  (profile_steps_new[0] - StpPresentPosition) != 0 );

    time_duration = millis() - motor_movement_start;    // Calculates Stepper Motion Execution Time 
    time_duration_double = time_duration / 1000.0;
    Serial.print("Total execution time [sec] = "); Serial.println(time_duration_double,3);

    if ( time_duration_double <= ( ETDF * (*Texec)) )   // if synced motion was successful
    {
      *Texec = time_duration_double;
      *stp_error = 0;
    }
    else
    {
      *Texec = time_duration_double;
      *stp_error = 154;
    }

    if( (*stp_error == 0) && (*sensor_error == NO_ERROR) )
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
    
    // II . In fixed step only the number of relative steps to move was calculated. Here the function flowchart is different:
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

bool CustomStepperOvidiusShield::syncPreSetStepperGoalPositionVarStep2(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta,  volatile byte * currentDirStatus, bool * segment_exists, uint32_t * profile_steps,   int *stp_error)
{
  // FUN_CODE = __* used for stp_error assignment
  /*
   *  Written for p2pcsp2.ino. Based on syncPreSetStepperGoalPositionVarStep BUT here calculateProfVelAccel_preassignedVelTexec3 is used for (Vexec,Axec,Texec)
   */
    bool return_function_state;
    int ERROR_RETURNED;
    
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
 
    bool SEGMENT_EXISTS; 
    return_function_state = CustomStepperOvidiusShield::segmentExists_StpTrapzVelProfile2(currentAbsPos_double, goalAbsPos_double,  *Vexec, Aexec, *Texec, *Ta, &SEGMENT_EXISTS, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
      (*segment_exists) = (&SEGMENT_EXISTS);  // in order to access in main ino file
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

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

bool CustomStepperOvidiusShield::syncSetStepperGoalPositionVarStep2(tools::dataLogger *ptr2logger, File *ptr2logfiles, sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet,  sensors::imu_filter FILTER_SELECT, sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *force_error, double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, double * Texec,  volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, bool * segment_exists, bool UPDATE_FORCE, bool UPDATE_IMU, uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck,   int *stp_error)
{
  // FUN_CODE = 18* used for stp_error assignment

  /*
   * Same as syncSetStepperGoalPositionVarStep, but here execute_StpTrapzProfile2 is executed
   * This implements state machine logic for stepping!
   */
  //ptr2ForceSensor = ForceSensor;

  bool return_function_state;
  int ERROR_RETURNED;

  double c0 = CustomStepperOvidiusShield::calculateInitialStepDelay(Aexec);

  return_function_state = CustomStepperOvidiusShield::execute_StpTrapzProfile2(ptr2logger, ptr2logfiles, ptr2IMU, ptr2imu_packet, FILTER_SELECT,  ptr2ForceSensor, ptr2ForceSensorAxis, UPDATED_FORCE_MEASUREMENTS_KGS, force_error, profile_steps, segment_exists,  Texec,  c0, currentDirStatus,  UPDATE_FORCE,  UPDATE_IMU,ptr2custom_dxl, ptr2_dxl_pp_pck, ptr2_dxl_pv_pck, &ERROR_RETURNED);
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
    _currentAbsPos_rad = (*currentAbsPos_double);   // a private member to store abs position
    _currentAngVel_rps = 0;                         // a private member to store abs velocity
    _currentAngAccel_rps2 = 0;                      // a private member to store abs acceleration
    return true;
  }
  else
  {
    return false;
  }

}