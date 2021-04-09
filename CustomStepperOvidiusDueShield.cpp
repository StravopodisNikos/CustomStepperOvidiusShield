 /*
  * CustomStepperOvidiusDueShield.cpp - Library for controlling Steppers of the Ovidius Metamorphic Manipulator
  * Created by N.A. Stravopodis, March, 2021.
  * 
  * This Library is a simple copy-paste of the CustomStepperOvidiusShield.
  * It will be used to upgrade Ovidius Robot Microprocessor to DUE, because
  * of memory crashes during data logging on the MEGA board.
  * 
  *  [20-3-21] EEPROM Stepper functions are erased. Only slow homing is imple-
  *  mented(every time @ setup) and the variables read from EEPROM will be written
  *  as globals.
  * 
  *  Method changes:
  *  1. save_STP_EEPROM_settings - > init_stepper_globals(volatile byte & currentDirStatus, double & currentAbsPos_double, double & VelocityLimitStp, double & AccelerationLimitStp, double & MaxPosLimitStp);
  */

#include "Arduino.h"
#include "CustomStepperOvidiusDueShield.h"
#include <utility/StepperMotorSettings.h>
#include <utility/stepper_led_intervals.h>
#include <utility/stepper_led_indicators.h>
#include <utility/stepper_debug.h>

#include <OvidiusSensors.h>
#include <utility/OvidiusSensors_config.h>
#include <utility/OvidiusSensors_debug.h>

#include <Dynamixel2Arduino.h>
#include "DynamixelProPlusOvidiusShield.h"

#include <vector>

using namespace std;

enum ROT_DIR stepperDirRotation;

const double pi              = 3.14159265359;

volatile bool CustomStepperOvidiusDueShield::KILL_MOTION;

// Constructor
CustomStepperOvidiusDueShield::CustomStepperOvidiusDueShield(int stepID, int stepPin, int dirPin, int enblPin, int homeTriggerPin, int limitSwitchPin2, int limitSwitchPin3, int RED_LED_pin,int GREEN_LED_pin,int BLUE_LED_pin, int spr, int GEAR_FACTOR, int ft )
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enblPin, OUTPUT);
    pinMode(homeTriggerPin, INPUT_PULLUP);       // HOME TRIGGER SWITCH 
    pinMode(limitSwitchPin2, INPUT);             // LIMIT HALL SWITCH
    pinMode(limitSwitchPin3, INPUT);             // LIMIT HALL SWITCH

    //pinMode(RED_LED_pin, OUTPUT);
    //pinMode(GREEN_LED_pin, OUTPUT);
    //pinMode(BLUE_LED_pin, OUTPUT);

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

    _a              = (float) ( ( 2.0 * pi ) / (  _spr ) );               // Stepper Motor Step Angle(Angular Position of Motor shaft)[rad]
    _ag             = (float) ( ( 2.0 * pi ) / ( _GEAR_FACTOR * _spr ) ); // Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
    _accel_width    = (float) ( 1.0 / ACCEL_WIDTH_DENOM );                                 // Acceleration Phase width
    
    _accel_count    = 0;
    _ctVel_count    = 0;
    _decel_count    = 0; 

    _return_fn_state = false;

    // STATES INTERVALS
    _update_FORCE_interval     = UPDATE_FORCE_MEAS_INTERVAL;
    _update_joint_vel_interval = UPDATE_VEL_MEAS_INTERVAL;
    _update_joint_pos_interval = UPDATE_POS_MEAS_INTERVAL;
    _update_CURRENT_interval   = UPDATE_CURRENT_MEAS_INTERVAL;

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
    _last_CURRENT_update    = 0;
    _last_joint_pos_update  = 0;
    _last_joint_vel_update  = 0;
  }

// =========================================================================================================== //
//
//  P R I V A T E -- C L A S S -- M E M B E R S
//
// =========================================================================================================== //

void CustomStepperOvidiusDueShield::pauseForFixedTime_millis(unsigned long delayTime) {

    unsigned long time_now_micros = millis();
    while(millis() < time_now_micros + delayTime){}                   //wait approx. [μs]

    return;
} 

// =========================================================================================================== //

// singleStepVarDelay -> DEPRECATED
void CustomStepperOvidiusDueShield::singleStepVarDelay(unsigned long delayTime) {
  // Custom Stepping Function for Velocity-Acceleration Profiles

    unsigned long time_now_micros = micros();
    digitalWrite(_stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(_stepPin, LOW);
} // END function singleStepVarDelay

// =========================================================================================================== //

// updateSingleStepVarDelay
void CustomStepperOvidiusDueShield::updateSingleStepVarDelay(unsigned long delayTime, uint32_t * StpPresentPosition, bool * updateDelayTime) {
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

      *updateDelayTime = true;      // now that a step was executed, asks the processor to calculate the next delay time
    }
    else
    {
      *updateDelayTime = false;
    }
    
} 

// =========================================================================================================== //

void CustomStepperOvidiusDueShield::updateDelayTime(float * new_delayTime_sec, float * prev_delayTime_sec, uint32_t StpPresentPosition, bool * segment_exists, uint32_t * profile_steps, unsigned char *stp_error)
{
    // This function is executed only if updateSingleStepVarDelay sets the corresponding flag to true
    // New delay time calculated in [sec] is returned with "delayTime_sec"

    const float RAMP_FACTOR       = 1.0f;                        // Velocity Profile ramp slope factor

    // Initialize counters for Profile Execution
    float rest = 0;

    if((*segment_exists))                                                                                                     // Linear Segment exists
    {
      if(StpPresentPosition < profile_steps[1])
      {
        _accel_count++;                                                                                                       // Acceleration Phase: delta_t -> minimizes
        (*new_delayTime_sec) =  *prev_delayTime_sec - RAMP_FACTOR * ( (2.0f*(*prev_delayTime_sec)+rest)/(4.0f*_accel_count+1.0f) );     // c_n [sec]
      }
      else if( (StpPresentPosition > profile_steps[1]) && (StpPresentPosition < profile_steps[1]+profile_steps[2]) )          // Linear Segment: delta_t -> constant
      {
        _ctVel_count++;
        (*new_delayTime_sec) = (*prev_delayTime_sec);  
      }
      else
      {
        _decel_count++; 
        (*new_delayTime_sec) =  (*prev_delayTime_sec) - RAMP_FACTOR * ( (2.0f*(*prev_delayTime_sec)+rest)/(4.0f*_decel_count+1.0f) );    // Deceleration Phase: delta_t -> maximizes [sec] 
      }
    }
    else
{                                                                                                                               // Linear Segment doesn't exist
      if(StpPresentPosition < profile_steps[1])                                                                                 // Acceleration Phase: delta_t -> minimizes
      {
        _accel_count++;
        (*new_delayTime_sec) = (*prev_delayTime_sec) - RAMP_FACTOR * ( (2.0f*(*prev_delayTime_sec)+rest)/(4.0f*_accel_count+1.0f) );       // c_n [sec]
      }                                   
      else
    {                                                                                                                           // Deceleration Phase: delta_t -> maximizes
        _decel_count++;                                                                                                         // Negative Value!
        (*new_delayTime_sec) = (*prev_delayTime_sec) - RAMP_FACTOR * ( (2.0f*(*prev_delayTime_sec)+rest)/(4.0f*_decel_count+1.0f) );       // Deceleration Phase: delta_t -> maximizes [sec] 
      }
    }

  // [29-3-21] HUGE BUG => AFFECTS CT VELOCITY FACE!!!! => I AM TOTALLY NOOB => THIS SHOULB BE IMPLEMENTED ONLY IN execute function!
  //(*new_delayTime_sec) = (*new_delayTime_sec) / 2.0f ;       // Because half step pulse is generated each time!

  return;
}
// =========================================================================================================== //
void CustomStepperOvidiusDueShield::updateForceMeasurements(sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , bool & update_force , debug_error_type * force_error)
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
/*
void CustomStepperOvidiusDueShield::updateIMU(sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet, sensors::imu_filter FILTER_SELECT, bool &update_imu, debug_error_type * imu_error)
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
*/

// =========================================================================================================== //

// multiStepVarDelay
void CustomStepperOvidiusDueShield::multiStepVarDelay(unsigned long delayTime, uint32_t numSteps2Move) {
  // Custom Stepping Function for Velocity-Acceleration Profiles

  for (size_t i = 0; i < numSteps2Move; i++)
  {
    unsigned long time_now_micros = micros();
    digitalWrite(_stepPin, HIGH);
    while(micros() < time_now_micros + delayTime){}                   //wait approx. [μs]
    digitalWrite(_stepPin, LOW);
  }

} // END function multiStepVarDelay

void CustomStepperOvidiusDueShield::updateStpAngVelStp(float &current_ang_vel, float half_step_delay_sec)
{
  // this fn works only with state machine functions. Since in state machine
  // half pulses are generated for the calculated delay time, here wmega is 
  // extracted for half step info.

  current_ang_vel = ( float ) ( _ag / ( 2.0f * half_step_delay_sec ) );

  return;
}

void CustomStepperOvidiusDueShield::updateStpAbsPos_rad(float &realTimeStpAbsPos, uint32_t StpPresentPulse, unsigned char *stp_error)
{
  // StpPresentPulse is the counter of pulses for relative p2p stepper movement!
  // _prevAbsPos_rad MUST be initialized right before p2p execution loop!

  // first find dq from loop execution pulse counter
  float Delta_q_rad = convertStpPulses2Radian(StpPresentPulse, stp_error);

  realTimeStpAbsPos = _prevAbsPos_rad + (_sign_of_Dq * Delta_q_rad);

  return;
}

// =========================================================================================================== //

void CustomStepperOvidiusDueShield::getJointsAbsPosition_rad(float * ROBOT_ABS_POS_RAD, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, uint32_t StpPresentPulse, bool &update_joint_pos, unsigned char *error_code)
{
    // implemented for state machine only - not in ino file!
    // RETURN nDoFx1 array ROBOT_ABS_POS_RAD (~= currentConfiguration in ino file)
    // [27-3-21] BUG - since this function works with updateSingleStepVarDelay the step pulse must be changed to /2!

    uint32_t real_step_for_rad_convert;

    if ( millis() - _last_joint_pos_update > _update_joint_pos_interval )
    {
      _last_joint_pos_update = millis();

      // stepper
      real_step_for_rad_convert = StpPresentPulse / 2; // only locally 
      updateStpAbsPos_rad(_currentAbsPos_rad, real_step_for_rad_convert, error_code);
      ROBOT_ABS_POS_RAD[0] = _currentAbsPos_rad;
      // dxl
      // call dxl library function that reads position
      if ( ptr2custom_dxl->syncGetDynamixelsPresentPosition(ptr2_dxl_pp_pck->Dxl_ids, ptr2_dxl_pp_pck->Dxl_ids_size, ptr2_dxl_pp_pck->dxl_pp, ptr2_dxl_pp_pck->SR_pp, error_code, ptr2_dxl_pp_pck->dxl2ard_obj) )
      {
        // convert dxl pulses -> radians
        ROBOT_ABS_POS_RAD[1] = ptr2custom_dxl->convertDxlPulses2Radian(ptr2_dxl_pp_pck->dxl_pp[0]);
        ROBOT_ABS_POS_RAD[2] = ptr2custom_dxl->convertDxlPulses2Radian(ptr2_dxl_pp_pck->dxl_pp[1]);
        ROBOT_ABS_POS_RAD[3] = ptr2custom_dxl->convertDxlPulses2Radian(ptr2_dxl_pp_pck->dxl_pp[2]);

        update_joint_pos = true;
      }
      else
      {
          update_joint_pos = false;
          *error_code = syncGetPP_returned_error;
          return;        
      }

    }
    else
    {
      update_joint_pos = false;
    }
}

void CustomStepperOvidiusDueShield::getJointsAngVelocity_rs(float * ROBOT_ANG_VEL_RS, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PV_PACKET *ptr2_dxl_pv_pck, float half_step_delay_sec, bool &update_joint_vel, unsigned char *error_code)
{
    // implemented for state machine only - not in ino file!
    if ( millis() - _last_joint_vel_update > _update_joint_vel_interval )
    {

      _last_joint_vel_update = millis();

      // stepper
      updateStpAngVelStp(_currentAngVel_rps, half_step_delay_sec);
      ROBOT_ANG_VEL_RS[0] = _currentAngVel_rps;

      // dxl
      if( ptr2custom_dxl->syncGetDynamixelsPresentVelocity(ptr2_dxl_pv_pck->Dxl_ids,ptr2_dxl_pv_pck->Dxl_ids_size, ptr2_dxl_pv_pck->dxl_pv, ptr2_dxl_pv_pck->SR_pv, error_code, ptr2_dxl_pv_pck->dxl2ard_obj) )
      {
        // convert dxl vel units -> radians/sec
        ROBOT_ANG_VEL_RS[1] = ptr2custom_dxl->convertDxlVelUnits2RadPsec(ptr2_dxl_pv_pck->dxl_pv[0]);
        ROBOT_ANG_VEL_RS[2] = ptr2custom_dxl->convertDxlVelUnits2RadPsec(ptr2_dxl_pv_pck->dxl_pv[1]);
        ROBOT_ANG_VEL_RS[3] = ptr2custom_dxl->convertDxlVelUnits2RadPsec(ptr2_dxl_pv_pck->dxl_pv[2]);

        update_joint_vel = true;
      }
      else
      {
          update_joint_vel = false;
          //*error_code = syncGetPV_returned_error;
          return;
      }
    }
    else
    {
      update_joint_vel = false;
    }

    return;
    
} 

// =========================================================================================================== //
//void CustomStepperOvidiusDueShield::getJointsCurrent_mA(double * ROBOT_CUR_mA, sensors::currentSensor * ptr2CurrentSensor, sensors::current_packet * ptr2current_packet, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PC_PACKET *ptr2_dxl_pc_pck, bool & update_current, unsigned char *error_code )
void CustomStepperOvidiusDueShield::getJointsCurrent_A(float * ROBOT_CUR_A, sensors::currentSensor * ptr2CurrentSensor, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PC_PACKET *ptr2_dxl_pc_pck, bool & update_current, unsigned char *error_code )
{
    debug_error_type current_sensor_error_returned;
    float stepper_measured_current;

    if (millis() - _last_CURRENT_update > _update_CURRENT_interval)
    {
      // [24-3-21] stepper-Adafruit sensor -> DEPRECATED => CHANGED ARGUMENT OF FUNCTION CALL!
      //ptr2CurrentSensor->measureCurrent_mA(ptr2current_packet, &current_sensor_error_returned );
      //ROBOT_CUR_mA[0] = ptr2current_packet->current_measurement_mA;

      _last_CURRENT_update = millis();

      // dxl
      if ( ptr2custom_dxl->syncGetDynamixelsPresentCurrent(ptr2_dxl_pc_pck->Dxl_ids,ptr2_dxl_pc_pck->Dxl_ids_size,ptr2_dxl_pc_pck->dxl_pc,ptr2_dxl_pc_pck->SR_pc, error_code,ptr2_dxl_pc_pck->dxl2ard_obj) )
      {
        // convert mA-> A
        ROBOT_CUR_A[1] = (float) ( ptr2_dxl_pc_pck->dxl_pc[0] / 1000.0f );
        ROBOT_CUR_A[2] = (float) ( ptr2_dxl_pc_pck->dxl_pc[1] / 1000.0f );
        ROBOT_CUR_A[3] = (float) ( ptr2_dxl_pc_pck->dxl_pc[2] / 1000.0f );

        // [24-3-21] stepper-ACS712 current module
        // [30-3-21] After dxl syncGet because delay is used in measureCurrentACS712_A execution time ~1ms
        ptr2CurrentSensor->measureCurrentACS712_A(stepper_measured_current, &current_sensor_error_returned );
        ROBOT_CUR_A[0] = stepper_measured_current;
      }
      else
      {
        update_current = false; return;
      }


      if ( current_sensor_error_returned == NO_ERROR )
      {
        update_current = true;
      }
      else
      {
        update_current = false;
      }


    }
    else
    {
      update_current = false;
    }

    //update_current = true; // only for demo

    return;
}

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::getDynamixelsMotionState(DynamixelProPlusOvidiusShield *ptr2custom_dxl, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char *error_code )
{
  // Returns TRUE if (ALL dxls)->NO MOTION(FINISHED) or FALSE if->(even 1 dxl)->MOTION(STILL MOVING)
  int Dxls = ptr2_dxl_mov_pck->Dxl_ids_size;
  bool DXLS_FINISHED_MOTION = false;
  uint8_t DXL_MOVING[Dxls];

  if ( ptr2custom_dxl->syncGetDynamixelsMoving(ptr2_dxl_mov_pck->Dxl_ids, ptr2_dxl_mov_pck->Dxl_ids_size, ptr2_dxl_mov_pck->dxl_mov, ptr2_dxl_mov_pck->SR_mov, error_code, ptr2_dxl_mov_pck->dxl2ard_obj) )
  {
    DXL_MOVING[0] = ptr2_dxl_mov_pck->dxl_mov[0];
    DXL_MOVING[1] = ptr2_dxl_mov_pck->dxl_mov[1];
    DXL_MOVING[2] = ptr2_dxl_mov_pck->dxl_mov[2];
  }
  else
  {
    return false;
  }

  for (size_t i = 0; i < Dxls; i++)
  {
    if ( DXL_MOVING[i] == true ) // this dxl is still moving
    {
      DXLS_FINISHED_MOTION = false; break; 
    }
    else
    {
      DXLS_FINISHED_MOTION = true;
    }      
  }

  return DXLS_FINISHED_MOTION;
}


// =========================================================================================================== //
//
//  P U B L I C -- C L A S S -- M E M B E R S
//
// =========================================================================================================== //
void CustomStepperOvidiusDueShield::setStepperLed(const unsigned char *led_indicator)
{
    // FIRST TURN OFF PREVIOUS VAL
    analogWrite(_RED_LED_pin   , 0);  // write RED VALUE
    analogWrite(_GREEN_LED_pin , 0);  // write GREEN VALUE 
    analogWrite(_BLUE_LED_pin  , 0); // write BLUE VALUE 

    analogWrite(_RED_LED_pin   , led_indicator[0]);  // write RED VALUE
    analogWrite(_GREEN_LED_pin , led_indicator[1]);  // write GREEN VALUE 
    analogWrite(_BLUE_LED_pin  , led_indicator[2]); // write BLUE VALUE 
 
    return;        
}

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::readSessionFileLoggedData(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char * ptr2logfiles_names [LOG_FILES_DIR_CHAR_LEN], byte FILE_ID, char * data_buffer, int & buffer_size, int max_data_length, debug_error_type *sensor_error)
{
  // Sensor File ID is given by user and only the specified file data are returned

  if (FILE_ID > CUR_LOG_ID) // [30-3-21] max sensor id is that of the current sensor
  {
    *sensor_error = WRONG_FILE_ID_GIVEN;
    return false;
  }
  
  ptr2logger->openFile((ptr2logfiles + FILE_ID), ptr2logfiles_names[FILE_ID], O_CREAT | O_APPEND | O_WRITE, sensor_error);
  
  ptr2logger->readData(data_buffer, buffer_size, max_data_length, (ptr2logfiles + FILE_ID), sensor_error);
  
  ptr2logger->closeFile((ptr2logfiles + FILE_ID), sensor_error);

  if (* sensor_error == NO_ERROR)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::setStepperHomePositionSlow(float * currentAbsPos_double, volatile byte *currentDirStatus,  volatile bool *kill_motion_triggered,  unsigned char *stp_error){
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
        
        CustomStepperOvidiusDueShield::multiStepVarDelay(homing_stepping_delay, breakFreeSteps); // break free from magnetic field

        *kill_motion_triggered = false; // set the volatile value back to false
      }
      
      CustomStepperOvidiusDueShield::singleStepVarDelay(homing_stepping_delay);                  

      // Everything worked
      *stp_error = NO_STP_ERROR;
  }

  // sets global variable to new position(HOME) value
  *currentAbsPos_double = 0;
  // set private var to current abs pos in rad
  _currentAbsPos_rad    = *currentAbsPos_double;

if (*stp_error == NO_STP_ERROR)
{
  return true;
}
else
{
  return false;
}
} // END OF FUNCTION

// =========================================================================================================== //
/*
bool CustomStepperOvidiusDueShield::setStepperHomePositionFast(double * currentAbsPos_double, volatile byte * currentDirStatus, volatile bool * kill_motion_triggered,  unsigned char * stp_error)
{
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
  relative_steps_2_move = CustomStepperOvidiusDueShield::calculateRelativeSteps2Move(currentAbsPos_double, &goalAbsPos_double, stp_error);
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
  return_function_state = CustomStepperOvidiusDueShield::setStepperGoalDirection(currentAbsPos_double, &goalAbsPos_double, currentDirStatus);
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
          
      CustomStepperOvidiusDueShield::singleStepVarDelay(homing_stepping_delay);

      motor_step++;

      if (*kill_motion_triggered == true)
      {
        digitalWrite(_dirPin, *currentDirStatus); // change direction, since the currentDirStatus value was changed inside ISR function
        
        CustomStepperOvidiusDueShield::multiStepVarDelay(homing_stepping_delay, breakFreeSteps); // break free from magnetic field

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
*/
// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::setStepperGoalDirection(float * currentAbsPos_double, float * goalAbsPos_double, volatile byte * currentDirStatus){
  // FUN_CODE = 5 used for stp_error assignment

  // Calculate Delta_q1
  float Delta_q1 = *goalAbsPos_double - *currentAbsPos_double;

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

bool CustomStepperOvidiusDueShield::setStepperGoalPositionFixedStep(float * currentAbsPos_double, float * goalAbsPos_double, volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered,  unsigned char *stp_error)
{
  /*
   *  FUN_CODE = 6 used for stp_error assignment
   *  Executes sequence for joint-space operation
   */
  
  uint32_t relative_steps_2_move;
  bool return_function_state;

  // Determine Direction of Rotation
  return_function_state = CustomStepperOvidiusDueShield::setStepperGoalDirection(currentAbsPos_double, goalAbsPos_double, currentDirStatus);
  if (return_function_state)
  {
    *stp_error = 0;
  }
  else
  {
    *stp_error = 1;
  }
  
  // Calculate relative steps to move
  relative_steps_2_move = CustomStepperOvidiusDueShield::calculateRelativeSteps2Move(currentAbsPos_double, goalAbsPos_double, stp_error);
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
  return_function_state = CustomStepperOvidiusDueShield::moveStp2Position(&relative_steps_2_move, currentDirStatus, kill_motion_triggered, stp_error);
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

bool CustomStepperOvidiusDueShield::testP2Pparams_StpTrapzVelProfile(float * currentAbsPos_double, float * goalAbsPos_double, float * Vexec, float * Aexec, float * Texec, float *Ta, unsigned char *stp_error) {

  /*
   *  FUN_CODE = 7 used for stp_error assignment
   *  This fn assumes that a known fixed relation between V,A exists
   *  1. Accepts current&goal joint position, p2p execution time and computes V,A|exec
   *  2. If V,A|exec <= V,A|max then sets (Tsync=Texec) 
   *  3. If V,A|exec > V,A|max then sends error code that requests to increase Texec and aborts
   */
  float VelocityLimitStp      = 1.0;
  float AccelerationLimitStp  = 20.0;

  // Calculate 
  float Delta_q1 = *goalAbsPos_double - *currentAbsPos_double;	// [rad]

  // Type of trajectory: Trajectory with assigned duratins T,Ta-p.69 Melchiorri
	*Vexec = abs( Delta_q1 ) / ( (1 - _accel_width) * (*Texec) )  ;  // [rad/sec]
	*Aexec = abs( Delta_q1 ) / ( _accel_width * ( 1 - _accel_width) * pow((*Texec),2.0) ) ;  // [rad/sec^2]
  *Ta = _accel_width * (*Texec);

  // Read Vmax,Amax for Stepper from EEPROM
  //EEPROM.get(VL_JOINT1_STEPPER_EEPROM_ADDR, *VelocityLimitStp);    			

	//EEPROM.get(AL_JOINT1_STEPPER_EEPROM_ADDR, *AccelerationLimitStp);  

  if ( ( (*Vexec) <= (VelocityLimitStp) ) && ( (*Aexec) <= (AccelerationLimitStp) ))
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

bool CustomStepperOvidiusDueShield::moveStp2Position(uint32_t * relative_steps_2_move, volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, unsigned char *stp_error)
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
    CustomStepperOvidiusDueShield::singleStepVarDelay(fixed_stepping_delay);
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
      CustomStepperOvidiusDueShield::singleStepVarDelay(fixed_stepping_delay);
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

uint32_t CustomStepperOvidiusDueShield::calculateRelativeSteps2Move(float * currentAbsPos_double, float * goalAbsPos_double, unsigned char * stp_error)
{
    // FUN_CODE = 9 used for stp_error assignment

    float Delta_q1_double = abs( (*goalAbsPos_double) - (*currentAbsPos_double) );

    unsigned char ERROR_RETURNED;

    uint32_t relative_steps_2_move =  CustomStepperOvidiusDueShield::convertRadian2StpPulses(Delta_q1_double, &ERROR_RETURNED);

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

uint32_t CustomStepperOvidiusDueShield::convertRadian2StpPulses(float position_in_radians, unsigned char *stp_error)
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
        position_in_stp_pulses = (uint32_t) ( (abs(position_in_radians) * _GEAR_FACTOR * _spr) / (2*pi) );
        //position_in_stp_pulses = (uint32_t) abs(position_in_radians) / _ag;
        *stp_error = 102;
    }

return position_in_stp_pulses;
}
  
// =========================================================================================================== //

float CustomStepperOvidiusDueShield::convertStpPulses2Radian(uint32_t position_in_stp_pulses, unsigned char *stp_error)
{
  // FUN_CODE = 11* used for stp_error assignment
    float position_in_radians;
    
    if (position_in_stp_pulses == 0)
    {
        position_in_radians = 0;
        *stp_error = 111;
    }
    else
    {   // _ag = ( 2 * pi ) / ( _GEAR_FACTOR * _spr )
        position_in_radians = (float) ( (position_in_stp_pulses * 2 * pi) / (_spr * GEAR_FACTOR_PLANETARY) );
        //position_in_radians = (double) (position_in_stp_pulses) * _ag;
        *stp_error = 112;
    }
    
    return position_in_radians;
}

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::segmentExists_StpTrapzVelProfile(float * currentAbsPos_double, float * goalAbsPos_double,  float & Vexec, float * Aexec, float & Texec, float & Ta,  bool * segmentExists, unsigned char *stp_error){

  // FUN_CODE = 12* used for stp_error assignment
  /*
   *  Given Vexec,Aexec checks if linear segment exists
   */

  float Delta_q1_double = *goalAbsPos_double - *currentAbsPos_double;	// [rad]

  float h_cond = pow((Vexec),2.0) / (*Aexec) ;				        // Condition factor for linear segment existence in Trapezoidal Velocity Profile

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

bool CustomStepperOvidiusDueShield::segmentExists_StpTrapzVelProfile2(float * currentAbsPos_double, float * goalAbsPos_double,  float & Vexec, float * Aexec, float & Texec, float & Ta,  bool * segmentExists, unsigned char *stp_error){

  // FUN_CODE = __ used for stp_error assignment
  /*
   *  Based on segmentExists_StpTrapzVelProfile2 BUT here calculateProfVelAccel_preassignedVelTexec3 was used in p2pcsp2.ino file
   */

  float Delta_q1_double = abs( *goalAbsPos_double - *currentAbsPos_double );	// [rad]

  float h_cond = abs( pow((Vexec),2.0) / (*Aexec) );				        // Condition factor for linear segment existence in Trapezoidal Velocity Profile

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

float CustomStepperOvidiusDueShield::calculateInitialStepDelay(float * Aexec)
{
  // FUN_CODE = 13* used for stp_error assignment
  // Determine initial step delay time for real-time profile generation: c0 with ignored inaccuracy factor [sec]

  float sqrt_for_c0_calc =  (2.0f * _ag) /  (*Aexec);
  float c0              = (float) ( 0.676f * _ft * pow(10.0f,-6.0f) * sqrt(sqrt_for_c0_calc) );		// [29-3-21] Added INIT_STEP_DELAY_FACTOR for test	-> REMOVED!				                  

return c0;
}
// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::returnSteps_StpTrapzVelProfile(float * currentAbsPos_double, float * goalAbsPos_double, float & Vexec, float * Aexec, float * Texec, float * Ta,  bool * segmentExists, unsigned char * stp_error, uint32_t *profile_steps)
{
  // FUN_CODE = 14* used for stp_error assignment
  /*
   * RETURNS THE STEPS OF EACH P2P MOTION SEGMENT in an uint32_t array: profile_steps[] = {relative_steps_2_move, nmov_Ta, nmov_linseg, nmov_Td};
   */

    float Delta_q1_double = (*goalAbsPos_double) - (*currentAbsPos_double);

    unsigned char ERROR_RETURNED;
    uint32_t relative_steps_2_move;
    float Delta_q1_accel_double;
    float Delta_q1_lin_seg_double;
    uint32_t nmov_Ta,nmov_Td,nmov_linseg;

    // first calculate the relative steps of all motion
    relative_steps_2_move = CustomStepperOvidiusDueShield::calculateRelativeSteps2Move(currentAbsPos_double, goalAbsPos_double, stp_error);
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

        nmov_Ta     = CustomStepperOvidiusDueShield::convertRadian2StpPulses(Delta_q1_accel_double, &ERROR_RETURNED);        // Steps of Acceleration Phase; 
        nmov_linseg = relative_steps_2_move - 2 *  nmov_Ta;                                                             
        nmov_Td     = nmov_Ta;
        *stp_error = ERROR_RETURNED;
    }
    else
    {
        // In this case: accel_width changes value! Vmax is recalculated! Texec is recalculated!
        Delta_q1_accel_double  = 0.5 * (*Aexec) * pow((*Ta),2.0);

        nmov_Ta      = CustomStepperOvidiusDueShield::convertRadian2StpPulses(Delta_q1_accel_double, &ERROR_RETURNED);        // Steps of Acceleration Phase;
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
/*
bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile(uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, unsigned char * stp_error)
{
	
  //  FUN_CODE = 15* used for stp_error assignment
	//  INPUT: segmentExists, profile_steps = {relative_steps_2_move, nmov_Ta, nmov_linseg, nmov_Td}, 
  //         delta_t = c0(initial step delay), Texec=Motion Execution Time, delta_t = Initian step delay
	//

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

        CustomStepperOvidiusDueShield::singleStepVarDelay(new_delta_t_micros);                           // Executes Motor Step 

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
*/
// =========================================================================================================== //
/*
//bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile2(sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS ,debug_error_type *force_error, uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, bool UPDATE_FORCE, bool UPDATE_IMU, int * stp_error)
bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile2(tools::dataLogger *ptr2logger, File *ptr2logfiles, char *ptr2logfiles_names,  sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet,  sensors::imu_filter FILTER_SELECT,  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, bool UPDATE_FORCE, bool UPDATE_IMU, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, unsigned char * stp_error)
{
    // Log Files: 0->position, 1->velocity, 2->force, 3->current, 4->imu

    // Array for real time position/velocity
    double CURRENT_CONFIGURATION_RAD[nDoF];
    double CURRENT_JOINT_VELOCITY_RS[nDoF];
    unsigned long joint_pos_log_cnt = 1;    // single zero line was written in file creation!
    unsigned long joint_vel_log_cnt = 1;    // single zero line was written in file creation!
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
    unsigned long force_log_cnt = 1;        // single zero line was written in file creation!
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
          CustomStepperOvidiusDueShield::updateDelayTime(&new_delta_t, &prev_delta_t, StpPresentPosition, segmentExists, profile_steps_new, stp_error);
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
              ptr2logger->openFile((ptr2logfiles + POS_LOG_ID), ptr2logfiles_names[POS_LOG_ID], FILE_WRITE, sensor_error);
              for (size_t i = 0; i < nDoF; i++)
              {
                ptr2logger->writeData(CURRENT_CONFIGURATION_RAD[i], millis(), joint_pos_log_cnt, (ptr2logfiles + POS_LOG_ID), sensor_error);
              }
              ptr2logger->closeFile((ptr2logfiles + POS_LOG_ID), sensor_error);
          }
          // 8.2. Joint Velocity
          if (updateJointVelLog)
          {
              joint_vel_log_cnt++;
              ptr2logger->openFile((ptr2logfiles + VEL_LOG_ID), ptr2logfiles_names[VEL_LOG_ID], FILE_WRITE, sensor_error);
              for (size_t i = 0; i < nDoF; i++)
              {
                ptr2logger->writeData(CURRENT_JOINT_VELOCITY_RS[i], millis(), joint_vel_log_cnt, (ptr2logfiles + VEL_LOG_ID), sensor_error);
              }
              ptr2logger->closeFile((ptr2logfiles + VEL_LOG_ID), sensor_error);
          }
          // 8.3. Force
          if (updateForceLog)
          {
              force_log_cnt++;
              ptr2logger->openFile((ptr2logfiles + FOR_LOG_ID), ptr2logfiles_names[FOR_LOG_ID], FILE_WRITE, sensor_error);
              ptr2logger->writeData((double) *UPDATED_FORCE_MEASUREMENTS_KGS, millis(), force_log_cnt, (ptr2logfiles+ FOR_LOG_ID), sensor_error);
              ptr2logger->closeFile((ptr2logfiles + FOR_LOG_ID), sensor_error);
          }
          
          // 8.4. Current

          // 8.5. IMU       
        
        }
        new_delta_t_micros = (new_delta_t*1000000);

        // send step pulse unsigned long delayTime, long & StpPresentPosition, bool & updateDelayTime
        CustomStepperOvidiusDueShield::updateSingleStepVarDelay(new_delta_t_micros, &StpPresentPosition, &updateDelayTime);  // Updates Motor Step 
        
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
*/
// =========================================================================================================== //

//bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile3(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char *ptr2logfiles_names,  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, sensors::currentSensor * ptr2CurrentSensor, sensors::current_packet * ptr2current_packet, debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists, double delta_t, bool UPDATE_FORCE, bool UPDATE_CURRENT, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char * stp_error)
//bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile3(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char * ptr2logfiles_names,  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists, double delta_t, bool UPDATE_FORCE, bool UPDATE_CURRENT, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char * stp_error)
bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile3(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char * ptr2logfiles_names [LOG_FILES_DIR_CHAR_LEN],  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists, float delta_t, bool UPDATE_SENSOR[] , DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char * stp_error)
{
  // [24-3-21] Removed  sensors::current_packet * ptr2current_packet because Adafruit INA219 is not used
  // [25-3-21] Bug! Filenames must be given in a 2D array
  // [26-3-21] Only stepper movement will be considered. the evaluation of synced motion will be done in the corresponding ino file.
    KILL_MOTION = false;
    // SAME AS execute_StpTrapzProfile2 BUT NO IMU HERE!
    //bool DYNAMIXELS_STOPED_MOVING = false;  // assumes that sync set goal position command to dxl was sent before this function call!!!
    bool STEPPER_REACHED_GOAL_POS = false;  // logical, it hasn't even started!
    //bool SYNC_MOTION_FINISHED     = false;

    // Vars for real time position/velocity
    float CURRENT_CONFIGURATION_RAD[nDoF];
    float CURRENT_JOINT_VELOCITY_RS[nDoF];
    unsigned long joint_pos_log_cnt = 0;    // 1 => single zero line was written in file creation! 0 => nothing was written @file creation
    unsigned long joint_vel_log_cnt = 0;    // single zero line was written in file creation!
    bool updateJointPosLog = false;
    bool updateJointVelLog = false;
    // Vars for real time current measurement
    float CURRENT_MEASUREMENTS_mA[nDoF];
    unsigned long joint_cur_log_cnt = 0;    // single zero line was written in file creation!
    bool updateJointCurLog = false;

    // Vars for force measurements
    //ptr2ForceSensor = ForceSensor;
    float * UPDATED_FORCE_MEASUREMENT_KGS;   // [21-3-21] single force measurement Zaxis 
    unsigned long force_update_duration;
    unsigned long started_force_update;
    unsigned long force_log_cnt = 0;        // single zero line was written in file creation!
    bool updateForceLog = false;

    // Initialize variable for timing/derivative calculations
    float prev_delta_t = delta_t;                            // delta_t <=> c0
    float new_delta_t  = prev_delta_t;                       // Delay time in [sec] - Here the initial value is given, will change inside loop
    unsigned long new_delta_t_micros;                         // Delay time in [micros]

    uint32_t StpPresentPosition = 0;
    bool updateStepDelayTime = false;                         // the delay time MUST NOT BE CALCULATED in first do-while iteration

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

    // Stepping loop
    unsigned long motor_movement_start = millis();
    do
    {
      // Inside this if every desired update() function must be called
      if (updateStepDelayTime)
      {
        setStepperLed(stepper_turn_off_led);

        // 1. Always update delay time when stepper has changed state!
        CustomStepperOvidiusDueShield::updateDelayTime(&new_delta_t, &prev_delta_t, StpPresentPosition, segmentExists, profile_steps_new, stp_error);
        
        // prev_delta_t = new_delta_t;                   // the new_delta_t hasn't yet implemented but this is the value that will return as previous...

      }
      else
      {
        setStepperLed(stepper_stepping_in_progress);         // Olive

        // HERE until next step pin state change we have time to calculate real time data...
        // CAUTION -> NOTHING is ALWAYS executed here! Here we have time to check if it is time 
        // to update the real time data, and if and only if time has come, we calculate/read!

        // 2. Updates force sensor values -> talks to HX711 using OvidiusSensors methods
        if (UPDATE_SENSOR[FOR_LOG_ID])
        {
          //started_force_update = micros();
          updateForceMeasurements(ptr2ForceSensor, ptr2ForceSensorAxis, UPDATED_FORCE_MEASUREMENT_KGS, updateForceLog, sensor_error);
          //force_update_duration = micros() - started_force_update;
        }

        // 3. read current joint positions
        if (UPDATE_SENSOR[POS_LOG_ID])
        {
          getJointsAbsPosition_rad(CURRENT_CONFIGURATION_RAD, ptr2custom_dxl, ptr2_dxl_pp_pck, StpPresentPosition, updateJointPosLog, stp_error);
        }

        // 4. read current joint velocities
        if (UPDATE_SENSOR[VEL_LOG_ID])
        {
          getJointsAngVelocity_rs(CURRENT_JOINT_VELOCITY_RS, ptr2custom_dxl, ptr2_dxl_pv_pck, new_delta_t, updateJointVelLog ,  stp_error);
        }

        // 5. read current of joints
        if (UPDATE_SENSOR[CUR_LOG_ID])
        {
          //getJointsCurrent_mA(CURRENT_MEASUREMENTS_mA, ptr2CurrentSensor, ptr2current_packet,ptr2custom_dxl,ptr2_dxl_pc_pck, updateJointCurLog, stp_error  ); -> for INA219b only!
          getJointsCurrent_A(CURRENT_MEASUREMENTS_mA, ptr2CurrentSensor, ptr2custom_dxl, ptr2_dxl_pc_pck, updateJointCurLog, stp_error  );
        }

        // 6. check what dynamixels are doing(moving=1,finished=0)

        // 7. data logs:
        //if ( (updateJointPosLog) && (*stp_error != NO_ERROR) )
        if ( updateJointPosLog )
        {
            // [30-3-21] Makes sure the dynamixel packet is received and no delay is used!
            // pauseForFixedTime_millis(PAUSE_FOR_DXL_COM_MILLIS); -> used extra if and removed the bad delay!
            // Since: updateJointPosLog->true => _last_joint_pos_update was updated just before sync read command
            if ( millis() - _last_joint_pos_update > PAUSE_FOR_DXL_COM_MILLIS )
            {
              joint_pos_log_cnt++;
              ptr2logger->openFile((ptr2logfiles + POS_LOG_ID), ptr2logfiles_names[POS_LOG_ID] , O_CREAT | O_APPEND | O_WRITE, sensor_error);
              ptr2logger->writeData(CURRENT_CONFIGURATION_RAD, nDoF,  millis(), joint_pos_log_cnt, (ptr2logfiles + POS_LOG_ID), sensor_error);
              ptr2logger->closeFile((ptr2logfiles + POS_LOG_ID), sensor_error);
            }
        }
        if  ( updateJointVelLog ) 
        {
            if ( millis() - _last_joint_vel_update > PAUSE_FOR_DXL_COM_MILLIS )
            {
              joint_vel_log_cnt++;
              ptr2logger->openFile((ptr2logfiles + VEL_LOG_ID), ptr2logfiles_names[VEL_LOG_ID], O_CREAT | O_APPEND | O_WRITE, sensor_error);
              ptr2logger->writeData(CURRENT_JOINT_VELOCITY_RS, nDoF, millis(), joint_vel_log_cnt, (ptr2logfiles + VEL_LOG_ID), sensor_error);
              ptr2logger->closeFile((ptr2logfiles + VEL_LOG_ID), sensor_error);
            }
        }
        if ( updateForceLog ) 
        {
            if ( millis() - _last_FORCE_update > PAUSE_FOR_DXL_COM_MILLIS )
            {
              force_log_cnt++;
              ptr2logger->openFile((ptr2logfiles + FOR_LOG_ID), ptr2logfiles_names[FOR_LOG_ID], O_CREAT | O_APPEND | O_WRITE, sensor_error);
              ptr2logger->writeData(UPDATED_FORCE_MEASUREMENT_KGS, 1, millis(), force_log_cnt, (ptr2logfiles+ FOR_LOG_ID), sensor_error);
              ptr2logger->closeFile((ptr2logfiles + FOR_LOG_ID), sensor_error);
            }
        }
        if ( updateJointCurLog ) 
        {
            if ( millis() - _last_CURRENT_update > PAUSE_FOR_DXL_COM_MILLIS )
            {          
              joint_cur_log_cnt++;
              ptr2logger->openFile((ptr2logfiles + CUR_LOG_ID), ptr2logfiles_names[CUR_LOG_ID], O_CREAT | O_APPEND | O_WRITE, sensor_error);
              ptr2logger->writeData(CURRENT_MEASUREMENTS_mA, nDoF, millis(), joint_cur_log_cnt, (ptr2logfiles + CUR_LOG_ID), sensor_error);
              ptr2logger->closeFile((ptr2logfiles + CUR_LOG_ID), sensor_error);
            }              
        }   

      }

      // Convert the value to micros for updateSingleStepVarDelay and check if is feasible in current stepper motor driver
      // [29-3-21] /2 is introduced in order to fix HUGE bug found.
      new_delta_t_micros = (unsigned long) ( (new_delta_t / 2.0f) * 1000000) ;   // [DEL_TIME_SEC] * [SEC2MICROS]
      if (new_delta_t_micros < MIN_STEP_DELAY_MICROS)
      {
        new_delta_t_micros = MIN_STEP_DELAY_MICROS;
      }
      prev_delta_t = (new_delta_t / 2.0f);  // the modified new_delta_t will next implemented and this will return as previous...
      // Send step pulse unsigned long delayTime, long & StpPresentPosition, bool & updateDelayTime
      CustomStepperOvidiusDueShield::updateSingleStepVarDelay(new_delta_t_micros, &StpPresentPosition, &updateStepDelayTime);  // Updates Motor Step 

      // CHECK IF STEPPER FINISHED
      if ( (profile_steps_new[0] - StpPresentPosition) == 0)
      {
        STEPPER_REACHED_GOAL_POS = true;
        updateStepDelayTime = false;
      }
      else
      {
        STEPPER_REACHED_GOAL_POS = false;
      }

      // CHECK IF STEPPER HIT THE LIMITS
      if (KILL_MOTION)
      {
        STEPPER_REACHED_GOAL_POS = true; // terminates the stepping loop
        *stp_error = REACHED_JOINT1_LIMIT;
      }

    }while(  !STEPPER_REACHED_GOAL_POS ); // while loop takes into consideration ONLY the Stepper!

    //if ( STEPPER_REACHED_GOAL_POS && (!KILL_MOTION)  ) // [29-3-21] returns true only if stepper reached target without hitting the limits!
    if ( STEPPER_REACHED_GOAL_POS )
    {
      KILL_MOTION = false;     // change the volatile emergency variable in order to continue safe operation
      return true;
    }
    else
    {
      return false;
    }

} // END OF FUNCTION

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile4(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char * ptr2logfiles_names [LOG_FILES_DIR_CHAR_LEN],  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists, float delta_t, float * GOAL_CONFIGURATION, bool UPDATE_SENSOR[] , DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char * stp_error)
{
    // [30-3-21] SAME AS execute_StpTrapzProfile3, but here even if stepper finishes continues to check for dxls moving and logs data!
    KILL_MOTION = false;
    bool DXLS_REACHED_GOAL_POS    = false;  // assumes that sync set goal position command to dxl was sent before this function call!!!
    bool STEPPER_REACHED_GOAL_POS = false;  // logical, it hasn't even started!
    bool SYNC_MOTION_FINISHED     = false;

    // Vars for real time position/velocity
    float CURRENT_CONFIGURATION_RAD[nDoF];
    float CURRENT_JOINT_VELOCITY_RS[nDoF];
    unsigned long joint_pos_log_cnt = 0;    // 1 => single zero line was written in file creation! 0 => nothing was written @file creation
    unsigned long joint_vel_log_cnt = 0;    // single zero line was written in file creation!
    bool updateJointPosLog = false;
    bool updateJointVelLog = false;
    // Vars for real time current measurement
    float CURRENT_MEASUREMENTS_mA[nDoF];
    unsigned long joint_cur_log_cnt = 0;    // single zero line was written in file creation!
    bool updateJointCurLog = false;

    // Vars for force measurements
    //ptr2ForceSensor = ForceSensor;
    float * UPDATED_FORCE_MEASUREMENT_KGS;   // [21-3-21] single force measurement Zaxis 
    unsigned long force_update_duration;
    unsigned long started_force_update;
    unsigned long force_log_cnt = 0;        // single zero line was written in file creation!
    bool updateForceLog = false;

    // Initialize variable for timing/derivative calculations
    float prev_delta_t = delta_t;                            // delta_t <=> c0
    float new_delta_t  = prev_delta_t;                       // Delay time in [sec] - Here the initial value is given, will change inside loop
    unsigned long new_delta_t_micros;                         // Delay time in [micros]

    uint32_t StpPresentPosition = 0;
    bool updateStepDelayTime = false;                         // the delay time MUST NOT BE CALCULATED in first do-while iteration

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

    // Stepping loop
    unsigned long motor_movement_start = millis();
    do
    {
        // Inside this if every desired update() function must be called
        if (updateStepDelayTime)
        {
          setStepperLed(stepper_turn_off_led);

          // 1. Always update delay time when stepper has changed state!
          CustomStepperOvidiusDueShield::updateDelayTime(&new_delta_t, &prev_delta_t, StpPresentPosition, segmentExists, profile_steps_new, stp_error);
          
        }
        else
        {
          setStepperLed(stepper_stepping_in_progress);         // Olive

          // HERE until next step pin state change we have time to calculate real time data...
          // CAUTION -> NOTHING is ALWAYS executed here! Here we have time to check if it is time 
          // to update the real time data, and if and only if time has come, we calculate/read!

          // 2. Updates force sensor values -> talks to HX711 using OvidiusSensors methods
          if (UPDATE_SENSOR[FOR_LOG_ID])
          {
            //started_force_update = micros();
            updateForceMeasurements(ptr2ForceSensor, ptr2ForceSensorAxis, UPDATED_FORCE_MEASUREMENT_KGS, updateForceLog, sensor_error);
            //force_update_duration = micros() - started_force_update;
          }

          // 3. read current joint positions
          if (UPDATE_SENSOR[POS_LOG_ID])
          {
            getJointsAbsPosition_rad(CURRENT_CONFIGURATION_RAD, ptr2custom_dxl, ptr2_dxl_pp_pck, StpPresentPosition, updateJointPosLog, stp_error);
          }

          // 4. read current joint velocities
          if (UPDATE_SENSOR[VEL_LOG_ID])
          {
            getJointsAngVelocity_rs(CURRENT_JOINT_VELOCITY_RS, ptr2custom_dxl, ptr2_dxl_pv_pck, (new_delta_t / 2.0f), updateJointVelLog ,  stp_error);
          }

          // 5. read current of joints
          if (UPDATE_SENSOR[CUR_LOG_ID])
          {
            //getJointsCurrent_mA(CURRENT_MEASUREMENTS_mA, ptr2CurrentSensor, ptr2current_packet,ptr2custom_dxl,ptr2_dxl_pc_pck, updateJointCurLog, stp_error  ); -> for INA219b only!
            getJointsCurrent_A(CURRENT_MEASUREMENTS_mA, ptr2CurrentSensor, ptr2custom_dxl, ptr2_dxl_pc_pck, updateJointCurLog, stp_error  );
          }

          // 6. check what dynamixels are doing(moving=1,finished=0)

          // 7. data logs:
          //if ( (updateJointPosLog) && (*stp_error != NO_ERROR) )
          if ( updateJointPosLog )
          {
              // [30-3-21] Makes sure the dynamixel packet is received and no delay is used!
               pauseForFixedTime_millis(PAUSE_FOR_DXL_COM_MILLIS); //-> used extra if and removed the bad delay!
              // Since: updateJointPosLog->true => _last_joint_pos_update was updated just before sync read command
              //if ( millis() - _last_joint_pos_update > PAUSE_FOR_DXL_COM_MILLIS )
              //{
                joint_pos_log_cnt++;
                ptr2logger->openFile((ptr2logfiles + POS_LOG_ID), ptr2logfiles_names[POS_LOG_ID] , O_CREAT | O_APPEND | O_WRITE, sensor_error);
                ptr2logger->writeData(CURRENT_CONFIGURATION_RAD, nDoF,  millis(), joint_pos_log_cnt, (ptr2logfiles + POS_LOG_ID), sensor_error);
                ptr2logger->closeFile((ptr2logfiles + POS_LOG_ID), sensor_error);
              //}
          }
          if  ( updateJointVelLog ) 
          {
              //if ( millis() - _last_joint_vel_update > PAUSE_FOR_DXL_COM_MILLIS )
              //{
                joint_vel_log_cnt++;
                ptr2logger->openFile((ptr2logfiles + VEL_LOG_ID), ptr2logfiles_names[VEL_LOG_ID], O_CREAT | O_APPEND | O_WRITE, sensor_error);
                ptr2logger->writeData(CURRENT_JOINT_VELOCITY_RS, nDoF, millis(), joint_vel_log_cnt, (ptr2logfiles + VEL_LOG_ID), sensor_error);
                ptr2logger->closeFile((ptr2logfiles + VEL_LOG_ID), sensor_error);
              //}
          }
          if ( updateForceLog ) 
          {
              //if ( millis() - _last_FORCE_update > PAUSE_FOR_DXL_COM_MILLIS )
              //{
                force_log_cnt++;
                ptr2logger->openFile((ptr2logfiles + FOR_LOG_ID), ptr2logfiles_names[FOR_LOG_ID], O_CREAT | O_APPEND | O_WRITE, sensor_error);
                ptr2logger->writeData(UPDATED_FORCE_MEASUREMENT_KGS, 1, millis(), force_log_cnt, (ptr2logfiles+ FOR_LOG_ID), sensor_error);
                ptr2logger->closeFile((ptr2logfiles + FOR_LOG_ID), sensor_error);
              //}
          }
          if ( updateJointCurLog ) 
          {
               pauseForFixedTime_millis(PAUSE_FOR_DXL_COM_MILLIS);
              //if ( millis() - _last_CURRENT_update > PAUSE_FOR_DXL_COM_MILLIS )
              //{          
                joint_cur_log_cnt++;
                ptr2logger->openFile((ptr2logfiles + CUR_LOG_ID), ptr2logfiles_names[CUR_LOG_ID], O_CREAT | O_APPEND | O_WRITE, sensor_error);
                ptr2logger->writeData(CURRENT_MEASUREMENTS_mA, nDoF, millis(), joint_cur_log_cnt, (ptr2logfiles + CUR_LOG_ID), sensor_error);
                ptr2logger->closeFile((ptr2logfiles + CUR_LOG_ID), sensor_error);
              //}              
          }   

        }

        // the stepping is executed only if stepper hasn't reached target position
        if (!STEPPER_REACHED_GOAL_POS)
        {
          new_delta_t_micros = (unsigned long) ( (new_delta_t / 2.0f) * 1000000) ;   // [DEL_TIME_SEC] * [SEC2MICROS]
          prev_delta_t = (new_delta_t / 2.0f);  // the modified new_delta_t will next implemented and this will return as previous...

          if (new_delta_t_micros < MIN_STEP_DELAY_MICROS)
          {
            new_delta_t_micros = MIN_STEP_DELAY_MICROS;
            prev_delta_t = 0.000025f;
          }

          // Send step pulse unsigned long delayTime, long & StpPresentPosition, bool & updateDelayTime
          CustomStepperOvidiusDueShield::updateSingleStepVarDelay(new_delta_t_micros, &StpPresentPosition, &updateStepDelayTime);  // Updates Motor Step 
        }
        //else
        //{
          // requests to step updating delay time, so now only the #else Ln1704 is executed
        //  updateStepDelayTime = false;
        //  StpPresentPosition  = profile_steps_new[0]; // redundant, just to make sure StpPresentPosition will never change value  
        //}

        // CHECK IF STEPPER FINISHED
        if ( (profile_steps_new[0] - StpPresentPosition) == 0)
        {
          STEPPER_REACHED_GOAL_POS = true;
          updateStepDelayTime = false;
        }
        else
        {
          STEPPER_REACHED_GOAL_POS = false;
        }

        // CHECK IF DYNAMIXELS FINISHED: CURRENT_CONFIGURATION_RAD WILL CONTINUE TO CHANGE SINCE READING FROM DXL DOESN'T STOP!
        if( ( fabs(CURRENT_CONFIGURATION_RAD[1] - GOAL_CONFIGURATION[1]) < ANG_POS_THRESHOLD) && ( fabs(CURRENT_CONFIGURATION_RAD[2] - GOAL_CONFIGURATION[2]) < ANG_POS_THRESHOLD) && ( fabs(CURRENT_CONFIGURATION_RAD[3] - GOAL_CONFIGURATION[3]) < ANG_POS_THRESHOLD) )
        {
          DXLS_REACHED_GOAL_POS = true;
        }
        else
        {
          DXLS_REACHED_GOAL_POS = false;
        }

        if (STEPPER_REACHED_GOAL_POS && DXLS_REACHED_GOAL_POS)
        {
          SYNC_MOTION_FINISHED = true;
        }

      
      // CHECK IF STEPPER HIT THE LIMITS
      if (KILL_MOTION)
      {
        SYNC_MOTION_FINISHED = true; // terminates the stepping loop
        *stp_error = REACHED_JOINT1_LIMIT;
      }

    }while( !SYNC_MOTION_FINISHED  ); // while loop takes into consideration ONLY the Stepper!

    if ( SYNC_MOTION_FINISHED )
    {
      KILL_MOTION = false;     // change the volatile emergency variable in order to continue safe operation
      return true;
    }
    else
    {
      return false;
    }

} // END OF FUNCTION

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::execute_StpTrapzProfile5(tools::dataLogger *ptr2logger, tools::real_time_log_struct * ptr2log_structs,  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists, float delta_t, float * GOAL_CONFIGURATION, bool UPDATE_SENSOR[] , DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char * stp_error)
{
    // [30-3-21] SAME AS execute_StpTrapzProfile3, but here even if stepper finishes continues to check for dxls moving and logs data!
    KILL_MOTION = false;
    bool DXLS_REACHED_GOAL_POS    = false;  // assumes that sync set goal position command to dxl was sent before this function call!!!
    bool STEPPER_REACHED_GOAL_POS = false;  // logical, it hasn't even started!
    bool SYNC_MOTION_FINISHED     = false;

    // Vars for real time position/velocity
    float CURRENT_CONFIGURATION_RAD[nDoF];
    float CURRENT_JOINT_VELOCITY_RS[nDoF];
    byte joint_pos_log_cnt = 0;    // 1 => single zero line was written in file creation! 0 => nothing was written @file creation
    byte joint_vel_log_cnt = 0;    // single zero line was written in file creation!
    bool updateJointPosLog = false;
    bool updateJointVelLog = false;
    // Vars for real time current measurement
    float CURRENT_MEASUREMENTS_mA[nDoF];
    byte joint_cur_log_cnt = 0;    // single zero line was written in file creation!
    bool updateJointCurLog = false;

    // Vars for force measurements
    //ptr2ForceSensor = ForceSensor;
    float * UPDATED_FORCE_MEASUREMENT_KGS;   // [21-3-21] single force measurement Zaxis 
    unsigned long force_update_duration;
    unsigned long started_force_update;
    byte force_log_cnt = 0;        // single zero line was written in file creation!
    bool updateForceLog = false;

    // Initialize variable for timing/derivative calculations
    float prev_delta_t = delta_t;                            // delta_t <=> c0
    float new_delta_t  = prev_delta_t;                       // Delay time in [sec] - Here the initial value is given, will change inside loop
    unsigned long new_delta_t_micros;                         // Delay time in [micros]

    uint32_t StpPresentPosition = 0;
    bool updateStepDelayTime = false;                         // the delay time MUST NOT BE CALCULATED in first do-while iteration

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

    // Stepping loop
    unsigned long motor_movement_start = millis();
    do
    {
        // Inside this if every desired update() function must be called
        if (updateStepDelayTime)
        {
          setStepperLed(stepper_turn_off_led);

          // 1. Always update delay time when stepper has changed state!
          CustomStepperOvidiusDueShield::updateDelayTime(&new_delta_t, &prev_delta_t, StpPresentPosition, segmentExists, profile_steps_new, stp_error);
          
        }
        else
        {
          setStepperLed(stepper_stepping_in_progress);         // Olive

          // HERE until next step pin state change we have time to calculate real time data...
          // CAUTION -> NOTHING is ALWAYS executed here! Here we have time to check if it is time 
          // to update the real time data, and if and only if time has come, we calculate/read!

          // 2. Updates force sensor values -> talks to HX711 using OvidiusSensors methods
          if (UPDATE_SENSOR[FOR_LOG_ID])
          {
            updateForceMeasurements(ptr2ForceSensor, ptr2ForceSensorAxis, UPDATED_FORCE_MEASUREMENT_KGS, updateForceLog, sensor_error);
          }

          // 3. read current joint positions
          if (UPDATE_SENSOR[POS_LOG_ID])
          {
            getJointsAbsPosition_rad(CURRENT_CONFIGURATION_RAD, ptr2custom_dxl, ptr2_dxl_pp_pck, StpPresentPosition, updateJointPosLog, stp_error);
          }

          // 4. read current joint velocities
          if (UPDATE_SENSOR[VEL_LOG_ID])
          {
            getJointsAngVelocity_rs(CURRENT_JOINT_VELOCITY_RS, ptr2custom_dxl, ptr2_dxl_pv_pck, (new_delta_t / 2.0f), updateJointVelLog ,  stp_error);
          }

          // 5. read current of joints
          if (UPDATE_SENSOR[CUR_LOG_ID])
          {
            //getJointsCurrent_mA(CURRENT_MEASUREMENTS_mA, ptr2CurrentSensor, ptr2current_packet,ptr2custom_dxl,ptr2_dxl_pc_pck, updateJointCurLog, stp_error  ); -> for INA219b only!
            getJointsCurrent_A(CURRENT_MEASUREMENTS_mA, ptr2CurrentSensor, ptr2custom_dxl, ptr2_dxl_pc_pck, updateJointCurLog, stp_error  );
          }

          // 6. check what dynamixels are doing(moving=1,finished=0)

          // 7. data logs:
          if ( updateJointPosLog )
          {
              ptr2logger->writeData2LogArray(CURRENT_CONFIGURATION_RAD, nDoF,  millis(), joint_pos_log_cnt, (ptr2log_structs + POS_LOG_ID), sensor_error);
              joint_pos_log_cnt++;
          }
          if  ( updateJointVelLog ) 
          {
              ptr2logger->writeData2LogArray(CURRENT_JOINT_VELOCITY_RS, nDoF, millis(), joint_vel_log_cnt, (ptr2log_structs + VEL_LOG_ID), sensor_error);
              joint_vel_log_cnt++;         
          }
          if ( updateForceLog ) 
          {
              ptr2logger->writeData2LogArray(UPDATED_FORCE_MEASUREMENT_KGS, 1, millis(), force_log_cnt, (ptr2log_structs+ FOR_LOG_ID), sensor_error);
              force_log_cnt++;
          }
          if ( updateJointCurLog ) 
          {
              ptr2logger->writeData2LogArray(CURRENT_MEASUREMENTS_mA, nDoF, millis(), joint_cur_log_cnt, (ptr2log_structs + CUR_LOG_ID), sensor_error);
              joint_cur_log_cnt++;
          }   

        }

        // the stepping is executed only if stepper hasn't reached target position
        if (!STEPPER_REACHED_GOAL_POS)
        {
          new_delta_t_micros = (unsigned long) ( (new_delta_t / 2.0f) * 1000000) ;   // [DEL_TIME_SEC] * [SEC2MICROS]
          prev_delta_t = (new_delta_t / 2.0f);  // the modified new_delta_t will next implemented and this will return as previous...

          if (new_delta_t_micros < MIN_STEP_DELAY_MICROS)
          {
            new_delta_t_micros = MIN_STEP_DELAY_MICROS;
            prev_delta_t = 0.000025f;
          }

          // Send step pulse unsigned long delayTime, long & StpPresentPosition, bool & updateDelayTime
          CustomStepperOvidiusDueShield::updateSingleStepVarDelay(new_delta_t_micros, &StpPresentPosition, &updateStepDelayTime);  // Updates Motor Step 
        }
        //else
        //{
          // requests to step updating delay time, so now only the #else Ln1704 is executed
        //  updateStepDelayTime = false;
        //  StpPresentPosition  = profile_steps_new[0]; // redundant, just to make sure StpPresentPosition will never change value  
        //}

        // CHECK IF STEPPER FINISHED
        if ( (profile_steps_new[0] - StpPresentPosition) == 0)
        {
          STEPPER_REACHED_GOAL_POS = true;
          updateStepDelayTime = false;
        }
        else
        {
          STEPPER_REACHED_GOAL_POS = false;
        }

        // CHECK IF DYNAMIXELS FINISHED: CURRENT_CONFIGURATION_RAD WILL CONTINUE TO CHANGE SINCE READING FROM DXL DOESN'T STOP!
        if( ( fabs(CURRENT_CONFIGURATION_RAD[1] - GOAL_CONFIGURATION[1]) < ANG_POS_THRESHOLD) && ( fabs(CURRENT_CONFIGURATION_RAD[2] - GOAL_CONFIGURATION[2]) < ANG_POS_THRESHOLD) && ( fabs(CURRENT_CONFIGURATION_RAD[3] - GOAL_CONFIGURATION[3]) < ANG_POS_THRESHOLD) )
        {
          DXLS_REACHED_GOAL_POS = true;
        }
        else
        {
          DXLS_REACHED_GOAL_POS = false;
        }

        if (STEPPER_REACHED_GOAL_POS && DXLS_REACHED_GOAL_POS)
        {
          SYNC_MOTION_FINISHED = true;
        }

      
      // CHECK IF STEPPER HIT THE LIMITS
      if (KILL_MOTION)
      {
        SYNC_MOTION_FINISHED = true; // terminates the stepping loop
        *stp_error = REACHED_JOINT1_LIMIT;
      }

    }while( !SYNC_MOTION_FINISHED  ); // while loop takes into consideration ONLY the Stepper!

    if ( SYNC_MOTION_FINISHED )
    {
      KILL_MOTION = false;     // change the volatile emergency variable in order to continue safe operation
      return true;
    }
    else
    {
      return false;
    }

} // END OF FUNCTION

// =========================================================================================================== //

/*
bool CustomStepperOvidiusDueShield::setStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta,  volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered, bool * segment_exists, uint32_t * profile_steps,   unsigned char *stp_error)
{
  // FUN_CODE = 16* used for stp_error assignment
  // Follows flowchart of setStepperGoalPositionFixedStep but now a var step is used in each segment of motion
  // This function executes a Trapezoidal Velocity Profile. It can be used for synced motion with Dynamixels.

    bool return_function_state;

    // I . Determine Direction of Rotation
    return_function_state = CustomStepperOvidiusDueShield::setStepperGoalDirection(currentAbsPos_double, goalAbsPos_double, currentDirStatus);
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
    unsigned char ERROR_RETURNED;
    return_function_state = CustomStepperOvidiusDueShield::testP2Pparams_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, Vexec, Aexec, Texec, Ta, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    bool SEGMENT_EXISTS; 
    return_function_state = CustomStepperOvidiusDueShield::segmentExists_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double,  *Vexec, Aexec, *Texec, *Ta, &SEGMENT_EXISTS, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    //uint32_t PROFILE_STEPS[4];
    return_function_state = CustomStepperOvidiusDueShield::returnSteps_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, *Vexec, Aexec, Texec, Ta,  &SEGMENT_EXISTS, stp_error, profile_steps);
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
    double c0 = CustomStepperOvidiusDueShield::calculateInitialStepDelay(Aexec);

    return_function_state = CustomStepperOvidiusDueShield::execute_StpTrapzProfile(profile_steps, &SEGMENT_EXISTS,  Texec,  c0, currentDirStatus, &ERROR_RETURNED);
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
*/
// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::syncPreSetStepperGoalPositionVarStep(float * currentAbsPos_double, float * goalAbsPos_double, float * Vexec, float * Aexec, float * Texec, float * Ta,  volatile byte * currentDirStatus, bool * segment_exists, uint32_t * profile_steps,   unsigned char *stp_error)
{
  // FUN_CODE = 17* used for stp_error assignment
  /*
   * Follows function 16. This function IS used for synced motion with Dynamixels. To execute Stepper motion
   * syncSetStepperGoalPositionVarStep must be called after txPacket for Dxl GP has been sent!
   * Final Texec,Ta must be passed to calculateProfAccel_preassignedVelTexec for syncing motion with Dxls!
   */
    bool return_function_state;

    // I . Determine Direction of Rotation
    return_function_state = CustomStepperOvidiusDueShield::setStepperGoalDirection(currentAbsPos_double, goalAbsPos_double, currentDirStatus);
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
    unsigned char ERROR_RETURNED;
    return_function_state = CustomStepperOvidiusDueShield::testP2Pparams_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, Vexec, Aexec, Texec, Ta, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    bool SEGMENT_EXISTS; 
    return_function_state = CustomStepperOvidiusDueShield::segmentExists_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double,  *Vexec, Aexec, *Texec, *Ta, &SEGMENT_EXISTS, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    //uint32_t PROFILE_STEPS[4];
    return_function_state = CustomStepperOvidiusDueShield::returnSteps_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, *Vexec, Aexec, Texec, Ta,  &SEGMENT_EXISTS, stp_error, profile_steps);
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

bool CustomStepperOvidiusDueShield::syncPreSetStepperGoalPositionVarStep2(float * currentAbsPos_double, float * goalAbsPos_double, float * Vexec, float * Aexec, float * Texec, float * Ta,  volatile byte * currentDirStatus, bool * segment_exists, uint32_t * profile_steps,   unsigned char *stp_error)
{
  // FUN_CODE = __* used for stp_error assignment
  /*
   *  Written for p2pcsp2.ino. Based on syncPreSetStepperGoalPositionVarStep BUT here calculateProfVelAccel_preassignedVelTexec3 is used for (Vexec,Axec,Texec)
   */
    bool return_function_state;
    unsigned char ERROR_RETURNED;
    
    // I . Determine Direction of Rotation
    return_function_state = CustomStepperOvidiusDueShield::setStepperGoalDirection(currentAbsPos_double, goalAbsPos_double, currentDirStatus);
    if (return_function_state)
    {
      *stp_error = 0;
    }
    else
    {
      *stp_error = 171;
    }
 
    bool SEGMENT_EXISTS; 
    return_function_state = CustomStepperOvidiusDueShield::segmentExists_StpTrapzVelProfile2(currentAbsPos_double, goalAbsPos_double,  *Vexec, Aexec, *Texec, *Ta, &SEGMENT_EXISTS, &ERROR_RETURNED);
    if (return_function_state)
    {
      *stp_error = 0;
      (*segment_exists) = (&SEGMENT_EXISTS);  // in order to access in main ino file
    }
    else
    {
      *stp_error = ERROR_RETURNED;
    }

    return_function_state = CustomStepperOvidiusDueShield::returnSteps_StpTrapzVelProfile(currentAbsPos_double, goalAbsPos_double, *Vexec, Aexec, Texec, Ta,  &SEGMENT_EXISTS, stp_error, profile_steps);
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
/*
bool CustomStepperOvidiusDueShield::syncSetStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, double * Texec,  volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, bool * segment_exists, uint32_t * profile_steps,   unsigned char *stp_error)
{
  // FUN_CODE = 18* used for stp_error assignment
  // Executed after function 17 has completed. In main code, previously, syncSetDynamixelsGoalPosition must be called!

  bool return_function_state;
  unsigned char ERROR_RETURNED;

  double c0 = CustomStepperOvidiusDueShield::calculateInitialStepDelay(Aexec);

  return_function_state = CustomStepperOvidiusDueShield::execute_StpTrapzProfile(profile_steps, segment_exists,  Texec,  c0, currentDirStatus, &ERROR_RETURNED);
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
*/
// =========================================================================================================== //
/*
bool CustomStepperOvidiusDueShield::syncSetStepperGoalPositionVarStep2(tools::dataLogger *ptr2logger, File *ptr2logfiles, char *ptr2logfiles_names, sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet,  sensors::imu_filter FILTER_SELECT, sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *force_error, double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, double * Texec,  volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, bool * segment_exists, bool UPDATE_FORCE, bool UPDATE_IMU, uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck,   unsigned char *stp_error)
{

  bool return_function_state;
  unsigned char ERROR_RETURNED;

  double c0 = CustomStepperOvidiusDueShield::calculateInitialStepDelay(Aexec);

  return_function_state = CustomStepperOvidiusDueShield::execute_StpTrapzProfile2(ptr2logger, ptr2logfiles, ptr2logfiles_names, ptr2IMU, ptr2imu_packet, FILTER_SELECT,  ptr2ForceSensor, ptr2ForceSensorAxis, UPDATED_FORCE_MEASUREMENTS_KGS, force_error, profile_steps, segment_exists,  Texec,  c0, currentDirStatus,  UPDATE_FORCE,  UPDATE_IMU,ptr2custom_dxl, ptr2_dxl_pp_pck, ptr2_dxl_pv_pck, &ERROR_RETURNED);
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
*/
// =========================================================================================================== //

//bool CustomStepperOvidiusDueShield::syncSetStepperGoalPositionVarStep3(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char *ptr2logfiles_names, sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis ,sensors::currentSensor * ptr2CurrentSensor, sensors::current_packet * ptr2current_packet, debug_error_type *sensor_error, double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, bool * segment_exists, bool UPDATE_FORCE, bool UPDATE_CURRENT, uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck,  unsigned char *stp_error)
//bool CustomStepperOvidiusDueShield::syncSetStepperGoalPositionVarStep3(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char * ptr2logfiles_names, sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis ,sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, bool * segment_exists, bool UPDATE_FORCE, bool UPDATE_CURRENT, uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck,  unsigned char *stp_error)
bool CustomStepperOvidiusDueShield::syncSetStepperGoalPositionVarStep3(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char * ptr2logfiles_names[LOG_FILES_DIR_CHAR_LEN], sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis ,sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, float * currentAbsPos_double, float * goalAbsPos_double, float * Aexec, bool * segment_exists, bool UPDATE_SENSOR[], uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char *stp_error)
{
  // Same as syncSetStepperGoalPositionVarStep2, but here no IMU sensor is used!
  // [24-3-21] Removed sensors::current_packet * ptr2current_packet because Adafruit ina219 is not used anymore!
  
  bool return_function_state;

  double c0 = CustomStepperOvidiusDueShield::calculateInitialStepDelay(Aexec);

  return_function_state = CustomStepperOvidiusDueShield::execute_StpTrapzProfile3(ptr2logger, ptr2logfiles, ptr2logfiles_names, ptr2ForceSensor, ptr2ForceSensorAxis, ptr2CurrentSensor,sensor_error, profile_steps, segment_exists,  c0,  UPDATE_SENSOR, ptr2custom_dxl, ptr2_dxl_pp_pck, ptr2_dxl_pv_pck,ptr2_dxl_pc_pck, ptr2_dxl_mov_pck , stp_error);
  if (return_function_state)
  {
    // SINCE TRUE RETURNED => STEPPER REACHED GOAL POSITION
    (*currentAbsPos_double) = (*goalAbsPos_double);
    _currentAbsPos_rad = (*currentAbsPos_double);   // a private member to store abs position
    _currentAngVel_rps = 0;                         // a private member to store abs velocity
    _currentAngAccel_rps2 = 0;                      // a private member to store abs acceleration

    KILL_MOTION = false;
    return true;
  }
  else
  {
    KILL_MOTION = false;
    return false;
  }

}

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::syncSetStepperGoalPositionVarStep4(tools::dataLogger *ptr2logger, File *ptr2logfiles, const char * ptr2logfiles_names[LOG_FILES_DIR_CHAR_LEN], sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis ,sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, float * currentAbsPos_double, float * goalAbsPos_double, float * GOAL_CONFIGURATION, float * Aexec, bool * segment_exists, bool UPDATE_SENSOR[], uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char *stp_error)
{
  // Same as syncSetStepperGoalPositionVarStep2, but here no IMU sensor is used!
  // [24-3-21] Removed sensors::current_packet * ptr2current_packet because Adafruit ina219 is not used anymore!
  
  bool return_function_state;

  double c0 = CustomStepperOvidiusDueShield::calculateInitialStepDelay(Aexec);

  return_function_state = CustomStepperOvidiusDueShield::execute_StpTrapzProfile4(ptr2logger, ptr2logfiles, ptr2logfiles_names, ptr2ForceSensor, ptr2ForceSensorAxis, ptr2CurrentSensor,sensor_error, profile_steps, segment_exists,  c0, GOAL_CONFIGURATION,  UPDATE_SENSOR, ptr2custom_dxl, ptr2_dxl_pp_pck, ptr2_dxl_pv_pck,ptr2_dxl_pc_pck, ptr2_dxl_mov_pck , stp_error);
  if (return_function_state)
  {
    // SINCE TRUE RETURNED => STEPPER REACHED GOAL POSITION
    (*currentAbsPos_double) = (*goalAbsPos_double);
    _currentAbsPos_rad = (*currentAbsPos_double);   // a private member to store abs position
    _currentAngVel_rps = 0;                         // a private member to store abs velocity
    _currentAngAccel_rps2 = 0;                      // a private member to store abs acceleration

    KILL_MOTION = false;
    return true;
  }
  else
  {
    KILL_MOTION = false;
    return false;
  }

}

// =========================================================================================================== //

bool CustomStepperOvidiusDueShield::syncSetStepperGoalPositionVarStep5(tools::dataLogger *ptr2logger,tools::real_time_log_struct * ptr2log_structs, sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis ,sensors::currentSensor * ptr2CurrentSensor, debug_error_type *sensor_error, float * currentAbsPos_double, float * goalAbsPos_double, float * GOAL_CONFIGURATION, float * Aexec, bool * segment_exists, bool UPDATE_SENSOR[], uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, DXL_PC_PACKET *ptr2_dxl_pc_pck, DXL_MOV_PACKET *ptr2_dxl_mov_pck, unsigned char *stp_error)
{
  
  bool return_function_state;

  double c0 = CustomStepperOvidiusDueShield::calculateInitialStepDelay(Aexec);

  return_function_state = CustomStepperOvidiusDueShield::execute_StpTrapzProfile5(ptr2logger, ptr2log_structs, ptr2ForceSensor, ptr2ForceSensorAxis, ptr2CurrentSensor,sensor_error, profile_steps, segment_exists,  c0, GOAL_CONFIGURATION,  UPDATE_SENSOR, ptr2custom_dxl, ptr2_dxl_pp_pck, ptr2_dxl_pv_pck,ptr2_dxl_pc_pck, ptr2_dxl_mov_pck , stp_error);
  if (return_function_state)
  {
    // SINCE TRUE RETURNED => STEPPER REACHED GOAL POSITION
    (*currentAbsPos_double) = (*goalAbsPos_double);
    _currentAbsPos_rad = (*currentAbsPos_double);   // a private member to store abs position
    _currentAngVel_rps = 0;                         // a private member to store abs velocity
    _currentAngAccel_rps2 = 0;                      // a private member to store abs acceleration

    KILL_MOTION = false;
    return true;
  }
  else
  {
    KILL_MOTION = false;
    return false;
  }

}