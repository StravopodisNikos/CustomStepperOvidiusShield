 /*
  * CustomStepperOvidiusShield.h - Library for controlling Steppers of the Ovidius Metamorphic Manipulator
  * Created by N.A. Stravopodis, December , 2020.
  */

#ifndef CustomStepperOvidiusShield_h
#define CustomStepperOvidiusShield_h

// OPENCR EEPROM AREA ADDRESSES FOR STEPPER NEMA34[0~255]
#define CP_JOINT1_STEPPER_EEPROM_ADDR       1      // double    
#define CD_JOINT1_STEPPER_EEPROM_ADDR       10     // uint32_t
#define VL_JOINT1_STEPPER_EEPROM_ADDR       20     // double
#define AL_JOINT1_STEPPER_EEPROM_ADDR       30     // double    
#define MAX_POS_JOINT1_STEPPER_EEPROM_ADDR  40     // double    

#include "Arduino.h"
#include "HX711.h"
#include <OvidiusSensors.h>
#include <utility/OvidiusSensors_config.h>
#include <Dynamixel2Arduino.h>
#include "DynamixelProPlusOvidiusShield.h"

using namespace std;
/*
extern int  stp_error;
//extern bool return_function_state;
extern bool homingSwitchActivated;
extern bool limit1SwitchActivated;
extern bool limit2SwitchActivated;
extern volatile bool KILL_MOTION;

extern unsigned long time_now_micros;
extern unsigned long time_now_millis;

extern uint32_t currentAbsPos;
extern double currentAbsPos_double;
extern volatile byte currentDirStatus;

extern bool segmentExists;
extern bool positionReached;
extern bool unlockStepper;

extern double VelocityLimitStp;
extern double AccelerationLimitStp;
extern double MaxPosLimitStp;
*/
enum ROT_DIR{CW, CCW};

class CustomStepperOvidiusShield
{
    public:

        CustomStepperOvidiusShield(int stepID, int stepPin, int dirPin, int enblPin, int homeTriggerPin, int limitSwitchPin2, int limitSwitchPin3, int RED_LED_pin,int GREEN_LED_pin,int BLUE_LED_pin, int spr, int GEAR_FACTOR, int ft );

        void updateStpAngVelStp(double &current_ang_vel, double delay_sec);
        
        void updateStpAbsPos_rad(double &realTimeStpAbsPos, uint32_t StpPresentPulse, unsigned char *stp_error);

        void set_ag();

        void getJointsAbsPosition_rad(double * ROBOT_ABS_POS_RAD, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, uint32_t StpPresentPulse, bool &update_joint_pos, unsigned char *error_code);
        void getJointsAngVelocity_rs(double * ROBOT_ANG_VEL_RS, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PV_PACKET *ptr2_dxl_pv_pck, double half_step_delay_sec, bool &update_joint_vel, unsigned char *error_code);

        // read EEPROM settings for stepper Motion Profiles 
        void read_STP_EEPROM_settings(volatile byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp);
        void save_STP_EEPROM_settings(volatile byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp);

        // Moves motor to home position - Hall Sensor and Limit switches Needed
        bool setStepperHomePositionSlow(double * currentAbsPos_double, volatile byte *currentDirStatus,  volatile bool *kill_motion_triggered,  unsigned char *stp_error);
        
        // Moves motor to home position - Hall sensor only for evaluation, No Limit switches Needed - currentAbsPos is read from EEPROM
        bool setStepperHomePositionFast(double * currentAbsPos_double, volatile byte * currentDirStatus, volatile bool * kill_motion_triggered,  unsigned char * stp_error);

        bool setStepperGoalDirection(double * currentAbsPos_double, double * goalAbsPos_double, volatile byte * currentDirStatus);

        bool setStepperGoalPositionFixedStep(double * currentAbsPos_double, double * goalAbsPos_double, volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered,  unsigned char *stp_error);

        bool testP2Pparams_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double *Ta, unsigned char *stp_error);

        bool moveStp2Position(uint32_t * relative_steps_2_move, volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, unsigned char *stp_error);

        uint32_t calculateRelativeSteps2Move(double * currentAbsPos_double, double * goalAbsPos_double, unsigned char *stp_error);

        uint32_t convertRadian2StpPulses(double position_in_radians, unsigned char *stp_error);

        double convertStpPulses2Radian(uint32_t position_in_stp_pulses, unsigned char *stp_error);

        bool segmentExists_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double,  double & Vexec, double * Aexec, double & Texec, double & Ta,  bool * segmentExists, unsigned char *stp_error);

        bool segmentExists_StpTrapzVelProfile2(double * currentAbsPos_double, double * goalAbsPos_double,  double & Vexec, double * Aexec, double & Texec, double & Ta,  bool * segmentExists, unsigned char *stp_error);

        double calculateInitialStepDelay(double * Aexec);

        bool returnSteps_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double, double &Vexec, double * Aexec, double * Texec, double * Ta,  bool * segmentExists, unsigned char * stp_error, uint32_t *profile_steps);
        
        //bool execute_StpTrapzProfile(uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, unsigned char * stp_error);

        bool execute_StpTrapzProfile2(tools::dataLogger *ptr2logger, File *ptr2logfiles , char *ptr2logfiles_names, sensors::imu9dof * ptr2ImuSensor, sensors::imu_packet * ptr2imu_packet,  sensors::imu_filter FILTER_SELECT,  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, bool UPDATE_FORCE, bool UPDATE_IMU, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, unsigned char * stp_error);

        bool execute_StpTrapzProfile3(tools::dataLogger *ptr2logger, File *ptr2logfiles, char *ptr2logfiles_names,  sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *sensor_error, uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, bool UPDATE_FORCE, bool UPDATE_IMU, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck, unsigned char * stp_error);

        //bool setStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta, volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered, bool * segment_exists, uint32_t * profile_steps,  unsigned char *stp_error);

        bool syncPreSetStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta,  volatile byte * currentDirStatus, bool * segment_exists, uint32_t * profile_steps,   unsigned char *stp_error);

        bool syncPreSetStepperGoalPositionVarStep2(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta,  volatile byte * currentDirStatus, bool * segment_exists, uint32_t * profile_steps,   unsigned char *stp_error);

        //bool syncSetStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, double * Texec,  volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, bool * segment_exists, uint32_t * profile_steps,   unsigned char *stp_error);

        bool syncSetStepperGoalPositionVarStep2(tools::dataLogger *ptr2logger, File *ptr2logfiles , char *ptr2logfiles_names, sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet,  sensors::imu_filter FILTER_SELECT, sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *force_error, double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, double * Texec,  volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, bool * segment_exists, bool UPDATE_FORCE, bool UPDATE_IMU, uint32_t * profile_steps,  DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck,   unsigned char *stp_error);

        bool syncSetStepperGoalPositionVarStep3(tools::dataLogger *ptr2logger, File *ptr2logfiles, char *ptr2logfiles_names, sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS , debug_error_type *force_error, double * currentAbsPos_double, double * goalAbsPos_double, double * Aexec, double * Texec,  volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, bool * segment_exists, bool UPDATE_FORCE, bool UPDATE_IMU, uint32_t * profile_steps, DynamixelProPlusOvidiusShield *ptr2custom_dxl,  DXL_PP_PACKET *ptr2_dxl_pp_pck, DXL_PV_PACKET *ptr2_dxl_pv_pck,   unsigned char *stp_error);

    private:
        int _stepID;
        int _stepPin;
        int _dirPin;
        int _enblPin;
        int _homeTriggerPin;
        int _limitSwitchPin2;
        int _limitSwitchPin3;
        int _RED_LED_pin;
        int _GREEN_LED_pin;
        int _BLUE_LED_pin;
        unsigned long _update_LED_interval;         // [millis]
        unsigned long _last_LED_update;             // [millis]

        bool _return_fn_state;

        // Stepper gearbox
        int _spr;
        int _GEAR_FACTOR;
        int _ft;
        double _a;
        double _ag;

        // Monitoring data
        signed char _sign_of_Dq;
        long _currentAbsPos_pulse;             // must be synced with StpPresentPosition
        double _currentAbsPos_rad;
        double _currentAngVel_rps;             // Current Stepper Angular velocity [rad/sec]
        double _currentAngAccel_rps2;          // Current Stepper Angular acceleration [rad/sec2]

        double _prevAbsPos_rad;
        double _prevAngVel_rps;                // Previous Stepper Angular velocity [rad/sec]
        double _prevAngAccel_rps2;             // Previous Stepper Angular acceleration [rad/sec2]

        unsigned long _last_joint_pos_update;
        int _update_joint_pos_interval;
        unsigned long _last_joint_vel_update;
        int _update_joint_vel_interval;

        // Velocity Profile vars 
        double _accel_width;      
        long _accel_count;
        long _ctVel_count;
        long _decel_count;
        boolean _STEP_PIN_STATE;
        unsigned long _update_STEP_STATE_interval;         // [micros]
        unsigned long _last_STEP_STATE_update;             // [micros] 

        void singleStepVarDelay(unsigned long delayTime);

        void multiStepVarDelay(unsigned long delayTime, uint32_t numSteps2Move);

        void updateSingleStepVarDelay(unsigned long delayTime, long * StpPresentPosition, bool * updateDelayTime);

        void updateDelayTime(double * new_delayTime_sec, double * prev_delayTime_sec, long StpPresentPosition, bool * segment_exists, uint32_t * profile_steps, unsigned char *stp_error);

        void updateForceMeasurements(sensors::force3axis * ptr2ForceSensor, HX711 * ptr2ForceSensorAxis, float * UPDATED_FORCE_MEASUREMENTS_KGS, bool &update_force ,debug_error_type *force_error);

        void updateIMU(sensors::imu9dof * ptr2IMU, sensors::imu_packet * ptr2imu_packet, sensors::imu_filter FILTER_SELECT, bool &update_imu, debug_error_type * imu_error);

        /*
        sensors::force3axis ForceSensor[num_FORCE_SENSORS] = {
            sensors::force3axis(DOUT_PIN_X, SCK_PIN_X),   //ForceSensor[0] -> ForceSensorX
            sensors::force3axis(DOUT_PIN_Y, SCK_PIN_Y),   //ForceSensor[1] -> ForceSensorY
            sensors::force3axis(DOUT_PIN_Z, SCK_PIN_Z),   //ForceSensor[2] -> ForceSensorZ
        };
        */
        //sensors::force3axis * ForceSensor;              // points to the force sensor object
        //HX711 * ForceSensorAxis;                        // points to hx711                   
        unsigned long _update_FORCE_interval;             // [millis]
        unsigned long _last_FORCE_update;                 // [millis]
        // IMU interval is not initialized but read from sensors::imu9dof
        // because filter frequency is fixed to system design and may vary
        // together with constants defined in inherited classes of class
        // imu9dof, so its value should be given inside that class ony!
        unsigned long _last_IMU_update;                   // [millis]   

};

 #endif