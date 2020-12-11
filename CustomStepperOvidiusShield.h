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

using namespace std;

const double pi              = 3.14159265359;

extern int  stp_error;
extern bool return_function_state;
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

enum ROT_DIR{CW, CCW};

class CustomStepperOvidiusShield
{
    public:

        CustomStepperOvidiusShield(int stepID, int stepPin, int dirPin, int enblPin, int homeTriggerPin, int limitSwitchPin2, int limitSwitchPin3, int RED_LED_pin,int GREEN_LED_pin,int BLUE_LED_pin, int spr, int GEAR_FACTOR, int ft );

        // read EEPROM settings for stepper Motion Profiles 
        void read_STP_EEPROM_settings(volatile byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp);
        void save_STP_EEPROM_settings(volatile byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp);

        // Moves motor to home position - Hall Sensor and Limit switches Needed
        bool setStepperHomePositionSlow(uint32_t *currentAbsPos, double * currentAbsPos_double, volatile byte *currentDirStatus,  volatile bool *kill_motion_triggered,  int *stp_error);
        
        // Moves motor to home position - Hall sensor only for evaluation, No Limit switches Needed - currentAbsPos is read from EEPROM
        bool setStepperHomePositionFast(double * currentAbsPos_double, uint32_t * currentAbsPos, volatile byte * currentDirStatus);

        bool setStepperGoalDirection(double * currentAbsPos_double, double * goalAbsPos_double, volatile byte * currentDirStatus);

        bool setStepperGoalPositionFixedStep(double * currentAbsPos_double, double * goalAbsPos_double, volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered,  int *stp_error);

        bool testP2Pparams_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double *Ta, int *stp_error);

        bool moveStp2Position(uint32_t * relative_steps_2_move, volatile byte * currentDirStatus, volatile bool *kill_motion_triggered, int *stp_error);

        uint32_t calculateRelativeSteps2Move(double * currentAbsPos_double, double * goalAbsPos_double, int *stp_error);

        uint32_t convertRadian2StpPulses(double position_in_radians, int *stp_error);

        double convertStpPulses2Radian(uint32_t position_in_stp_pulses, int *stp_error);

        bool segmentExists_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double,  double & Vexec, double * Aexec, double & Texec, double & Ta,  bool * segmentExists, int *stp_error);

        double calculateInitialStepDelay(double * Aexec);

        bool returnSteps_StpTrapzVelProfile(double * currentAbsPos_double, double * goalAbsPos_double, double &Vexec, double * Aexec, double * Texec, double * Ta,  bool * segmentExists, int * stp_error, uint32_t *profile_steps);
        
        bool execute_StpTrapzProfile(uint32_t * profile_steps, bool * segmentExists,  double * Texec,  double delta_t, volatile byte * currentDirStatus, int * stp_error);

        bool setStepperGoalPositionVarStep(double * currentAbsPos_double, double * goalAbsPos_double, double * Vexec, double * Aexec, double * Texec, double * Ta, volatile byte * currentDirStatus, uint32_t * relative_movement_in_steps, volatile bool *kill_motion_triggered, bool * segment_exists, uint32_t * profile_steps,  int *stp_error);

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

        int _spr;
        int _GEAR_FACTOR;
        int _ft;
        double _a;
        double _ag;
        double _accel_width;
        double _step_delay_time;
        double _simultaneous_velocity;

        void singleStepVarDelay(unsigned long delayTime);

};

 #endif