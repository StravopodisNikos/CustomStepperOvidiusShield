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
#include "Vector.h"

using namespace std;

const float pi              = 3.14159265359;

extern bool return_function_state;
extern Vector<double> TrajAssignedDuration;
extern vector<unsigned long> PROFILE_STEPS;
extern vector<double> vector_for_trajectoryVelocity;

extern unsigned long time_now_micros;
extern unsigned long time_now_millis;

extern unsigned long currentAbsPos;
extern double currentAbsPos_double;
extern long currentMoveRel;
extern byte currentDirStatus;
extern bool segmentExists;
extern bool positionReached;
extern bool unlockStepper;
extern double VelocityLimitStp;
extern double AccelerationLimitStp;
extern double MaxPosLimitStp;

class CustomStepperMetamorphicManipulator
{
    public:

        CustomStepperMetamorphicManipulator(int stepID, int stepPin, int dirPin, int enblPin, int ledPin, int hallHomePin, int limitSwitchMinPin, int limitSwitchMaxPin, int lockPin, int spr, int GEAR_FACTOR, int ft );
        
        // read EEPROM settings for stepper Motion Profiles 
        void read_STP_EEPROM_settings( byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp);
        void save_STP_EEPROM_settings( byte * currentDirStatus, double * currentAbsPos_double, double * VelocityLimitStp, double * AccelerationLimitStp, double * MaxPosLimitStp);

        // Moves motor to home position - Hall Sensor and Limit switches Needed
        bool setStepperHomePositionSlow();
        
        // Moves motor to home position - Hall sensor only for evaluation, No Limit switches Needed - currentAbsPos is read from EEPROM
        bool setStepperHomePositionFast(double * currentAbsPos_double, unsigned long * currentAbsPos,  byte *currentDirStatus);

    private:
        int _stepID;
        int _stepPin;
        int _dirPin;
        int _enblPin;
        int _homeTriggerPin;
        int _limitSwitchPin2;
        int _limitSwitchPin3;
        int _lockPin;
        int _spr;
        int _GEAR_FACTOR;
        int _ft;
        float _a;
        float _ag;
        float _accel_width;
        double _step_delay_time;
        double _simultaneous_velocity;

        void singleStepVarDelay(unsigned long delayTime);

};

 #endif