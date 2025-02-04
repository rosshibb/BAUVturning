/*
PleobotIR.h - library for IR control of ten legged Pleobot
Created by Tomoya Sasao, February 2, 2025
*/

#ifndef PleobotIR_h
#define PleobotIR_h

#include "Arduino.h"

class PleobotIR
{
    public:
        PleobotIR();
        void optionIRRemote(unsigned int &beat_Period_Millis, unsigned int beat_Period_Millis_Incr, unsigned int &State, bool &option_Changed, float amplitude[SERVOS*2], unsigned int &leftRightControl);
        void attachServos();
        void initializeMotion(unsigned long &last_Loop_Time_Millis, float &phase_Start, float &phase_)
        void alphaAngleDeg(float phase_, float amplitude_[], float min_Alpha[], float temp_Asym[], float d_PS[], float d_RS[], float phase_Lag, float alpha_Angle_Degree[])
        float conditionalPiecewise(float phase_, float amplitude_, float min_Alpha, float temp_Asym, float d_PS, float d_RS, float phase_Lag)
        float pieceWiseSelect(float x, float a, float b, float c, float d_PS, float d_RS, float phi,  int K, int side)
        int writeServoPosition(float pulley_Ratio, unsigned int min_Servo_Pulse[], unsigned int max_Servo_Pulse[], float servo_Angle_Range[], int corr_Fact[], float alpha_Angle_Degree[])
        void updateBeatPeriod(unsigned int &beat_Period_Steps, const unsigned int servo_Period_Millis, unsigned int beat_Period_Millis, float &phase_, unsigned int &period_Steps_Counter, int beat_Step_Phase_Begin[], float phase_Lag)
        void switchTurnLeftRight(float amplitude_[], float amplitude_stable[], float amplitude_changed[], int changing_index)

};

#endif