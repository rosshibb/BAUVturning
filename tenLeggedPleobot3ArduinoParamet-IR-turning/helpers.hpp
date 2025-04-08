// custom functions used in IR control of Pleobot3
#include <Servo.h> 
#include <IRremote.hpp>   // IR remote control library
#include "constants.cpp"

// Define IR remote keys for specific programs
#define Button0 0xE916FF00        // trait inspector; 0-> yaw offset angle, 1-> zigzag number, 2-> pitch offset angle, 3-> period
#define Button1 0xF30CFF00        // Keep the legs vertical for maintenance
#define Button2 0xE718FF00        // Run the kinematics program with controllable yaw
#define Button3 0xA15EFF00        // Run the yaw zigzag program
#define Button4 0xF708FF00        // Run the kinematics program with controllable pitch
#define Button5 0xE31CFF00        // Run the pitch zigzag program
#define Button6 0xA55AFF00        // keep the legs horizontal for transportation
#define Button7 0xBD42FF00        // Currently unused
#define Button8 0xAD52FF00        // Currently unused
#define ButtonPlus 0xEA15FF00     // state 0: increase the specified trait; states 2-5: increase the period
#define ButtonMinus 0xF807FF00    // state 0: decrease the specified trait; states 2-5: decrease the period
#define ButtonNext 0xBF40FF00     // Move to the next state (current option + 1)
#define ButtonPrev 0xBB44FF00     // Move to the previous state (current option - 1)
#define ButtonChMinus 0xBA45FF00  // state 0: move to the previous trait; state 2: decrease the amplitude of specified side; state 4: decrease the amplitude of front legs (subsequently increasing the amplitude of back legs)
#define ButtonCh 0xB946FF00       // state 0&2: switch the side of legs' amplitude to control;
#define ButtonChPlus 0xB847FF00   // state 0: move to the next trait; state 2: increase the amplitude of specified side; state 4: increase the amplitude of front legs (subsequently decreasing the amplitude of back legs)
#define ButtonPlayPause 0xBC43FF00  // state 0: switch binary representation between static and dynamic digits
#define ButtonEQ 0xF609FF00       // state 0: reset the specified trait to default; states 2-5: reset the period to default

int maxState = 6; // the highest number of state currently used

#define SERVOS 5 // the number of servos on each side of the body

float amplitudeStable[SERVOS*2] = {59.0733, 67.0363, 72.3134, 84.0342, 86.127, 59.0733, 67.0363, 72.3134, 84.0342, 86.1277}; // total amplitude of the leg, not to be updated; same order as in servoPins
int ampIncrementDegree = 5; // increment to increase and decrease the amplitude of leg for pitch control
int pitchIncrementDegree = 2; // increment of pitch offset in degrees

unsigned int beatPeriodMillisDefault = 1500; // default value of beat period in ms
unsigned int beatPeriodMillisIncr = 100; // increment to increase and decrease the beat period duration to control the beat frequency

// Store the servo specs
// calibration parameters for 2nd prototype
unsigned int servoPins[SERVOS*2] = {3,5,7,9,11,4,6,8,10,12}; // pins for all the servos, sorted with the first 5 being left legs, from P1 to P5 pleopods. Equivalent to [P1L P2L P3L P4L P5L P1R P2R P3R P4R P5R]
unsigned int minServoPulse[SERVOS*2] = {650,650,650,650,650,650,650,650,650,650}; // minimum servo pulse; same order as in servoPins
unsigned int maxServoPulse[SERVOS*2] = {2350,2350,2350,2350,2350,2350,2350,2350,2350,2350}; // maximum servo pulse; same order as in servoPins
float servoAngleRange[SERVOS*2] = {155.641,156.466,153.502,158.893,159.532,155.155,157.404,157.125,158.934,154.985}; // measured physical range of the servo for the respective min and max pulse. Requires initial calibration of each servo to determine their range. ; Same order as in servoPins
int corrFact[SERVOS*2] = {-30,45,20,45,-15,20,-30,35,-60,-40}; // correction factor after the servos are installed to align the pleopods perfectly vertically; same order as in servoPins
const unsigned int servoUpdatePeriodMillis = 20; // 20ms = 50Hz, the default for servo actuation

int binaryArray[SERVOS] = {1,0,0,0,0}; // first digit represent sign 1-> +, following 4 are binary digits
int binarySwitch = 1; // for switching between binary representation, either 0 or 1

int yawCounterL = 0; int yawCounterR = 0; // counts the offset from the default value of yaw amplitude on each side
int pitchCounter = 0; // counts the offset from the default value of pitch amplitude

// Initiate servos for the left and right legs - keep left and right servos separate for now
Servo Pleft[SERVOS]; // left legs
Servo Pright[SERVOS]; // right legs

void resetAmplitude(float amplitude_[], float amplitude_Stable[]){
  for(unsigned int i = 0; i < SERVOS*2; i++){ // Reset the amplitude changes so that the when you enter the motion program it will be default
    amplitude_[i] = amplitude_Stable[i];
  }
}
// Re-compute the other timing parameters for the beat
void updateBeatPeriod(unsigned int &beat_Period_Steps, const unsigned int servo_Period_Millis, unsigned int beat_Period_Millis, float &phase_, unsigned int &period_Steps_Counter,
int beat_Step_Phase_Begin[], float phase_Lag){
  beat_Period_Steps = beat_Period_Millis / servo_Period_Millis; // re-calculate the number of kinematics steps for the new beat period
  period_Steps_Counter = ceil(phase_ / ((float)servo_Period_Millis / beat_Period_Millis)); // calculate the rounded-up integer value for the period step counter, relative to the counter step of the previous beat period value
  phase_ = ((float)servo_Period_Millis / beat_Period_Millis) * period_Steps_Counter;// recalculate the proper phase for the new counter
  for(int i = 0; i < SERVOS; i++){ // recalculate beatPeriodSteps at which each leg updates the amplitude in yaw control
    float ifloat = i;
    float bps = beat_Period_Steps;
    beat_Step_Phase_Begin[4-i] = int(floor(bps*phase_Lag*ifloat));
    beat_Step_Phase_Begin[9 - i] = int(floor(bps*phase_Lag*ifloat));
  }
  // Serial.println(beat_Period_Steps);
}

// state 0: reset specified trait when EQ is pressed; state 2-5: reset beat period when EQ is pressed
void resetEQ(int state_, int trait_, int leftRightControl_, int &yawCounterL_, int &yawCounterR_, int &turningStrokeCount_, unsigned int &beat_Period_Steps, 
      const unsigned int servo_Period_Millis, unsigned int &beat_Period_Millis, int beatPeriodMillisDefault_, float &phase_, unsigned int &period_Steps_Counter,
      int beat_Step_Phase_Begin[], float phase_Lag, float amplitude_[], float amplitudeStable_[], int ampIncrementDegree_, int &pitchCounter_, int pitchIncrementDegree_){
  if(state_ == 0){
    if(trait_ == 0){
      if(leftRightControl_ == 0){
        yawCounterL_ = 0;
        for(unsigned int i = 0; i < SERVOS; i++){
          amplitude_[i+(leftRightControl_*SERVOS)] = amplitudeStable_[i+(leftRightControl_*SERVOS)] + (ampIncrementDegree_ * yawCounterL_);
        }
      }
      else if(leftRightControl_ == 1){
        yawCounterR_ = 0;
        for(unsigned int i = 0; i < SERVOS; i++){
          amplitude_[i+(leftRightControl_*SERVOS)] = amplitudeStable_[i+(leftRightControl_*SERVOS)] + (ampIncrementDegree_ * yawCounterR_);
        }
      }
    }
    else if(trait_ == 1){
      turningStrokeCount_ = 3;
    }
    else if(trait_ == 2){
      pitchCounter_ = 0;
    }
    else if(trait_ == 3){
      beat_Period_Millis = beatPeriodMillisDefault;
      updateBeatPeriod(beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag);
    }
  }
  else if(state_ == 2){
    if(leftRightControl_ == 0){
      yawCounterL_ = 0;
        for(unsigned int i = 0; i < SERVOS; i++){
          amplitude_[i+(leftRightControl_*SERVOS)] = amplitudeStable_[i+(leftRightControl_*SERVOS)] + (ampIncrementDegree_ * yawCounterL_);
        }
    }
    else if(leftRightControl_ == 1){
      yawCounterR_ = 0;
        for(unsigned int i = 0; i < SERVOS; i++){
          amplitude_[i+(leftRightControl_*SERVOS)] = amplitudeStable_[i+(leftRightControl_*SERVOS)] + (ampIncrementDegree_ * yawCounterR_);
        }
    }
  }
  else if(state_ == 4){
    pitchCounter_ == 0;
    for(int i = 0; i < SERVOS * 2; i++) {
      int offset = 2 - (i % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
      amplitude_[i] = amplitudeStable_[i];
    }
  }
  else{
    beat_Period_Millis = beatPeriodMillisDefault_;
    updateBeatPeriod(beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag);
  }
}

// Compute the pulse width for each servo and command the servos to move to their positions
int writeServoPosition(float pulley_Ratio, unsigned int min_Servo_Pulse[], unsigned int max_Servo_Pulse[], float servo_Angle_Range[], int corr_Fact[], float alpha_Angle_Degree[]){
  // define unchanging variables
  const unsigned int alpha_kinematics_ref = 90; // reference value of alpha kinematics of the leg (not the servo) relative to the horizontal (in degrees). This corresponds to the leg being orthogonal relative to the body axis.

  // write the left and right legs of a pair in the same loop
  for (unsigned int i = 0; i < SERVOS; i++){
    // left leg
    float alpha_servo_ref_l = (servo_Angle_Range[i]/2) + ((corr_Fact[i] * servo_Angle_Range[i]) / (max_Servo_Pulse[i] - min_Servo_Pulse[i]));
    float alpha_servo_degree_l = (alpha_servo_ref_l - (alpha_kinematics_ref / pulley_Ratio)) + (alpha_Angle_Degree[i] / pulley_Ratio); // will need to take the corresponding value from multiple alpha_Angle_Degree for each leg of index i
    int alpha_servo_l = min_Servo_Pulse[i] + (int)((alpha_servo_degree_l * (max_Servo_Pulse[i] - min_Servo_Pulse[i])) / servo_Angle_Range[i]);

    // right leg -- need to separate left and right in the variables to ensure we can write the servos in the next step. Also update the i indices
    float alpha_servo_ref_r = (servo_Angle_Range[i+SERVOS]/2) + ((-corr_Fact[i+SERVOS] * servo_Angle_Range[i+SERVOS]) / (max_Servo_Pulse[i+SERVOS] - min_Servo_Pulse[i+SERVOS]));
    float alpha_servo_degree_r = (alpha_servo_ref_r - (alpha_kinematics_ref / pulley_Ratio)) + (alpha_Angle_Degree[i + SERVOS] / pulley_Ratio);
    int alpha_servo_r = max_Servo_Pulse[i+SERVOS] - (int)((alpha_servo_degree_r * (max_Servo_Pulse[i+SERVOS] - min_Servo_Pulse[i+SERVOS])) / servo_Angle_Range[i+SERVOS]);   


    // Write the servo positions
    Pleft[i].writeMicroseconds(alpha_servo_l);
    Pright[i].writeMicroseconds(alpha_servo_r);

    // for testing only---------------------------
    // if(i == 1){ // 0 = P1, 4 = P5
    //   return alpha_servo_r;
    // }
}
}

//-------------------------------------------------------------------------------------------------------------------
// functions used in setup
// Attach the servos
void attachServos(){
  for (unsigned int i = 0; i < SERVOS; i++){
    Pleft[i].attach(servoPins[i],minServoPulse[i],maxServoPulse[i]);
    Pright[i].attach(servoPins[i+SERVOS],minServoPulse[i+SERVOS],maxServoPulse[i+SERVOS]);
  }
}
// Initialize motion program
void initializeMotion(unsigned long &last_Loop_Time_Millis, float &phase_Start, float &phase_ ) {
  // beat_period_millis = _beat_period_millis;
  last_Loop_Time_Millis = 0;
  phase_Start = 0; 
  phase_ = 0; // reset phase to zero
  // motors_enabled = true;
}

//-------------------------------------------------------------------------------------------------------------------
// Kinematics equation that generates the angle for the corresponding phase of the beat - added after phase calculation
// Set of four functions
float pieceWiseSelect(float x, float a, float b, float c, float d_PS, float d_RS, float phi,  int K, int side){
  float alpha_Angle_Degree;
  
  if(side == 1){ // side PS 2
    alpha_Angle_Degree = b + (d_PS * (a / 2)) - ((a / 2) * cos((x - K - phi) * (PI / c))) - ((a / 2) * (d_PS - 1) * cos((x - K -phi) * ((2 * PI) / c)));
  }

  else if(side == 2){ // side PS 1
    alpha_Angle_Degree = b + (d_PS * (a / 2)) - ((a / 2) * cos((x - K - phi + 1) * (PI /c))) - ((a / 2) * (d_PS - 1) * cos((x - K -phi + 1) * ((2 * PI) / c)));
  }

  else if(side == 3){ // side RS 2
    alpha_Angle_Degree = b + (d_RS * (a / 2)) - ((a / 2) * cos((x - K -phi - 1) * (PI /(c - 1)))) - ((a / 2) * (d_RS - 1) * cos((x - K -phi - 1) * ((2 * PI) / (c - 1))));
  }

  else if(side == 4){ // side RS 1
    alpha_Angle_Degree = b + (d_RS * (a / 2)) - ((a / 2) * cos((x - K - phi) * (PI /(c - 1)))) - ((a / 2) * (d_RS - 1) * cos((x - K -phi) * ((2 * PI) / (c - 1))));
  }

return alpha_Angle_Degree;
}

// Conditional function to calculate the alpha angle (in deg) given the inter-pleopod phase lag
float conditionalPiecewise(float phase_, float amplitude_, float min_Alpha, float temp_Asym, float d_PS, float d_RS, float phase_Lag){
  float alpha_Angle_Degree;
  int k = floor(phase_);

  if (phase_Lag < 0 || phase_Lag >= 1){ // normalizes phase lag to be inbetween 0-1
        phase_Lag = phase_Lag - floor(phase_Lag);
  }

  if (phase_Lag == 0){
    if (phase_ - k <= temp_Asym){
    alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 1);
    }
    else{
    alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 3);
    }
  }

  else if (phase_Lag == 1 - temp_Asym){
    if(phase_ - k <= temp_Asym){
    alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 4);    
    }
    else{
    alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 1);     
    }
  }

  else if (phase_Lag < (1 - temp_Asym) && phase_Lag > 0){
    if (phase_ - k <= phase_Lag){
      alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 4); 
    }
    else if (phase_ -k > phase_Lag && phase_ - k <= phase_Lag + temp_Asym){
      alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 1);
    }
    else{
      alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 3);
    }
  } 

  else if (phase_Lag > (1 - temp_Asym) && phase_Lag < 1){
    if (phase_ - k <= phase_Lag - (1 - temp_Asym) && phase_ -k >= 0){
      alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 2); 
    }
    else if (phase_ - k > phase_Lag - (1 - temp_Asym) && phase_ - k <= phase_Lag){
      alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 4);
    }
    else{
      alpha_Angle_Degree = pieceWiseSelect(phase_, amplitude_, min_Alpha, temp_Asym, d_PS, d_RS, phase_Lag, k, 1);
    }
  }

  return alpha_Angle_Degree;
}

void alphaAngleDeg(float phase_, float amplitude_[], float min_Alpha[], float temp_Asym[], float d_PS[], float d_RS[], float phase_Lag, float alpha_Angle_Degree[]) {
  // float servo_Angle[SERVOS*2]; // initiate an array to store all the angle of all the legs as LLLLLRRRRR. This order is important as writeServoPosition is set to read the leg index that way.
  unsigned int counterLag; // counter used to calculate the phase lag from P5 to P1 given that the calculations are performed in the reverse order from P1 to P5.

  // left legs
  for (unsigned int i = 0; i < SERVOS; i++){
    counterLag = SERVOS - 1 - i;
    alpha_Angle_Degree[i] = conditionalPiecewise(phase_, amplitude_[i], min_Alpha[i], temp_Asym[i], d_PS[i], d_RS[i], phase_Lag * counterLag); // calculate the alpha angle in degrees
  }

  // right legs
  for (unsigned int i = 0; i < SERVOS; i++){
    counterLag = SERVOS - 1 - i;
    alpha_Angle_Degree[i+SERVOS] = conditionalPiecewise(phase_, amplitude_[i+SERVOS], min_Alpha[i+SERVOS], temp_Asym[i+SERVOS], d_PS[i+SERVOS], d_RS[i+SERVOS], phase_Lag * counterLag); // calculate the alpha angle in degrees
  }
  // return servo_Angle;
}

//-------------------------------------------------------------------------------------------------------------------
// functions used for yaw control
// switch turning left and right [CURRENTLY NOT USED]
// void switchTurnLeftRight(float amplitude_[], float amplitude_stable[], float amplitude_changed[], int changing_index, int turning_Left_Right[]){
//   if(turning_Left_Right[changing_index] == 1){ // update the amplitude to turn right
//     amplitude_[changing_index] = amplitude_stable[changing_index]; // left leg goes back to default amplitude
//     amplitude_[changing_index + SERVOS] = amplitude_changed[changing_index + SERVOS]; // right leg goes to an updated amplitude
//     turning_Left_Right[changing_index] = 0;
//     // Serial.print("P");
//     // Serial.print(5 - changing_index);
//     // Serial.println(" SWITCH TO RIGHT!!!!!!!!!!!!!!!!!!!!!!!!!!");
//   } else if(turning_Left_Right[changing_index] == 0){ // update the amplitude to turn left
//     amplitude_[changing_index] = amplitude_changed[changing_index]; // left leg goes to an updated amplitude
//     amplitude_[changing_index + SERVOS] = amplitude_stable[changing_index + SERVOS]; // right goes back to default amplitude
//     turning_Left_Right[changing_index] = 1;
//     // Serial.print("P");
//     // Serial.print(5 - changing_index);
//     // Serial.println(" SWITCH TO LEFT!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//   }
// }
// switch turning left and right with binary
void switchTurnLeftRightBinaryComp(int yawCounterL_, int yawCounterR_, float amplitude_[], float amplitude_Stable[], float amplitude_changed[], int changing_index, int turning_Left_Right[], int amp_Increment_Degree){
  if(turning_Left_Right[changing_index] == 1){ // update the amplitude to turn right
    amplitude_[changing_index] = amplitude_Stable[changing_index] + amp_Increment_Degree * yawCounterR_;
    amplitude_[changing_index+SERVOS] = amplitude_Stable[changing_index+SERVOS] + amp_Increment_Degree * yawCounterL_;

    turning_Left_Right[changing_index] = 0;

  } else if(turning_Left_Right[changing_index] == 0){ // update the amplitude to turn left
    amplitude_[changing_index] = amplitude_Stable[changing_index] + amp_Increment_Degree * yawCounterL_;
    amplitude_[changing_index+SERVOS] = amplitude_Stable[changing_index+SERVOS] + amp_Increment_Degree * yawCounterR_;

    turning_Left_Right[changing_index] = 1;
  }
}
// switch pitching up and down [CURRENTLY NOT USED]
// void switchPitchUpDown(float amplitude_[], float amplitude_stable[], float amplitude_changed[], int changing_index, int pitching_Up_Down[]){
//   if(pitching_Up_Down[changing_index] == 0){ // update the amplitude to pitch up
//     amplitude_[changing_index] = amplitude_changed[changing_index]; // front legs decrease in amplitude, back legs increase in amplitude
//     amplitude_[changing_index + SERVOS] = amplitude_changed[changing_index + SERVOS]; // front legs decrease in amplitude, back legs increase in amplitude
//     pitching_Up_Down[changing_index] = 1;
//   } else if(pitching_Up_Down[changing_index] == 1){ // update the amplitude to pitch down
//     amplitude_[changing_index] = amplitude_changed[changing_index + SERVOS]; // front legs increase in amplitude, back legs decrease in amplitude
//     amplitude_[changing_index + SERVOS] = amplitude_changed[changing_index]; // front legs increase in amplitude, back legs decrease in amplitude
//     pitching_Up_Down[changing_index] = 0;
//   }
// }
// switch pitching up and down
void switchPitchUpDownBinaryComp(float amplitude_[], float amplitude_stable[], float amplitude_changed[], int changing_index, int pitching_Up_Down[], int pitchCounter_, int pitchIncrementDegree_){
  int offset = 2 - (changing_index % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
  if(pitching_Up_Down[changing_index] == 0){ // update the amplitude to pitch up
    amplitude_[changing_index] = amplitude_stable[changing_index] + pitchIncrementDegree_ * offset * pitchCounter_;
    amplitude_[changing_index + SERVOS] = amplitude_stable[changing_index + SERVOS] + pitchIncrementDegree_ * offset * pitchCounter_;
    pitching_Up_Down[changing_index] = 1;
  } else if(pitching_Up_Down[changing_index] == 1){ // update the amplitude to pitch down
    amplitude_[changing_index] = amplitude_stable[changing_index] - pitchIncrementDegree_ * offset * pitchCounter_;  
    amplitude_[changing_index + SERVOS] = amplitude_stable[changing_index + SERVOS] - pitchIncrementDegree_ * offset * pitchCounter_;  
    pitching_Up_Down[changing_index] = 0;
  }
}
// update the traits according to their respective counters when entering a state
void setupState(int state_, float amplitude_[], float amplitude_Stable[], int amp_Increment_Degree, int yawCounterL_, int yawCounterR_, int pitchIncrementDegree_, int pitchCounter_,
  unsigned long &lastLoopTimeMillis_, unsigned int &beat_Period_Steps, const unsigned int servo_Period_Millis, unsigned int beat_Period_Millis, float &phase_, unsigned int &period_Steps_Counter,
  int beat_Step_Phase_Begin[], float phase_Lag, int yawCurrentStrokeCount_[], int pitchCurrentStrokeCount_[], int turningLeftRight_[], int pitchingUpDown_[]){
    
  unsigned long currentMillis_ = millis();
  phase_ = 0.0001;
  lastLoopTimeMillis_ = currentMillis_;
  //     Serial.print("phase lag before : ");
  // Serial.println(phase_Lag);
  updateBeatPeriod(beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag);
  // Serial.print("beat period steps : ");
  // Serial.println(beat_Period_Steps);
  //   Serial.print("phase lag after : ");
  // Serial.println(phase_Lag);
  if(state_ == 2 || state_ == 3){
    for(unsigned int i = 0; i < SERVOS; i++){
      amplitude_[i+(0*SERVOS)] = amplitude_Stable[i+(0*SERVOS)] + (amp_Increment_Degree * yawCounterL_); // calculate the amplitude based on counters
    }
    for(unsigned int i = 0; i < SERVOS; i++){
      amplitude_[i+(1*SERVOS)] = amplitude_Stable[i+(1*SERVOS)] + (amp_Increment_Degree * yawCounterR_); // calculate the amplitude based on counters
    }
  }
  else if(state_ == 4 || state_ == 5){
    for(int i = 0; i < SERVOS * 2; i++) {
      int offset = 2 - (i % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
      amplitude_[i] = amplitude_Stable[i] + pitchIncrementDegree_ * offset * pitchCounter_;
    }

  }

  if(state_ == 3 || state_ == 5){
    for(int i = 0; i < SERVOS; i++){
      yawCurrentStrokeCount_[i] = 0;
      pitchCurrentStrokeCount_[i] = 0;
      turningLeftRight_[i] = 0;
      pitchingUpDown_[i] = 0;
    }
    yawCurrentStrokeCount_[4] = 1;
    pitchCurrentStrokeCount_[4] = 1;
  }
}
// control with IR remote
void optionIRRemote(unsigned int &beat_Period_Millis, unsigned int beat_Period_Millis_Incr, unsigned int &State, bool &option_Changed, float amplitude_[SERVOS*2],
      float amplitude_Stable[], unsigned int &left_Right_Control, int amp_Increment_Degree, unsigned int &beat_Period_Steps, int servo_Period_Millis, float phase_,
      unsigned int &period_Steps_Counter, int beat_Step_Phase_Begin[SERVOS*2], float phase_Lag, int &trait_, int &turningStrokeCount_, float yawAmplitudeChanged_[], int &binarySwitch_,
      int &yawCounterL_, int &yawCounterR_, int beatPeriodMillisDefault_, int &pitchCounter_, int pitchIncrementDegree_, unsigned long &lastLoopTimeMillis_,
      int yawCurrentStrokeCount_[], int pitchCurrentStrokeCount_[], int turningLeftRight_[], int pitchingUpDown_[]){
  if(IrReceiver.decode()){  
    if(IrReceiver.decodedIRData.decodedRawData == Button0){ // trait inspector; 0-> yaw offset angle, 1-> zigzag number, 2-> pitch offset angle, 3-> period
      State = 0;
      resetAmplitude(amplitude_, amplitude_Stable);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button1){ // Keep the legs vertical for maintenance
      State = 1;
      resetAmplitude(amplitude_, amplitude_Stable);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button2){ // Run the kinematics program with controllable yaw
      State = 2;
      resetAmplitude(amplitude_, amplitude_Stable);
      setupState(State, amplitude_, amplitude_Stable, amp_Increment_Degree, yawCounterL_, yawCounterR_, pitchIncrementDegree_, pitchCounter_, lastLoopTimeMillis_,
      beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, yawCurrentStrokeCount_, pitchCurrentStrokeCount_, turningLeftRight_, pitchingUpDown_);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button3){ // Run the yaw zigzag program
      State = 3;
      resetAmplitude(amplitude_, amplitude_Stable);
      setupState(State, amplitude_, amplitude_Stable, amp_Increment_Degree, yawCounterL_, yawCounterR_, pitchIncrementDegree_, pitchCounter_, lastLoopTimeMillis_,
      beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, yawCurrentStrokeCount_, pitchCurrentStrokeCount_, turningLeftRight_, pitchingUpDown_);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button4){ // Run the kinematics program with controllable pitch
      State = 4;
      resetAmplitude(amplitude_, amplitude_Stable);
      setupState(State, amplitude_, amplitude_Stable, amp_Increment_Degree, yawCounterL_, yawCounterR_, pitchIncrementDegree_, pitchCounter_, lastLoopTimeMillis_,
      beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, yawCurrentStrokeCount_, pitchCurrentStrokeCount_, turningLeftRight_, pitchingUpDown_);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button5){ // Run the pitch zigzag program
      State = 5;
      resetAmplitude(amplitude_, amplitude_Stable);
      setupState(State, amplitude_, amplitude_Stable, amp_Increment_Degree, yawCounterL_, yawCounterR_, pitchIncrementDegree_, pitchCounter_, lastLoopTimeMillis_,
      beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, yawCurrentStrokeCount_, pitchCurrentStrokeCount_, turningLeftRight_, pitchingUpDown_);
      Serial.print("state : ");
      Serial.println(State);
    }
    else if(IrReceiver.decodedIRData.decodedRawData == Button6){ // keep the legs horizontal for transportation
      State = 6;
      resetAmplitude(amplitude_, amplitude_Stable);
      setupState(State, amplitude_, amplitude_Stable, amp_Increment_Degree, yawCounterL_, yawCounterR_, pitchIncrementDegree_, pitchCounter_, lastLoopTimeMillis_,
      beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, yawCurrentStrokeCount_, pitchCurrentStrokeCount_, turningLeftRight_, pitchingUpDown_);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonMinus){ // state 0: decrease the specified trait; states 2-5: decrease the period
    if (State == 0){
      if(trait_ == 0){
        if(left_Right_Control == 0){ // Left
          if(yawCounterL_ > -15){
            yawCounterL_--;
          }
          for(unsigned int i = 0; i < SERVOS; i++){
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + (amp_Increment_Degree * yawCounterL_);
          }
          // Serial.print("yawCounterL : ");
          // Serial.println(yawCounterL_);
        }
        else if(left_Right_Control == 1){ // Right
          if(yawCounterR_ > -15){
            yawCounterR_--;
          }
          for(unsigned int i = 0; i < SERVOS; i++){
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + (amp_Increment_Degree * yawCounterR_);
          }
          // Serial.print("yawCounterR : ");
          // Serial.println(yawCounterR_);
        }
      }else if(trait_ == 1){ // update how many times to stroke before switching in zigzag states
        if(turningStrokeCount_ > 1){
          turningStrokeCount_--;
          // Serial.print("turning stroke count : ");
          // Serial.println(turningStrokeCount_);
        }
      }else if(trait_ == 2){ // pitch amplitude offset
        if(pitchCounter_ > -12){
          pitchCounter_--;
        }
        // for(int i = 0; i < SERVOS; i++){
        //  
        // }
      }else if(trait_ == 3){ // update period
        if(beat_Period_Millis >= 200 + beat_Period_Millis_Incr && beat_Period_Millis <= 3000){ // if interval is > than the minimum allowed frequency + increment: this is done so the loop does not compute an interval less than the nminimum allowable 200 ms)
          beat_Period_Millis = beat_Period_Millis - beat_Period_Millis_Incr; // decrease the beat period
          option_Changed = true;
        }
        // Serial.print("beat period : ");
        // Serial.println(beat_Period_Millis);
      }
    }
    else{
        if(beat_Period_Millis >= 200 + beat_Period_Millis_Incr && beat_Period_Millis <= 3000){ // if interval is > than the minimum allowed frequency + increment: this is done so the loop does not compute an interval less than the nminimum allowable 200 ms)
          beat_Period_Millis = beat_Period_Millis - beat_Period_Millis_Incr; // decrease the beat period
          option_Changed = true;
        }
        // Serial.print("beat period : ");
        // Serial.println(beat_Period_Millis);      
    }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonPlus){ // state 0: increase the specified trait; states 2-5: increase the period
    if (State == 0){
      if(trait_ == 0){
        if(left_Right_Control == 0){ // Left
          if(yawCounterL_ < 4){
            yawCounterL_++;
          }
          for(unsigned int i = 0; i < SERVOS; i++){
              amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + amp_Increment_Degree * yawCounterL_;
          }
          // Serial.print("yawCounterL : ");
          // Serial.println(yawCounterL_);
        }
      if(left_Right_Control == 1){ // Right
        if(yawCounterR_ < 4){
          yawCounterR_++;
        }
        for(unsigned int i = 0; i < SERVOS; i++){
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + amp_Increment_Degree * yawCounterR_;
          }
        // Serial.print("yawCounterR : ");
        // Serial.println(yawCounterR_);
      }

      }else if(trait_ == 1){ // update how many times to stroke before switching in zigzag states
        if(turningStrokeCount_ < 15){
          turningStrokeCount_++;
          // Serial.print("turning stroke count : ");
          // Serial.println(turningStrokeCount_);
        }
      }else if(trait_ == 2){ // pitch amplitude offset
        if(pitchCounter_ < 12){
          pitchCounter_++;
        }
        // Serial.print("pitch amplitude offset : ");
        // Serial.println(yawAmplitudeChanged_[0]);
      }
      else if(trait_ == 3){ // update period
        if(beat_Period_Millis >= 200 && beat_Period_Millis <= 3000-beat_Period_Millis_Incr){ // if interval is > than the minimum allowed frequency + increment: this is done so the loop does not compute an interval less than the nminimum allowable 200 ms)
          beat_Period_Millis = beat_Period_Millis + beat_Period_Millis_Incr; // increase the beat period
          option_Changed = true;
        }
        // Serial.print("beat period : ");
        // Serial.println(beat_Period_Millis);
      }
    }
    else{
      if(beat_Period_Millis >= 200 && beat_Period_Millis <= 3000-beat_Period_Millis_Incr){ // if interval is > than the minimum allowed frequency + increment: this is done so the loop does not compute an interval less than the nminimum allowable 200 ms)
          beat_Period_Millis = beat_Period_Millis + beat_Period_Millis_Incr; // increase the beat period
          option_Changed = true;
        }
        // Serial.print("beat period : ");
        // Serial.println(beat_Period_Millis);
    }

    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonNext){ // Flip throught the options one by one in increasing numbers   
      if(State >= 0 && State <= maxState){
        if (State == maxState){ // check that the state does not go beyond 2, otherwise re-set to 0 and stat a new option loop
          State = 0;
          resetAmplitude(amplitude_, amplitude_Stable);
          // option_Changed = true;
        }
        else{
          State+=1; // increase the value by +1 to switch between options
          resetAmplitude(amplitude_, amplitude_Stable);
          // option_Changed = true;
        }
      }
      setupState(State, amplitude_, amplitude_Stable, amp_Increment_Degree, yawCounterL_, yawCounterR_, pitchIncrementDegree_, pitchCounter_, lastLoopTimeMillis_,
      beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, yawCurrentStrokeCount_, pitchCurrentStrokeCount_, turningLeftRight_, pitchingUpDown_);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonPrev){ // Flip throught the options one by one in decreasing numbers 
      if(State >= 0 && State <= maxState){
        if (State == 0){ // check that the state does not go below 0, otherwise re-set to 2 and start a new option loop
          State = maxState;
          resetAmplitude(amplitude_, amplitude_Stable);
          // option_Changed = true;
        }
        else{
          State-=1; // increase the value by +1 to switch between options
          resetAmplitude(amplitude_, amplitude_Stable);
          // option_Changed = true;
        }
      }
      setupState(State, amplitude_, amplitude_Stable, amp_Increment_Degree, yawCounterL_, yawCounterR_, pitchIncrementDegree_, pitchCounter_, lastLoopTimeMillis_,
      beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, yawCurrentStrokeCount_, pitchCurrentStrokeCount_, turningLeftRight_, pitchingUpDown_);
      Serial.print("state : ");
      Serial.println(State);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonChMinus){ // state 0: move to the previous trait; state 2: decrease the amplitude of specified side; state 4: decrease the amplitude of front legs (subsequently increasing the amplitude of back legs)
      if (State == 0){
        trait_--;
        if(trait_ < 0){
          trait_ = 3;
        }
        Serial.print("trait : ");
        Serial.println(trait_);
        // option_Changed = true;
      }
      else if(State == 2){
        if(left_Right_Control == 0){ // Left
          if(yawCounterL_ > -15){
            yawCounterL_--;
          }
          for(unsigned int i = 0; i < SERVOS; i++){
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + (amp_Increment_Degree * yawCounterL_);
          }
        }
        else if(left_Right_Control == 1){ // Right
          if(yawCounterR_ > -15){
            yawCounterR_--;
          }
          for(unsigned int i = 0; i < SERVOS; i++){
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + (amp_Increment_Degree * yawCounterR_);
          }
        }
      }
      else if(State == 4){
        if(pitchCounter_ > -12){
          pitchCounter_--;
        }
          for(int i = 0; i < SERVOS * 2; i++) {
            int offset = 2 - (i % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
            amplitude_[i] = amplitude_Stable[i] + pitchIncrementDegree_ * offset * pitchCounter_;
          }

      }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonChPlus){ // state 0: move to the next trait; state 2: increase the amplitude of specified side; state 4: increase the amplitude of front legs (subsequently increasing the amplitude of back legs)
      if (State == 0){
        trait_++;
        if(trait_ > 3){
          trait_ = 0;
        }
        Serial.print("trait : ");
        Serial.println(trait_);
        // option_Changed = true;
      }
      else if(State == 2){
        if(left_Right_Control == 0){ // Left
          if(yawCounterL_ < 4){
            yawCounterL_++;
          }
          for(unsigned int i = 0; i < SERVOS; i++){
              amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + amp_Increment_Degree * yawCounterL_;
          }
        }
        else if(left_Right_Control == 1){ // Right
          if(yawCounterR_ < 4){
            yawCounterR_++;
          }
          for(unsigned int i = 0; i < SERVOS; i++){
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_Stable[i+(left_Right_Control*SERVOS)] + amp_Increment_Degree * yawCounterR_;
          }
        }
        }
        else if(State == 4){
          if(pitchCounter_ < 12){
            pitchCounter_++;
          }
          for(int i = 0; i < SERVOS * 2; i++) {
            int offset = 2 - (i % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
            amplitude_[i] = amplitude_Stable[i] + pitchIncrementDegree_ * offset * pitchCounter_;
          }
        }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonCh){ // state 0&2: switch the side of legs' amplitude to control;
      if (State == 2 || State == 0){
        left_Right_Control = left_Right_Control + 1;
      if(left_Right_Control >= 2){
        left_Right_Control = 0;
      }
    }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonPlayPause){ // switch to next binary representation
      if (State == 0){
        if (binarySwitch_ == 0){
          binarySwitch_ = 1;
        }
        else if (binarySwitch_ == 1){
          binarySwitch_ = 0;
        }
      }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonEQ){ // state 0: reset the specified trait to default; states 2-5: reset the period to default
      // resetAmplitude(amplitude_, amplitude_Stable);
      resetEQ(State, trait_, left_Right_Control, yawCounterL_, yawCounterR_, turningStrokeCount_, beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, beatPeriodMillisDefault_,
      phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag, amplitude_, amplitude_Stable,amp_Increment_Degree, pitchCounter_, pitchIncrementDegree_);
    }

  IrReceiver.resume(); // this statement is needed to close the if statement and allow for new values to be read
  }
}
// converts decimal number to binary array
void decimalToBinary(int num_, int binaryArray_[]) {
    int pos_sign;
    if (num_ < 0) {  // Negative number
        pos_sign = 0;  // pos_sign variable to 0 => negative
        binaryArray_[0] = 0;
        num_ = num_*-1;
    }
    else {
        pos_sign = 1;  // pos_sign variable to 1 => positive
        binaryArray_[0] = 1;
    }
    if (num_ > 15 || num_ < -15 || num_ == 0) {
        num_ = 0;  // if you go past abs(15), set the num_ to 0, set the corresponding bits and leave function
        for (int k = 1; k < 5; k++){
          binaryArray_[k] = 0;
        }
        return;
    }
    int j = 4;
    for (int k = 1; k < 5; k++){  // Reset P2-5 back to 0
        binaryArray_[k] = 0;
    }
    while (num_ > 0) {
        binaryArray_[j] = num_ % 2;
        num_ /= 2;
        j--;
    }
}
// generates sinusoidal leg motion
float binarySineTrait(float phase_, int trait_){
  float binarySineAngle;
  float mid_point = 25 + (65/2); 
  float ampl2_ = (65);
  if (trait_ == 0){
    binarySineAngle = mid_point + (ampl2_ / 2) * sin(2 * PI * phase_);
    return binarySineAngle;
  }
  else if (trait_ == 1){
    return mid_point;
  }
  else if (trait_ == 2){
    return 90;
  }
  else{
    binarySineAngle = mid_point + (ampl2_ / 2) * sin(2 * PI * phase_);
    return binarySineAngle;
  }
}
// move leg out of midpoint to represent 1 in state 0
void updateAmplitudeAngles(int counterBinary[], float amplitude_[], int trait_, float phase_, int leftRightControl_){
  int k = 40;
  for(int i = 1; i < 5; i++){
    amplitude_[i] = 90 - counterBinary[i]*k;
    amplitude_[i+SERVOS] = 90 - counterBinary[0]*k;
  }
  // amplitude_[0] = 90 - trait_*15;
  amplitude_[SERVOS] = binarySineTrait(phase_, trait_);
  amplitude_[0] = binarySineTrait(phase_, trait_);
  if (trait_ == 0){
    if (leftRightControl_ == 0){
      amplitude_[SERVOS] = 25;
      amplitude_[0] = binarySineTrait(phase_, trait_);
    }
    else{
      amplitude_[0] = 25;
      amplitude_[SERVOS] = binarySineTrait(phase_, trait_);
    }
  }
}

float binarySine(float mid_, float ampl_, float phase_, int binary_val){
  float binarySineAngle;
  if (binary_val == 0){
    return 90;
  }
  binarySineAngle = mid_ + (ampl_ / 2) * sin(2 * PI * phase_);
  return binarySineAngle;
}
// move leg periodically to represent 1 in state 0
void updateAmplitudeAnglesDynamic(int counterBinary[], float amplitude_[], int trait_, float phase_, int leftRightControl_){
  float mid = 90;
  float amplitudeSine = 73.5;
  for(int i = 1; i < 5; i++){
    amplitude_[i] = binarySine(mid, amplitudeSine, phase_, counterBinary[i]);
    amplitude_[i+SERVOS] = binarySine(mid, amplitudeSine, phase_, counterBinary[0]);
  }
  amplitude_[SERVOS] = binarySineTrait(phase_, trait_);
  amplitude_[0] = binarySineTrait(phase_, trait_);
    if (trait_ == 0){
    if (leftRightControl_ == 0){
      amplitude_[SERVOS] = 25;
      amplitude_[0] = binarySineTrait(phase_, trait_);
    }
    else{
      amplitude_[0] = 25;
      amplitude_[SERVOS] = binarySineTrait(phase_, trait_);
    }
  }
}
// displays specified trait in state 0
void displayTrait(int trait_, float yawAmplitudeChanged_[], float amplitudeStable_[], int ampIncrementDegree_, int binaryArray_[], float amplitude_[], int turningStrokeCount_,
      float pitchAmplitudeChanged_[], int beatPeriodMillisIncr_, int beatPeriodMillis_, int beatPeriodMillisDefault_, float phase_, int binarySwitch_, int leftRightControl_,
      int &yawCounterL_, int &yawCounterR_, int pitchCounter_){
  int n;
  if(trait_ == 0){
    if(leftRightControl_ == 0){
      n = yawCounterL_;
    }else if(leftRightControl_ == 1){
      n = yawCounterR_;
    }
  }else if(trait_ == 1){
    n = turningStrokeCount_;
  }else if(trait_ == 2){
    // n = (pitchAmplitudeChanged_[0]- amplitudeStable_[0])/ampIncrementDegree_;
    n = pitchCounter_;
  }else if(trait_ == 3){
    n = (beatPeriodMillis_ - beatPeriodMillisDefault_)/beatPeriodMillisIncr_;
  }

  decimalToBinary(n, binaryArray_);
  if (binarySwitch_ == 0){
    updateAmplitudeAngles(binaryArray_, amplitude_, trait_, phase_, leftRightControl_);
  }
  else if (binarySwitch_ == 1){
    updateAmplitudeAnglesDynamic(binaryArray_, amplitude_, trait_, phase_, leftRightControl_);
  }
}

