// custom functions used in IR control of Pleobot3
#include <Servo.h> 
#include <IRremote.hpp>   // IR remote control library

#define SERVOS 5 // the number of servos on each side of the body

// Define IR remote keys for specific programs
#define Button0 0xE916FF00        // Keep the legs horizontal
#define Button1 0xF30CFF00        // Keep the legs vertical for maintenance
#define Button2 0xE718FF00        // Run the normal kinematics program
#define Button3 0xA15EFF00        // Run the yaw zigzag program
#define Button4 0xF708FF00        // Run the controllable pitch program
#define Button5 0xE31CFF00        // Run the pitch zigzag program
#define Button6 0xA55AFF00        // Currently unused
#define Button7 0xBD42FF00        // Currently unused
#define Button8 0xAD52FF00        // Currently unused
#define ButtonPlus 0xEA15FF00     // increase the frequency -- Volume Plus on remote
#define ButtonMinus 0xF807FF00    // decrease the frequency -- Volume Minus on remote
#define ButtonNext 0xBF40FF00     // Move to the next program (current option + 1)
#define ButtonPrev 0xBB44FF00     // Move to the previous program (current option - 1)
#define ButtonChMinus 0xBA45FF00  // when in state 2, decrease a side leg angle by 5 degrees; when in state 4, decrease amplitude on front legs, increase amplitude on back legs
#define ButtonCh 0xB946FF00       // Currently switch which side you are controlling the increase/decrease angle of
#define ButtonChPlus 0xB847FF00   // when in state 2, increase a side leg angle by 5 degrees; when in state 4, decrease amplitude on front legs, increase amplitude on back legs
#define ButtonPlayPause 0xBC43FF00  // reset the beat period to default value
#define ButtonEQ 0xF609FF00       // Used to reset the amplitude changes to currently controlled side

int maxState = 5; // the highest number of state currently used

// Store the servo specs
// calibration parameters for 2nd prototype
unsigned int servoPins[SERVOS*2] = {3,5,7,9,11,4,6,8,10,12}; // pins for all the servos, sorted with the first 5 being left legs, from P1 to P5 pleopods. Equivalent to [P1L P2L P3L P4L P5L P1R P2R P3R P4R P5R]
unsigned int minServoPulse[SERVOS*2] = {650,650,650,650,650,650,650,650,650,650}; // minimum servo pulse; same order as in servoPins
unsigned int maxServoPulse[SERVOS*2] = {2350,2350,2350,2350,2350,2350,2350,2350,2350,2350}; // maximum servo pulse; same order as in servoPins
float servoAngleRange[SERVOS*2] = {155.641,156.466,153.502,158.893,159.532,155.155,157.404,157.125,158.934,154.985}; // measured physical range of the servo for the respective min and max pulse. Requires initial calibration of each servo to determine their range. ; Same order as in servoPins
int corrFact[SERVOS*2] = {-30,45,20,45,-15,20,-30,35,-60,-40}; // correction factor after the servos are installed to align the pleopods perfectly vertically; same order as in servoPins
const unsigned int servoUpdatePeriodMillis = 20; // 20ms = 50Hz, the default for servo actuation
unsigned int beatPeriodMillisDefault = 1500; // default value of beat period in ms


// Initiate servos for the left and right legs - keep left and right servos separate for now
Servo Pleft[SERVOS]; // left legs
Servo Pright[SERVOS]; // right legs

void resetAmplitude(float amplitude_[], float amplitude_Stable[]){
  for(unsigned int i = 0; i < SERVOS*2; i++){ // Reset the amplitude changes so that the when you enter the motion program it will be default
    amplitude_[i] = amplitude_Stable[i];
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

// Re-compute the other timing parameters for the beat
void updateBeatPeriod(unsigned int &beat_Period_Steps, const unsigned int servo_Period_Millis, unsigned int beat_Period_Millis, float &phase_, unsigned int &period_Steps_Counter,
int beat_Step_Phase_Begin[], float phase_Lag){
  beat_Period_Steps = beat_Period_Millis / servo_Period_Millis; // re-calculate the number of kinematics steps for the new beat period
  period_Steps_Counter = ceil(phase_ / ((float)servo_Period_Millis / beat_Period_Millis)); // calculate the rounded-up integer value for the period step counter, relative to the counter step of the previous beat period value
  phase_ = ((float)servo_Period_Millis / beat_Period_Millis) * period_Steps_Counter;// recalculate the proper phase for the new counter
  for(int i = 0; i < SERVOS; i++){ // recalculate beatPeriodSteps at which each leg updates the amplitude in yaw control
    beat_Step_Phase_Begin[4-i] = int(floor(beat_Period_Steps*phase_Lag*i));
    beat_Step_Phase_Begin[9 - i] = int(floor(beat_Period_Steps*phase_Lag*i));
  }
  Serial.println(beat_Period_Steps);

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
void switchTurnLeftRight(float amplitude_[], float amplitude_stable[], float amplitude_changed[], int changing_index, int turning_Left_Right[]){ // the index of the amplitude array to update the amplitude
  if(turning_Left_Right[changing_index] == 1){ // update the amplitude to turn right
    amplitude_[changing_index] = amplitude_stable[changing_index]; // left leg goes back to default amplitude
    amplitude_[changing_index + SERVOS] = amplitude_changed[changing_index + SERVOS]; // right leg goes to an updated amplitude
    turning_Left_Right[changing_index] = 0;
    // Serial.print("P");
    // Serial.print(5 - changing_index);
    // Serial.println(" SWITCH TO RIGHT!!!!!!!!!!!!!!!!!!!!!!!!!!");
  } else if(turning_Left_Right[changing_index] == 0){ // update the amplitude to turn left
    amplitude_[changing_index] = amplitude_changed[changing_index]; // left leg goes to an updated amplitude
    amplitude_[changing_index + SERVOS] = amplitude_stable[changing_index + SERVOS]; // right goes back to default amplitude
    turning_Left_Right[changing_index] = 1;
    // Serial.print("P");
    // Serial.print(5 - changing_index);
    // Serial.println(" SWITCH TO LEFT!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
}

void switchPitchUpDown(float amplitude_[], float amplitude_stable[], float up_amplitude_changed[], float down_amplitude_changed[], int changing_index, int pitching_Up_Down[]){
  if(pitching_Up_Down[changing_index] == 0){ // update the amplitude to pitch up
    amplitude_[changing_index] = up_amplitude_changed[changing_index]; // front legs decrease in amplitude, back legs increase in amplitude
    amplitude_[changing_index + SERVOS] = up_amplitude_changed[changing_index + SERVOS]; // front legs decrease in amplitude, back legs increase in amplitude
    pitching_Up_Down[changing_index] = 1;
    // Serial.print("P");
    // Serial.print(5 - changing_index);
    // Serial.print(" PITCH UP!!!!!!!!!!!!!!!!!!!!!!!!!! ");
    // Serial.print(amplitude_[changing_index]);
    // Serial.print(":");
    // Serial.println(amplitude_[changing_index + SERVOS]);
  } else if(pitching_Up_Down[changing_index] == 1){ // update the amplitude to pitch down
    amplitude_[changing_index] = down_amplitude_changed[changing_index]; // front legs increase in amplitude, back legs decrease in amplitude
    amplitude_[changing_index + SERVOS] = down_amplitude_changed[changing_index + SERVOS]; // front legs increase in amplitude, back legs decrease in amplitude
    pitching_Up_Down[changing_index] = 0;
    // Serial.print("P");
    // Serial.print(5 - changing_index);
    // Serial.print(" PITCH DOWN!!!!!!!!!!!!!!!!!!!!!!!!!! ");
    // Serial.print(amplitude_[changing_index]);
    // Serial.print(":");
    // Serial.println(amplitude_[changing_index + SERVOS]);
  }
}

void optionIRRemote(unsigned int &beat_Period_Millis, unsigned int beat_Period_Millis_Incr, unsigned int &State, bool &option_Changed, float amplitude_[SERVOS*2],
      float amplitude_Stable[], unsigned int &left_Right_Control, int amp_Increment_Degree, int amp_Limit, unsigned int &beat_Period_Steps, int servo_Period_Millis, float phase_,
      unsigned int &period_Steps_Counter, int beat_Step_Phase_Begin[SERVOS*2], float phase_Lag){
  if(IrReceiver.decode()){  
    if(IrReceiver.decodedIRData.decodedRawData == Button0){ // Keep the legs horizontal
      State = 0;
      resetAmplitude(amplitude_, amplitude_Stable);
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button1){ // Keep the legs vertical for maintenance
      State = 1;
      resetAmplitude(amplitude_, amplitude_Stable);
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button2){ // Run the normal kinematics program
      State = 2;
      resetAmplitude(amplitude_, amplitude_Stable);
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button3){ // Run the yaw zigzag prgram
      State = 3;
      resetAmplitude(amplitude_, amplitude_Stable);
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button4){ // Run the controllable pitch program
      State = 4;
      resetAmplitude(amplitude_, amplitude_Stable);
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == Button5){ // Pitch controllable state
      State = 5;
      resetAmplitude(amplitude_, amplitude_Stable);
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonMinus){ // decrease the beat period to increase the beat frequency in increments of 100 ms
      if(beat_Period_Millis >= 200 + beat_Period_Millis_Incr && beat_Period_Millis <= 3000){ // if interval is > than the minimum allowed frequency + increment: this is done so the loop does not compute an interval less than the nminimum allowable 200 ms)
        beat_Period_Millis = beat_Period_Millis - beat_Period_Millis_Incr; // decrease the beat period
        option_Changed = true;
      }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonPlus){ // increase the beat period to decrease the beat frequency in increments of 100 ms
      if(beat_Period_Millis >= 200 && beat_Period_Millis <= 3000-beat_Period_Millis_Incr){
        beat_Period_Millis = beat_Period_Millis + beat_Period_Millis_Incr;
        option_Changed = true;
      }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonNext){ // Flip throught the options one by one in increasing numbers (does not include the straight legs option)    
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
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonPrev){ // Flip throught the options one by one in decreasing numbers (does not include the straight legs option)    
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
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonChMinus){ // decrease the amplitude of leg by ampIncrementDegree (5 for now)
      if(State == 2){
        for(unsigned int i = 0; i < SERVOS; i++){
          if((amplitude_[i+(left_Right_Control*SERVOS)] - amp_Increment_Degree) > 0){ // Check to make sure that we are not going to decrease below 0
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_[i+(left_Right_Control*SERVOS)] - amp_Increment_Degree;
          }
        }
      }
      else if(State == 4){
        if(amplitude_[SERVOS-1] - amplitude_Stable[SERVOS-1] < amp_Limit){
          for(int i = 0; i < SERVOS * 2; i++) {
            int offset = 2 - (i % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
            amplitude_[i] = amplitude_[i] - amp_Increment_Degree * offset;
          }
        }

      }
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonChPlus){ // decrease the amplitude of leg by ampIncrementDegree (5 for now)
      if(State == 2){
        for(unsigned int i = 0; i < SERVOS; i++){
          if((amplitude_[i+(left_Right_Control*SERVOS)] + amp_Increment_Degree) <= (amplitude_Stable[i+(left_Right_Control*SERVOS)] + 20)){ // Check to make sure that we are not going to increase too much above the original val
            amplitude_[i+(left_Right_Control*SERVOS)] = amplitude_[i+(left_Right_Control*SERVOS)] + amp_Increment_Degree;
          }
        }
      }
      else if(State == 4){
        if(amplitude_[0] - amplitude_Stable[0] < amp_Limit){
          for(int i = 0; i < SERVOS * 2; i++) {
            int offset = 2 - (i % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
            amplitude_[i] = amplitude_[i] + amp_Increment_Degree * offset;
          }
        }
      }
      // option_Changed = true;
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonCh){ // change the side we are decreasing/increasing the amplitude of
      left_Right_Control = left_Right_Control + 1;
      if(left_Right_Control >= 2){
        left_Right_Control = 0;
      }
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonPlayPause){ // reset the beat period to default value
      beat_Period_Millis = beatPeriodMillisDefault;
      updateBeatPeriod(beat_Period_Steps, servo_Period_Millis, beat_Period_Millis, phase_, period_Steps_Counter, beat_Step_Phase_Begin, phase_Lag);
    }

    else if(IrReceiver.decodedIRData.decodedRawData == ButtonEQ){ // Reset the amplitude changes to both sides
      resetAmplitude(amplitude_, amplitude_Stable);
    }

  IrReceiver.resume(); // this statement is needed to close the if statement and allow for new values to be read
  }
}