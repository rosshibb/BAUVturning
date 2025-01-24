// Required libraries
#include <Servo.h> 
#include <IRremote.hpp>   // IR remote control library

#define SERVOS 5 // the number of servos on each side of the body

// Define IR remote keys for specific programs
#define Button0 0xE916FF00        // default option, keep the legs horizontal
#define Button1 0xF30CFF00        // Normal program
#define Button2 0xE718FF00        // Perform horizontal zig-zag pogram
#define Button3 0xA15EFF00        // Keep the legs vertical for maintenance
#define Button4 0xF708FF00        // Currently unused
#define Button5 0xE31CFF00        // Currently unused
#define Button6 0xA55AFF00        // Currently unused
#define Button7 0xBD42FF00        // Currently unused
#define Button8 0xAD52FF00        // Currently unused
#define ButtonPlus 0xEA15FF00     // increase the frequency -- Volume Plus on remote
#define ButtonMinus 0xF807FF00    // decrease the frequency -- Volume Minus on remote
#define ButtonNext 0xBF40FF00     // Move to the next program (current option + 1)
#define ButtonPrev 0xBB44FF00     // Move to the previous program (current option - 1)
#define ButtonChMinus 0xBA45FF00  // Currently unused
#define ButtonCh 0xB946FF00       // Currently unused
#define ButtonChPlus 0xB847FF00   // Currently unused
#define ButtonPlayPause 0xBC43FF00  // Currently unused
#define ButtonEQ 0xF609FF00       // Currently unused

// Store the servo specs
// calibration parameters for 2nd prototype
unsigned int servoPins[SERVOS*2] = {3,5,7,9,11,4,6,8,10,12}; // pins for all the servos, sorted with the first 5 being left legs, from P1 to P5 pleopods. Equivalent to [P1L P2L P3L P4L P5L P1R P2R P3R P4R P5R]
unsigned int minServoPulse[SERVOS*2] = {650,650,650,650,650,650,650,650,650,650}; // minimum servo pulse; same order as in servoPins
unsigned int maxServoPulse[SERVOS*2] = {2350,2350,2350,2350,2350,2350,2350,2350,2350,2350}; // maximum servo pulse; same order as in servoPins
float servoAngleRange[SERVOS*2] = {155.641,156.466,153.502,158.893,159.532,155.155,157.404,157.125,158.934,154.985}; // measured physical range of the servo for the respective min and max pulse. Requires initial calibration of each servo to determine their range. ; Same order as in servoPins
int corrFact[SERVOS*2] = {-30,45,20,45,-15,20,-30,35,-60,-40}; // correction factor after the servos are installed to align the pleopods perfectly vertically; same order as in servoPins
const unsigned int servoUpdatePeriodMillis = 20; // 20ms = 50Hz, the default for servo actuation

const unsigned int servoPeriodMillis = 20; // time to update the servo position given by servo.write 20ms = 50Hz, the default for servo actuation
float pulleyRatio = 1.5; // pulley ratio between the servo and the protopodite. WARNING: technically the pulley ratio is calculated as the ratio between the driven and driving pulleys, giving 0.66 not 1.5. 

// Initiate servos for the left and right legs - keep left and right servos separate for now
Servo Pleft[SERVOS]; // left legs
Servo Pright[SERVOS]; // right legs

// Swimming parameters
// unsigned long tStartMillis; // stores the timer value
float phaseStart = 0.0; // starting time of the beat
float phase = 0.0; // specific time during the beat period
unsigned int beatPeriodMillis = 1500; // duration of the beat period in millisecond; Enter a value in multiples of servoPeriodMillis to ensure that there will be no drift over time because the last step within a beat is <20 ms. In this case, this corresponds to a difference between periods of only 0.02Hz.
unsigned int beatPeriodMillisIncr = 100; // increment to increase and decrease the beat period duration to control the beat frequency

// Wave parameters
float amplitude[SERVOS*2] = {59.0733, 67.0363, 72.3134, 84.0342, 86.127, 59.0733, 67.0363, 72.3134, 84.0342, 86.1277}; // total amplitude of the leg; same order as in servoPins
float minAlpha[SERVOS*2] = {18.7606, 19.3838, 21.9856, 25.8914, 32.5847, 18.7606, 19.3838, 21.9856, 25.8914, 32.5847}; // minimum alpha, equivalent to alpha'; same order as in servoPins
float tempAsym[SERVOS*2] = {0.5370, 0.4905, 0.5272, 0.5000, 0.4545, 0.5370, 0.4905, 0.5272, 0.5000, 0.4545}; // temporal asymmetry in t/T; same order as in servoPins
float dPS[SERVOS*2] = {0.7500, 0.7989, 0.7675, 0.8142, 0.9899, 0.7500, 0.7989, 0.7675, 0.8142, 0.9899}; // shape of the power stroke - ascending cuve; same order as in servoPins
float dRS[SERVOS*2] = {0.7500, 0.7690, 0.8312, 0.9028, 1.0051, 0.7500, 0.7690, 0.8312, 0.9028, 1.0051}; // shape of the recovery stroke - descending curve; same order as in servoPins
float phaseLag = 0.18; // interpleopod phase lag. Assumes that the lag between the legs is the same Pn--Pn-1 is the same

// Servo motions parameters
unsigned long lastLoopTimeMillis; // used to compute the duration of each loop and ensure thatit last 20ms (see servoPeriodMillis)
unsigned int beatPeriodSteps = beatPeriodMillis / servoPeriodMillis; // number of steps required to perform a full period (based on the period duration and time required to move the servos)
float alphaAngleDegree[SERVOS*2]; // alpha angle used to compute the pulse width input to the servos at each kinematics step
unsigned int periodStepsCounter = 0; // counter that changes with every servo steps of duration servoPeriodMillis


// Switching options
unsigned int state = 0; // condition controlling the options. Goes up with the push of a switch and resets to 0 once all the options have been cycles through
unsigned int switchPin = 2; // pin for the options switch (may not work with the IR sensor)
bool optionChanged = false; // check whether the options were changed.This is needed to ensure smooth transitions in the frequency function when increasing or decreasing the beat period

// Read the serial monitor to change the beat frequency
String readString;

// Debugging parameters
unsigned int servoPosMicro;


void setup() {
Serial.begin(9600); // Let's use the serial monitor for debugging
// pinMode(switchPin,INPUT); // pint for the temporary switch to toggle through options
IrReceiver.begin(switchPin, ENABLE_LED_FEEDBACK); // set up the IR receiver

attachServos(); // initialize the servos (servo.attach)
initializeMotion(lastLoopTimeMillis,phaseStart,phase); // initialize the motion of the legs (set phase = 0, enable the motors)
// lastLoopTimeMillis = 0;
alphaAngleDeg(phase, amplitude, minAlpha, tempAsym, dPS, dRS, phaseLag, alphaAngleDegree);// initialize the first alpha degree point);// initialize the first alpha degree point
}


void loop() {
// read the pin value for the switch controlling the options
// total of 3 options: 0 = the legs are moved to their resting horizontal position; 1 = everything is disabled, the legs are kept vertical; 2 = run the kinematics motion program;  
// optionSwitch(switchPin, state);
optionIRRemote(beatPeriodMillis, beatPeriodMillisIncr, state, optionChanged);
// Serial.print(state);
// Serial.print(",");
// Serial.println(beatPeriodMillis);


// maintain the legs in horizontal resting position
if(state == 0){
  // lastLoopTimeMillis = millis();
  float tempAlphaHoriz = 25.0; // angle in degrees that each leg is stored at for transport
  float alphaHorizAngleDegree[SERVOS*2] = {tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz};
  servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaHorizAngleDegree);
  initializeMotion(lastLoopTimeMillis,phaseStart,phase); // initialize the motion of the legs (set phase = 0, enable the motors)
  periodStepsCounter = 0;
  alphaAngleDeg(phase, amplitude, minAlpha, tempAsym, dPS, dRS, phaseLag, alphaAngleDegree);
// Serial.println(state);
}

// run the kinematics program
else if(state == 2){
 unsigned long currentMillis = millis(); // initiate the start time for the loop to ensure that each iteration of the code takes servoPeriodMillis seconds (here 20 ms)

      if(currentMillis - lastLoopTimeMillis >= servoPeriodMillis){
        // Serial.print(currentMillis - lastLoopTimeMillis);
        lastLoopTimeMillis = currentMillis; // reset the millis counter to keep track of the time before sending instructions for a new step

        if(periodStepsCounter < beatPeriodSteps){

          // Move the servo using the alpha for this corresponding phase that was pre-computed in the previous loop
          servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaAngleDegree);
          // Serial.print(",");
          // Serial.print(periodStepsCounter);
          // Serial.print(",");
          // Serial.println(phase);
          // Serial.println(alphaAngleDegree);
          // Serial.print(servoPosMicro);
          // Serial.print(alphaAngleDegree[9]);
          // Serial.print(",");
          // Serial.print(alphaAngleDegree[9]);
          // Serial.print(",");
          // Serial.println(beatPeriodMillis);

          // Compute the alpha angle for the next period phase while the 20ms required to move the servo elapse
          // checks if the serial monitor senses a change which corespond to a new beat period value
          // if (Serial.available() > 0){ 
          if (optionChanged == true){
            // setBeatPeriod(servoPeriodMillis, beatPeriodMillis); // changes the beat period beatPeriodMillis to a new value (multiple of 20 ms) using input from the serial monitor.
            updateBeatPeriod(beatPeriodSteps, servoPeriodMillis, beatPeriodMillis, phase, periodStepsCounter); // update the counter and phase to ensure a snooth transition from the previous step with different beat period
            optionChanged = false; // re-set the option changed variable
          }

          else{
          // Compute the alpha angle for the next period phase while the 20ms required to move the servo elapse
          periodStepsCounter += 1; // increase the counter by one step
          phase = phaseStart + ((float)servoPeriodMillis / beatPeriodMillis) * periodStepsCounter; // calculate the phase for the specific step within a beat; ranges from 0 to 1.

          }
          // alphaAngleDegree = alphaAngleDeg(phase); // embedded within the write servo position function
          alphaAngleDeg(phase, amplitude, minAlpha, tempAsym, dPS, dRS, phaseLag, alphaAngleDegree); // embedded within the write servo position function

          if (periodStepsCounter == beatPeriodSteps){
            periodStepsCounter = 0;
          }
        }
      }
}


// maintain the legs vertical
else if(state == 1){
  lastLoopTimeMillis = millis();
  float tempAlphaHoriz = 90; // angle in degrees that each leg is stored at for vertical
  float alphaHorizAngleDegree[SERVOS*2] = {tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz};
  servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaHorizAngleDegree);
Serial.println(state);
}

// Reset state to servo disabled
else{
  state = 0;
}

}


// CUSTOM FUNCTIONS
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

// Read the pin value for the switch controling the options
// void optionSwitch(unsigned int switch_Pin, unsigned int &State){
// if(digitalRead(switch_Pin) == HIGH){ // total of 4 options: 0 = everything is disabled; 1 = run the kinematics motion program
//     State+=1; // increase the value by +1 to switch between options
//       while(digitalRead(switch_Pin) == HIGH)
//       {}  
//   }
// }

// Read the values output by the IR remote controlling the program options
void optionIRRemote(unsigned int &beat_Period_Millis, unsigned int beat_Period_Millis_Incr, unsigned int &State, bool &option_Changed){
  if(IrReceiver.decode()){  
  if(IrReceiver.decodedIRData.decodedRawData == Button0){ // default option, keep the legs horizontal
    State = 0;
    // option_Changed = true;
  }

  else if(IrReceiver.decodedIRData.decodedRawData == Button1){ // Run the normal kinematics program - steady forward swimming
    State = 1;
    // option_Changed = true;
  }

  else if(IrReceiver.decodedIRData.decodedRawData == Button2){ // Run the horizontal zig-zag pogram
  State = 2;
  // option_Changed = true;
  }

  else if(IrReceiver.decodedIRData.decodedRawData == Button3){ // Keep the legs vertical to perform maintenance
  State = 3;
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
    if(State >= 0 && State <= 2){
      if (State == 2){ // check that the state does not go beyond 2, otherwise re-set to 0 and stat a new option loop
        State = 0;
        // option_Changed = true;
      }

      else{
        State+=1; // increase the value by +1 to switch between options
        // option_Changed = true;
      }
    }
  }

  else if(IrReceiver.decodedIRData.decodedRawData == ButtonPrev){ // Flip throught the options one by one in decreasing numbers (does not include the straight legs option)    
    if(State >= 0 && State <= 2){
      if (State == 0){ // check that the state does not go below 0, otherwise re-set to 2 and start a new option loop
        State = 2;
        // option_Changed = true;
      }

      else{
        State-=1; // increase the value by +1 to switch between options
        // option_Changed = true;
      }
    }
  }

IrReceiver.resume(); // this statement is needed to close the if statement and allow for new values to be read
}
}


// Kinematics equation that generates the angle for the corresponding phase of the beat - added after phase calculation
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

// Fetch the new beat period from the serial monitor and reset the kinematics parameters to produce the leg motions with the new beat frequency
// void setBeatPeriod(const unsigned int servo_Period_Millis, unsigned int &beat_Period_Millis){
//     while (Serial.available()) {
//     delay(1);  
//     if (Serial.available() >0) {
//       char c = Serial.read();  //gets one byte from serial buffer
//       readString += c; //makes the string readString
//     } 
//   }

//   if (readString.length() >0) {
//     int n;
//     char carray[6]; //converting string to number
//     readString.toCharArray(carray, sizeof(carray));
//     n = atoi(carray); 
//     readString="";
//       // check that the entered value falls within appropriate criteria
//       if(n >= 200 && n % servo_Period_Millis == 0){
//         beat_Period_Millis = n; // store and return the new value for the beat peariod
//       }
//   } 

// }

// Re-compute the other timing parameters for the beat
void updateBeatPeriod(unsigned int &beat_Period_Steps, const unsigned int servo_Period_Millis, unsigned int beat_Period_Millis, float &phase_, unsigned int &period_Steps_Counter){
  beat_Period_Steps = beat_Period_Millis / servo_Period_Millis; // re-calculate the number of kinematics steps for the new beat period
  period_Steps_Counter = ceil(phase_ / ((float)servo_Period_Millis / beat_Period_Millis)); // calculate the rounded-up integer value for the period step counter, relative to the counter step of the previous beat period value
  phase_ = ((float)servo_Period_Millis / beat_Period_Millis) * period_Steps_Counter;// recalculate the proper phase for the new counter

}