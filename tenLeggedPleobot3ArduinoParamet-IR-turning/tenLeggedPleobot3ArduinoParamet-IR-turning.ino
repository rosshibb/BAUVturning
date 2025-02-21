#include "helpers.hpp" // helper functions

const unsigned int servoPeriodMillis = 20; // time to update the servo position given by servo.write 20ms = 50Hz, the default for servo actuation
float pulleyRatio = 1.5; // pulley ratio between the servo and the protopodite. WARNING: technically the pulley ratio is calculated as the ratio between the driven and driving pulleys, giving 0.66 not 1.5. 

// Swimming parameters
float phaseStart = 0.0; // starting time of the beat
float phase = 0.0; // specific time during the beat period
unsigned int beatPeriodMillis = 1500; // duration of the beat period in millisecond; Enter a value in multiples of servoPeriodMillis to ensure that there will be no drift over time because the last step within a beat is <20 ms. In this case, this corresponds to a difference between periods of only 0.02Hz.
unsigned int beatPeriodMillisIncr = 100; // increment to increase and decrease the beat period duration to control the beat frequency

// Wave parameters
float amplitudeStable[SERVOS*2] = {59.0733, 67.0363, 72.3134, 84.0342, 86.127, 59.0733, 67.0363, 72.3134, 84.0342, 86.1277}; // total amplitude of the leg, not to be updated; same order as in servoPins
float amplitude[SERVOS*2] = {59.0733, 67.0363, 72.3134, 84.0342, 86.127, 59.0733, 67.0363, 72.3134, 84.0342, 86.1277}; // total amplitude of the leg; same order as in servoPins
float minAlpha[SERVOS*2] = {18.7606, 19.3838, 21.9856, 25.8914, 32.5847, 18.7606, 19.3838, 21.9856, 25.8914, 32.5847}; // minimum alpha, equivalent to alpha'; same order as in servoPins
float tempAsym[SERVOS*2] = {0.5370, 0.4905, 0.5272, 0.5000, 0.4545, 0.5370, 0.4905, 0.5272, 0.5000, 0.4545}; // temporal asymmetry in t/T; same order as in servoPins
float dPS[SERVOS*2] = {0.7500, 0.7989, 0.7675, 0.8142, 0.9899, 0.7500, 0.7989, 0.7675, 0.8142, 0.9899}; // shape of the power stroke - ascending cuve; same order as in servoPins
float dRS[SERVOS*2] = {0.7500, 0.7690, 0.8312, 0.9028, 1.0051, 0.7500, 0.7690, 0.8312, 0.9028, 1.0051}; // shape of the recovery stroke - descending curve; same order as in servoPins
const float phaseLag = 0.18; // interpleopod phase lag. Assumes that the lag between the legs is the same Pn--Pn-1 is the same

// Servo motions parameters
unsigned long lastLoopTimeMillis; // used to compute the duration of each loop and ensure thatit last 20ms (see servoPeriodMillis)
unsigned int beatPeriodSteps = beatPeriodMillis / servoPeriodMillis; // number of steps required to perform a full period (based on the period duration and time required to move the servos)
float alphaAngleDegree[SERVOS*2]; // alpha angle used to compute the pulse width input to the servos at each kinematics step
unsigned int periodStepsCounter = 0; // counter that changes with every servo steps of duration servoPeriodMillis

// turning parameters
int turningLeftRight[] = {0,0,0,0,0}; // tells which way each leg is turning; left -> 0, right -> 1; e.g. {1,1,0,0,0} P1 and P2 are turning right, P3-5 are turning left
int yawCurrentStrokeCount[] = {0,0,0,0,0}; // how many period of strokes each leg completed, resets to 0 at turningStrokeCount; e.g. {2,2,1,1,1} P1 and P2 completed 2 cycles, P3-5 completed 1
float yawAmplitudeChanged[SERVOS*2]; // the amplitude of the turning side legs for yaw zigzag motion
unsigned int leftRightControl = 0; // for controlling altering the amplitude of either side. Starts at 0 -> Left. Will be able to switch to 1 -> Right, with press of the CH button.

// pitching parameters
int pitchingUpDown[] = {0,0,0,0,0}; // tells which way robot is pitching; up -> 0, down -> 1; e.g. {1,1,0,0,0} P1 and P2 are pitching , P3-5 are pitching up
int pitchCurrentStrokeCount[] = {0,0,0,0,0}; // how many period of strokes each leg completed, resets to 0 at turningStrokeCount; e.g. {2,2,1,1,1} P1 and P2 completed 2 cycles, P3-5 completed 1
int turningStrokeCount = 3; // stroke count before switching the turning side 
float pitchAmplitudeChanged[SERVOS*2]; // the amplitude when pitching in pitch zigzag motion
unsigned int frontBackControl = 0; // for controlling altering the amplitude of the front and back. Starts at 0 -> Front. Will be able to switch to 1 -> Back, with press of the CH button when in state 5.

int beatStepPhaseBegin[SERVOS*2]; // the periodStepsCounter at which each leg can update the amplitude to the lower value
int ampIncrementDegree = 5; // increment to increase and decrease the amplitude of leg for turning
const int ampLimit = 30; // limit of amplitude difference from the default setting in pitch control

// trait investigator parameters
int trait = 0; // specifies which trait to check; 0->yaw offset angle, 1->zigzag number, 2-> pitch offset angle, 4-> period
// int binaryArray[SERVOS] = {1,0,0,0,0}; // represents the quantity associated with each trait, first digit represents the sign, following 4 the number

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


for(int i = 0; i < SERVOS*2; i++){ // turning side has 20% of amplitude compared to the leading side
  yawAmplitudeChanged[i] = amplitudeStable[i];
}

for(int i = 0; i < SERVOS*2; i++){
  int offset = 2 - (i % SERVOS);  // Maps i = {0,5} → 2, {1,6} → 1, ..., {4,9} → -2
  pitchAmplitudeChanged[i] = amplitudeStable[i] + 15 * offset;
}

for(int i = 0; i < SERVOS; i++){
  beatStepPhaseBegin[4-i] = int(floor(beatPeriodSteps*phaseLag*i));
  beatStepPhaseBegin[9-i] = int(floor(beatPeriodSteps*phaseLag*i));
}

attachServos(); // initialize the servos (servo.attach)
initializeMotion(lastLoopTimeMillis,phaseStart,phase); // initialize the motion of the legs (set phase = 0, enable the motors)
// lastLoopTimeMillis = 0;
alphaAngleDeg(phase, amplitude, minAlpha, tempAsym, dPS, dRS, phaseLag, alphaAngleDegree);// initialize the first alpha degree point);// initialize the first alpha degree point
}


void loop() {
// total of 5 options: 0-> default, legs in horizontal position; 1-> legs in vertical position; 2-> kinematics program; 3-> swim in yaw zigzag; 4-> swim with controllable pitch; 5-> swim with pitch zigzag (to be implemented)
optionIRRemote(beatPeriodMillis, beatPeriodMillisIncr, state, optionChanged, amplitude, amplitudeStable, leftRightControl, ampIncrementDegree, ampLimit, beatPeriodSteps, 
      servoPeriodMillis, phase, periodStepsCounter, beatStepPhaseBegin, phaseLag, trait, turningStrokeCount, yawAmplitudeChanged, binarySwitch, yawCounterL, yawCounterR, beatPeriodMillisDefault,
      pitchCounter, pitchIncrementDegree, lastLoopTimeMillis, yawCurrentStrokeCount, pitchCurrentStrokeCount, turningLeftRight, pitchingUpDown);

// Trait Changing (Binary Look) 
if(state == 0){
  unsigned long currentMillis = millis(); // initiate the start time for the loop to ensure that each iteration of the code takes servoPeriodMillis seconds (here 20 ms)
  if(currentMillis - lastLoopTimeMillis >= servoPeriodMillis){
    // Serial.print(currentMillis - lastLoopTimeMillis);
    lastLoopTimeMillis = currentMillis; // reset the millis counter to keep track of the time before sending instructions for a new step
    if(periodStepsCounter < beatPeriodSteps){
      // Move the servo using the alpha for this corresponding phase that was pre-computed in the previous loop
      servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaAngleDegree); //TODO USE

      // Compute the alpha angle for the next period phase while the 20ms required to move the servo elapse
      // checks if the serial monitor senses a change which corespond to a new beat period value
      // if (Serial.available() > 0){ 
      if (optionChanged == true){
        // setBeatPeriod(servoPeriodMillis, beatPeriodMillis); // changes the beat period beatPeriodMillis to a new value (multiple of 20 ms) using input from the serial monitor.
        updateBeatPeriod(beatPeriodSteps, servoPeriodMillis, beatPeriodMillis, phase, periodStepsCounter, beatStepPhaseBegin, phaseLag); // update the counter and phase to ensure a snooth transition from the previous step with different beat period
        optionChanged = false; // re-set the option changed variable
      }
      else{
      // Compute the alpha angle for the next period phase while the 20ms required to move the servo elapse
        periodStepsCounter += 1; // increase the counter by one step
        phase = phaseStart + ((float)servoPeriodMillis / beatPeriodMillis) * periodStepsCounter; // calculate the phase for the specific step within a beat; ranges from 0 to 1.
      }
      // alphaAngleDegree = alphaAngleDeg(phase); // embedded within the write servo position function
       //TODO CHANGE
      displayTrait(trait, yawAmplitudeChanged, amplitudeStable, ampIncrementDegree, binaryArray, alphaAngleDegree, turningStrokeCount, pitchAmplitudeChanged, beatPeriodMillisIncr, beatPeriodMillis, beatPeriodMillisDefault, phase, binarySwitch, leftRightControl, yawCounterL, yawCounterR, pitchCounter);

      if (periodStepsCounter == beatPeriodSteps){
        periodStepsCounter = 0;
      }
    }
  }
}
// Keep the legs vertical for maintenance
else if(state == 1){
  lastLoopTimeMillis = millis();
  float tempAlphaHoriz = 90; // angle in degrees that each leg is stored at for vertical
  float alphaHorizAngleDegree[SERVOS*2] = {tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz};
  servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaHorizAngleDegree);
  // Serial.println(state);
  for(unsigned int i = 0; i < SERVOS*2; i++){ // Reset the amplitude changes so that the when you enter the motion program it will be default
    amplitude[i] = amplitudeStable[i];
  }
}
// Run the normal kinematics program
else if(state == 2){
 unsigned long currentMillis = millis(); // initiate the start time for the loop to ensure that each iteration of the code takes servoPeriodMillis seconds (here 20 ms)
  if(currentMillis - lastLoopTimeMillis >= servoPeriodMillis){
    // Serial.print(currentMillis - lastLoopTimeMillis);
    lastLoopTimeMillis = currentMillis; // reset the millis counter to keep track of the time before sending instructions for a new step
    if(periodStepsCounter < beatPeriodSteps){
      // Move the servo using the alpha for this corresponding phase that was pre-computed in the previous loop
      servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaAngleDegree);

      // Compute the alpha angle for the next period phase while the 20ms required to move the servo elapse
      // checks if the serial monitor senses a change which corespond to a new beat period value
      // if (Serial.available() > 0){ 
      if (optionChanged == true){
        // setBeatPeriod(servoPeriodMillis, beatPeriodMillis); // changes the beat period beatPeriodMillis to a new value (multiple of 20 ms) using input from the serial monitor.
        updateBeatPeriod(beatPeriodSteps, servoPeriodMillis, beatPeriodMillis, phase, periodStepsCounter, beatStepPhaseBegin, phaseLag); // update the counter and phase to ensure a snooth transition from the previous step with different beat period
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
// Run the yaw zigzag program
else if(state == 3){
  unsigned long currentMillis = millis(); // initiate the start time for the loop to ensure that each iteration of the code takes servoPeriodMillis seconds (here 20 ms)
  if(currentMillis - lastLoopTimeMillis >= servoPeriodMillis){
    lastLoopTimeMillis = currentMillis; // reset the millis counter to keep track of the time before sending instructions for a new step
    if(periodStepsCounter < beatPeriodSteps){
      servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaAngleDegree);

      for(int i = 0; i < SERVOS; i++){
        if(periodStepsCounter == beatStepPhaseBegin[i]){
          yawCurrentStrokeCount[i]++;
          Serial.print("yawCurrentStrokeCount");
          Serial.print(i);
          Serial.print(" : ");

          Serial.println(yawCurrentStrokeCount[i]);
          
          if (yawCurrentStrokeCount[i] == turningStrokeCount){
            switchTurnLeftRightBinaryComp(yawCounterL, yawCounterR, amplitude, amplitudeStable, yawAmplitudeChanged, i, turningLeftRight, ampIncrementDegree);
            yawCurrentStrokeCount[i] = 0;

          }

          for(i = 0; i < SERVOS; i++){
            Serial.print(beatStepPhaseBegin[i]);
            Serial.print(",");
          }

          // Serial.println("");

          // Serial.print("yawCurrentStrokeCount");
          // Serial.print(i);
          // Serial.print(" ");
          // Serial.println(yawCurrentStrokeCount[i]);
        }
      }
      
      if(optionChanged == true){
        updateBeatPeriod(beatPeriodSteps, servoPeriodMillis, beatPeriodMillis, phase, periodStepsCounter, beatStepPhaseBegin, phaseLag);
        optionChanged = false;
      }
      else{
        periodStepsCounter += 1; // increase the counter by one step
        phase = phaseStart + ((float)servoPeriodMillis / beatPeriodMillis) * periodStepsCounter; // calculate the phase for the specific step within a beat; ranges from 0 to 1.
      }
      alphaAngleDeg(phase, amplitude, minAlpha, tempAsym, dPS, dRS, phaseLag, alphaAngleDegree); // embedded within the write servo position function

      if (periodStepsCounter == beatPeriodSteps){
        periodStepsCounter = 0;
      }
    }

  }
  // Serial.println(beatPeriodSteps);
}
// Run the controllable pitch program
else if(state == 4){
   unsigned long currentMillis = millis();
  if(currentMillis - lastLoopTimeMillis >= servoPeriodMillis){
    lastLoopTimeMillis = currentMillis;

    if(periodStepsCounter < beatPeriodSteps){
      servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaAngleDegree);
      if (optionChanged == true){
        updateBeatPeriod(beatPeriodSteps, servoPeriodMillis, beatPeriodMillis, phase, periodStepsCounter, beatStepPhaseBegin, phaseLag);
        optionChanged = false; // re-set the option changed variable
      }
      else{
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
// Run the pitch zigzag program
else if(state == 5){
  unsigned long currentMillis = millis();
  if(currentMillis - lastLoopTimeMillis >= servoPeriodMillis){
    lastLoopTimeMillis = currentMillis; // reset the millis counter to keep track of the time before sending instructions for a new step
    if(periodStepsCounter < beatPeriodSteps){
      servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaAngleDegree);

      for(int i = 0; i < SERVOS; i++){
        if(periodStepsCounter == beatStepPhaseBegin[i]){
          pitchCurrentStrokeCount[i]++;
          if (pitchCurrentStrokeCount[i] == turningStrokeCount){
            // switchPitchUpDown(amplitude, amplitudeStable, pitchAmplitudeChanged, i, pitchingUpDown);
            switchPitchUpDownBinaryComp(amplitude, amplitudeStable, pitchAmplitudeChanged, i, pitchingUpDown, pitchCounter, pitchIncrementDegree);

            pitchCurrentStrokeCount[i] = 0;
          }
          for(i = 0; i < SERVOS; i++){
            // Serial.print(beatStepPhaseBegin[i]);
            // Serial.print(",");
          }

        }
      }
      
      if(optionChanged == true){
        updateBeatPeriod(beatPeriodSteps, servoPeriodMillis, beatPeriodMillis, phase, periodStepsCounter, beatStepPhaseBegin, phaseLag);
        optionChanged = false;
      }
      else{
      periodStepsCounter += 1; // increase the counter by one step
      phase = phaseStart + ((float)servoPeriodMillis / beatPeriodMillis) * periodStepsCounter; // calculate the phase for the specific step within a beat; ranges from 0 to 1.
      }
      alphaAngleDeg(phase, amplitude, minAlpha, tempAsym, dPS, dRS, phaseLag, alphaAngleDegree); // embedded within the write servo position function

      if (periodStepsCounter == beatPeriodSteps){
        periodStepsCounter = 0;
      }
    }
  }
}
// Keep the legs horizontal
else if(state == 6){
  // lastLoopTimeMillis = millis();
  float tempAlphaHoriz = 25.0; // angle in degrees that each leg is stored at for transport
  float alphaHorizAngleDegree[SERVOS*2] = {tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz, tempAlphaHoriz};
  servoPosMicro = writeServoPosition(pulleyRatio, minServoPulse, maxServoPulse, servoAngleRange, corrFact, alphaHorizAngleDegree);
  initializeMotion(lastLoopTimeMillis,phaseStart,phase); // initialize the motion of the legs (set phase = 0, enable the motors)
  periodStepsCounter = 0;
  alphaAngleDeg(phase, amplitude, minAlpha, tempAsym, dPS, dRS, phaseLag, alphaAngleDegree);
  for(unsigned int i = 0; i < SERVOS*2; i++){ // Reset the amplitude changes so that the when you enter the motion program it will be default
    amplitude[i] = amplitudeStable[i];
  }
}
// Check specified trait; first digit of binaryArray specifies the sign, 4 following digits the number in binary


// Reset state to servo disabled
else{
  state = 0;
}
}
