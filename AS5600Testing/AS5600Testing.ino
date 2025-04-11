//
//    FILE: AS5600_demo_radians.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.


#include "AS5600.h"
#define RAD_TO_DEG 57.295779513082320876798154814105
# define NUM_READINGS 10

//  Uncomment the line according to your sensor type
// AS5600L as5600;   //  use default Wire
AS5600 as5600;   //  use default Wire

int i = 0;
float readings[NUM_READINGS];

void setup()
{
  while(!Serial);
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.println();

  Wire.begin();

  as5600.begin();  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
}


void loop()
{

  float angle = (as5600.rawAngle() * AS5600_RAW_TO_RADIANS);
  // Store the reading
  readings[i] = angle;
  i = (i + 1) % NUM_READINGS;
  
  // Average using sin/cos method to handle angle wraparound
  float sum =0;
  for(int j = 0; j < NUM_READINGS; j++) {
    sum += readings[j];
  }

  float avgAngle = sum/NUM_READINGS; // radians
  float avgAngleDeg = avgAngle * RAD_TO_DEG;

  if (avgAngleDeg < 0) {
    avgAngleDeg += 360; // Normalize to 0–360°
  }

  Serial.print("Average angle: ");
  Serial.println(avgAngleDeg);


  // Serial.print(millis());
  // Serial.print("\t");
  // Serial.print(as5600.readAngle());
  // Serial.print("\t");
  // Serial.println(as5600.rawAngle() * AS5600_RAW_TO_RADIANS);
  if (i==NUM_READINGS){
    i = 0;
  }
  delay(20);
}


//  -- END OF FILE --
