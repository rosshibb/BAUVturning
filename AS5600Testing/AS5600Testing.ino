#include "AS5600.h"
#include <Wire.h>
AS5600 as5600;   //  use default Wire
void setup()
{
  Serial.begin(9600);
  while (!Serial){};
  // Serial.println(__FILE__);
  // Serial.print("AS5600_LIB_VERSION: ");
  // Serial.println(AS5600_LIB_VERSION);
  Wire.begin(); //14, 15);
  Serial.print("beginning: ");
  Serial.println(as5600.begin(1));  //  set direction pin.
  //as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);
}
void loop()
{
  //   // Serial.println("hello");
  // // //  Serial.print(millis());
  // // //  Serial.print("\t");
  // Serial.print(as5600.readAngle());
  // Serial.print("\t");
  Serial.println(as5600.rawAngle());
  // // //  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
  // Wire.beginTransmission(0x36);
  // Serial.println(Wire.endTransmission()==0);
  delay(20);
}