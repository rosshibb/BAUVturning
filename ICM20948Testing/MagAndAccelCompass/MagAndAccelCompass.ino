#include <ICM_20948.h>

#define AD0_VAL 1   // last bit of I2C address

ICM_20948_I2C myicm;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {};

  Wire.begin();
  Wire.setClock(400000);

  myicm.begin(Wire, AD0_VAL);
}

void loop()
{
  if (myicm.dataReady())
  {
    myicm.getAGMT();
    float ax = -myicm.accX();  // these calibration offsets are for MY sensor, you must substitute offsets for YOUR sensor
    float ay =  myicm.accY();
    float az = -myicm.accZ();
    float gx = -myicm.gyrX()         - 0.400;
    float gy =  myicm.gyrY()         - 0.730;
    float gz = -myicm.gyrZ()         + 0.300;
    float mx = -myicm.magX()         -  5.00;
    float my =  myicm.magY()         -  2.50;
    float mz = -myicm.magZ()         -  9.50;

    float roll    = atan2(-ay, -az);                    // positive is right wing up
    float pitch   = atan2(+ax, sqrt(ay*ay + az*az));    // positive is nose down
    // float heading = atan2(-my*cos(roll) + mz*sin(roll), mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll));  // positive is nose right
    float heading = atan2(-my, mx);

    // Serial.print("Orientation: ");

    // Serial.print(ax,ay,az);


    Serial.print(-180); Serial.print(" ");
    Serial.print(180/PI*heading);  Serial.print(" ");
    Serial.print(180/PI*roll);     Serial.print(" ");
    Serial.print(180/PI*pitch);    Serial.print(" ");;
    Serial.print(180); Serial.println();
  }
}