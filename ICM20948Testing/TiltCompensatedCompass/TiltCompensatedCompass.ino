#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20948.h>
#include <math.h>
float thetaM;
float phiM;
float thetaFold=0;
float thetaFnew;
float phiFold=0;
float phiFnew;

float thetaG=0;
float phiG=0;

float theta;
float phi;

float thetaRad;
float phiRad;

float Xm;
float Ym;
float psi;


float dt;
unsigned long millisOld;


Adafruit_ICM20948 myIMU;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myIMU.begin_I2C();
delay(1000);
millisOld=millis();
}

void loop() {
  // put your main code here, to run repeatedly:

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;
myIMU.getEvent(&accel, &gyro, &temp, &mag);

thetaM=-atan2(accel.acceleration.x/9.8,accel.acceleration.z/9.8)/2/3.141592654*360;
phiM=-atan2(accel.acceleration.y/9.8,accel.acceleration.z/9.8)/2/3.141592654*360;
phiFnew=.95*phiFold+.05*phiM;
thetaFnew=.95*thetaFold+.05*thetaM;

dt=(millis()-millisOld)/1000.;
millisOld=millis();
theta=(theta+gyro.gyro.y*dt)*.95+thetaM*.05;
phi=(phi-gyro.gyro.x*dt)*.95+ phiM*.05;
thetaG=thetaG+gyro.gyro.y*dt;
phiG=phiG-gyro.gyro.x*dt;

phiRad=phi/360*(2*3.14);
thetaRad=theta/360*(2*3.14);

Xm=mag.magnetic.x*cos(thetaRad)-mag.magnetic.y*sin(phiRad)*sin(thetaRad)+mag.magnetic.z*cos(phiRad)*sin(thetaRad);
Ym=mag.magnetic.y*cos(phiRad)+mag.magnetic.z*sin(phiRad);

psi=atan2(Ym,Xm)/(2*3.14)*360;

Serial.print(accel.acceleration.x/9.8);
Serial.print(",");
Serial.print(accel.acceleration.y/9.8);
Serial.print(",");
Serial.print(accel.acceleration.z/9.8);
// Serial.print(",");
// Serial.print(accel);
// Serial.print(",");
// Serial.print(gyro);
// Serial.print(",");
// Serial.print(mg);
// Serial.print(",");
// Serial.print(system);
Serial.print(",");
Serial.print(thetaM);
Serial.print(",");
Serial.print(phiM);
Serial.print(",");
Serial.print(thetaFnew);
Serial.print(",");
Serial.print(phiFnew);
Serial.print(",");
Serial.print(thetaG);
Serial.print(",");
Serial.print(phiG);
Serial.print(",");
Serial.print(theta);
Serial.print(",");
Serial.print(phi);
Serial.print(",");
Serial.println(psi);

phiFold=phiFnew;
thetaFold=thetaFnew;

 
delay(100);
}