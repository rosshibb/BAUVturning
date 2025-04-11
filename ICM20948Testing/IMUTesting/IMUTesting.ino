// Basic demo for accelerometer readings from Adafruit ICM20948
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// For SPI mode, we need a CS pin
#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

// define pins for each led
#define LED_LEFT 4
#define LED_DOWN 5
#define LED_RIGHT 8
#define LED_UP 9
#define LED_CENTER 6

# define NUM_READINGS 10
int i = 0;
float readings[NUM_READINGS];

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();

  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_DOWN, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_UP, OUTPUT);
  pinMode(LED_CENTER, OUTPUT);

  digitalWrite(LED_CENTER, HIGH);
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_DOWN, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  digitalWrite(LED_UP, HIGH);
  delay(100);

  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_DOWN, LOW);
  digitalWrite(LED_RIGHT, LOW);
  digitalWrite(LED_UP, LOW);

}

void loop() {
  float startMillis = millis();
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  float angle = atan2(mag.magnetic.y, mag.magnetic.x);
  // Store the reading
  readings[i] = angle;
  i = (i + 1) % NUM_READINGS;

  // Average using sin/cos method to handle angle wraparound
  float sumSin = 0, sumCos = 0;
  for(int j = 0; j < NUM_READINGS; j++) {
    sumSin += sin(readings[j]);
    sumCos += cos(readings[j]);
  }

  float avgAngle = atan2(sumSin / NUM_READINGS, sumCos / NUM_READINGS); // radians
  float avgAngleDeg = avgAngle * RAD_TO_DEG;

  if (avgAngleDeg < 0) {
    avgAngleDeg += 360; // Normalize to 0–360°
  }

  Serial.print("Average angle: ");
  Serial.println(avgAngleDeg);

  // LED logic based on average angle
  if(avgAngleDeg >= 0 && avgAngleDeg < 45){
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_DOWN, HIGH);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, LOW);
  }else if(avgAngleDeg >= 45 && avgAngleDeg < 90){
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_DOWN, HIGH);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, LOW);
  }else if(avgAngleDeg >= 90 && avgAngleDeg < 135){
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_DOWN, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_UP, LOW);
  }else if(avgAngleDeg >= 135 && avgAngleDeg < 180){
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_DOWN, LOW);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_UP, LOW);
  }else if(avgAngleDeg >= 180 && avgAngleDeg < 225){
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_DOWN, LOW);
    digitalWrite(LED_RIGHT, HIGH);
    digitalWrite(LED_UP, HIGH);
  }else if(avgAngleDeg >= 225 && avgAngleDeg < 270){
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_DOWN, LOW);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, HIGH);
  }else if(avgAngleDeg >= 270 && avgAngleDeg < 315){
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_DOWN, LOW);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, HIGH);
  }else if(avgAngleDeg >= 315 && avgAngleDeg < 360){
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_DOWN, LOW);
    digitalWrite(LED_RIGHT, LOW);
    digitalWrite(LED_UP, LOW);
  }

  Serial.print("\t\tMag X: ");Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");Serial.print(mag.magnetic.z);Serial.println(" uT");

  delay(100);

  float endMillis = millis();
  Serial.print("Computation time: ");
  Serial.println(endMillis - startMillis);
}
