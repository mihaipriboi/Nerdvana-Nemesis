#include <MPU6050_tockn.h>
#include <Wire.h>
#include "HC_SR04.h"

#define TRIG_PIN1 3
#define ECHO_PIN1 4
#define TRIG_PIN2 5
#define ECHO_PIN2 6
#define TRIG_PIN3 7
#define ECHO_PIN3 8

HC_SR04_1 sensorLeft(TRIG_PIN1, ECHO_PIN1, digitalPinToInterrupt(ECHO_PIN1));
HC_SR04_2 sensorFront(TRIG_PIN2, ECHO_PIN2, digitalPinToInterrupt(ECHO_PIN2));
HC_SR04_3 sensorRight(TRIG_PIN3, ECHO_PIN3, digitalPinToInterrupt(ECHO_PIN3));

MPU6050 mpu6050(Wire);

int leftS = 0, frontS = 0, rightS = 0;
long startTime, interval = 75;

void setup() {
  Serial.begin(115200);

  while(Serial.available() == 0);

  delay(10);
  
  Wire.begin();
  Wire.setTimeout(true);
  
  mpu6050.begin();
  Serial.println("ok!");
  mpu6050.calcGyroOffsets(false);

  //mpu6050.setGyroOffsets(-34, 2.17, 0.63);

  sensorLeft.begin();
  sensorFront.begin();
  sensorRight.begin();

  sensorLeft.start();
  sensorFront.start();
  sensorRight.start();
  startTime = millis();
}

void loop() {
  mpu6050.update();
  
  if(millis() - startTime > interval) {
    if(sensorLeft.isFinished()) {
      leftS = sensorLeft.getRange();
      sensorLeft.start();
    }
    if(sensorFront.isFinished()) {
      frontS = sensorFront.getRange();
      sensorFront.start();
    }
    if(sensorRight.isFinished()) {
      rightS = sensorRight.getRange();
      sensorRight.start();
    }
    
    Serial.print(leftS);
    Serial.print(" ");
    Serial.print(frontS);
    Serial.print(" ");
    Serial.print(rightS);
    Serial.print(" ");
    Serial.print(mpu6050.getGyroAngleX());
    Serial.println();
    startTime = millis();
  }
}