#include "RHGenericSPI.h"
#include "RH_RF22.h"
#include "RHDatagram.h"
#include "RHGenericDriver.h"
#include "RadioHead.h"
#include "RHReliableDatagram.h"
#include "RHSPIDriver.h"
#include "RHHardwareSPI.h"
#include <Wire.h>
#include "BMA250.h"

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

RH_RF22 driver(7,3);
BMA250 accel;
int timestamp , timestampfuture;
int i = 0;

RHReliableDatagram manager(driver , CLIENT_ADDRESS);
String message;

void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  accel.begin(BMA250_range_2g, BMA250_update_time_64ms);    //This sets up the BMA250 accelerometer
  manager.init();
  timestamp = millis();
}

void printOutput(int x , int y , int z){
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);
}
void loop() {
  
  accel.read();                                          //This function gets new data from the accelerometer
  //int angle = atan2(accel.Y, accel.X) * 180.0/PI;      //math to calculate the angle from the sensor readings
  //angle = constrain(angle-90.0,-90.0,90.0);
  //We only want angles from -90 to 90
  printOutput(accel.X , accel.Y , accel.Z);
  
//  timestampfuture = millis();
//  if(timestampfuture - timestamp < 1000) i++;
//  else{
//        Serial.println(i);
//        i = 0;
//        timestamp = millis();
//        }
  
  //seg.printInt(abs(angle));
  //uint8_t data[RH_RF22_MAX_MESSAGE_LEN];

  //message = accel.X;
  //message.toCharArray((char*)data ,RH_RF22_MAX_MESSAGE_LEN);
  //manager.sendtoWait(data , sizeof(data) , SERVER_ADDRESS); 
}
