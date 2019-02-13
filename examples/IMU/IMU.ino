/*
RoverWing IMU test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board. RoverWing board can be powered through the main power connector,
or it can get power from the Feather. The voltage readings returned by getVoltage()
measure voltage at main power connector; they are useless if RoverWing is
powered by the  Feather board.


Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>

Rover r; //this is the name of the rover!
float yaw, pitch, roll;
bool blink=false;

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  //Wire.setClock(400000); //use fast mode (400 kHz)
  Serial.begin(9600); //debugging terminal
  delay(1000); //wait for 1 second, so that roverwing initializes
  Serial.print("Connecting to RoverWing");
  while (!r.init() ){
    //if connecting fails, wait and try again...
    Serial.print(".");
    delay(200);
  }
  Serial.println("");
  Serial.println("Roverwing is connected");
  Serial.print("Firmware version: "); Serial.print(r.fwVersionMajor);
  Serial.print("."); Serial.println(r.fwVersionMinor);
  Serial.println("Initializing and calibrating the IMU");
  r.initIMU();
  delay(2000);
  if (r.IMUisActive() ){
    Serial.println("IMU initialized");
  } else{
    //if connecting fails, wait and try again...
    Serial.print("Problem getting IMU sttaus");
    delay(1000);
  }

}
void loop(){
  Serial.print("Yaw: "); Serial. print(r.getYaw());  Serial.print(" ");
  Serial.print("Pitch: "); Serial. print(r.getPitch());  Serial.print(" ");
  Serial.print("Roll: "); Serial. print(r.getRoll());  Serial.println(" ");
  delay(300);

  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
}
