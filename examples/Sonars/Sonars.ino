/*
RoverWing Sonar test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board. RoverWing board can be powered through the main power connector,
or it can get power from the Feather. The voltage readings returned by getVoltage()
measure voltage at main power connector; they are useless if RoverWing is
powered by the  Feather board.

To use this code, you need to connect one or more sonars (HC-SR04 or compatible)
to RoverWing


Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>

Rover r; //this is the name of the rover!
bool blink=false;


void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); //use fast mode (400 kHz)
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
  //Print basic info
  Serial.print("Firmware version: "); Serial.print(r.fwVersionMajor);
  Serial.print("."); Serial.println(r.fwVersionMinor);
  float v=r.getVoltage();
  Serial.print("Voltage: "); Serial.print(v);Serial.println("V");
  //now, let us activate sonars
  //this command activates all three sonars; after activation, RoverWing will
  //continuously fire each of the activated sonars in turn, recording the readings
  r.activateSonars(SONAR1_ACT+SONAR2_ACT+SONAR3_ACT); //SONARi_ACT are activation 'codes'
                                                      //(to be precise, bitmasks)

  //to change which sonars are activated,  issue another activateSonars() command
  //to deactivate all sonars, use
  //r.stopSonars();

}
void loop(){

  //get and print analog inputs

  r.getAllSonar(); //fetches values from RoverBoard and saves them in r.sonar[] array
  Serial.print("Sonar readings (mm): ");
  for (int i=0; i<3; i++){
    Serial.print("  ");
    Serial.print(r.sonar[i]); // values of sonar[i] are floats, representing
                              // distance in mm
                              // note that even though the values returned can contain
                              // decimals, e.g. 272.3 mm, in reality, accuracy of HC-SR04 sensor is
                              // about 5mm 
  }
  //instead of index i=0..2, you can alos use predefined values SONAR1, SONAR2, SONAR3
  // e.g. r.sonar[0] is the same as r.sonar[SONAR1];
  Serial.println(" ");
  delay(500);

  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
}
