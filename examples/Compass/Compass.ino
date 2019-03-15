/*
RoverWing Compass  test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board.

For this test, RoverWing must be  powered through the main power connector,
using 7-14V power source, and a magnetometer (compass) sensor connceted to the compass port.

Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>

Rover r; //this is the name of the rover!
bool blink=false;
float h;
int16_t offsets[3]; //for storing offsets i.e. sensor calibration data

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); //use fast mode (400 kHz)
  Serial.begin(9600); //debugging terminal
  delay(1000); //wait for 1 second, so that roverwing initializes
  Serial.print("Connecting to RoverWing");
  while (!r.begin() ){
    //if connecting fails, wait and try again...
    Serial.print(".");
    delay(200);
  }
  Serial.println("");
  Serial.println("Roverwing is connected");
  Serial.println("Firmware version: "+ r.fwVersion());
  Serial.println("Starting compass");
  r.magBegin();
  while (r.magStatus()!=MAG_STATUS_ON){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Compass connected");
  Serial.println("Starting calibration. Please move the sensor in figure 8 shape for next 20 seconds");
  //now calibrate
  r.magCalibrate(offsets)
  Serial.println("Calibration complete. For future use, the found offsets are: ");
  for (int i=0; i<3; i++){
    Serial.print(offsets[i]); Serial.print(" ");
  }
  Serial.println("");
}
void loop(){
  h=r.getHeading();
  Serial.print("Heading: "); Serial.println(h);
  delay(500);
  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
}
