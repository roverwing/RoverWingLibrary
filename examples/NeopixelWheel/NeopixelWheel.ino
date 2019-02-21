/*
RoverWing Neopixel  test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board. RoverWing board can be powered through the main power connector,
or it can get power from the Feather. The voltage readings returned by getVoltage()
measure voltage at main power connector; they are useless if RoverWing is
powered by the  Feather board.

This example expects that you have connected a string of Neopixels (3 color only - no white)
to the neopixel port of the RoverWing; max allowed is 45 pixels.

Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>

Rover r; //this is the name of the rover!
bool blink=false;
#define NUM_PIXELS 4 //number of connected Neopixels; edit to match your configuration
#define LOW_VOLTAGE 7.0 //voltage limit; if voltage drops below this limit, internal neopixel turns yellow
uint8_t hues[NUM_PIXELS]; //array of hues of pixels


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
  Serial.print("Firmware version: "); Serial.print(r.fwVersionMajor);
  Serial.print("."); Serial.println(r.fwVersionMinor);
  Serial.print("Voltage: "); Serial.println(r.getVoltage());
  //configure internal neopixel turn yellow if voltage drops below threshold
  r.setLowVoltage(LOW_VOLTAGE);
  //now let us setup the neopixels
  r.setPixelNumber(NUM_PIXELS);
  r.setPixelBrightness(64); //1/4 of full brightness - this is already quite bright
  //setup initial hues
  for (int i=0; i<NUM_PIXELS; i++){
    hues[i]=i*255/NUM_PIXELS; // put hues uniformly on the color wheel
  }
}

void loop(){
  for (int i=0; i<NUM_PIXELS; i++) {
    //update hues
    hues[i]+=25; // this way, at every loop we are rotatiang by 25/255, or approximately 1/10 of the revolution of the color wheel
    r.setPixelHSV(i,hues[i], 255, 255);
  }
  r.showPixel(); //this must be called to push the new colors to the actual neopixels
  //blink the built-in LED of the feather board
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  delay(250);
}
