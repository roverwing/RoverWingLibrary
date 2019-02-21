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
int currentPixel=0;
#define NUM_PIXELS 4 //number of connected Neopixels; edit to match your configuration
#define LOW_VOLTAGE 7.0 //voltage limit; if voltage drops below this limit, internal neopixel turns yellow


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
}

void loop(){
  r.setPixelRGB(currentPixel, 0,0,0); //turn off
  //move to next pixel
  currentPixel++;
  if (currentPixel==NUM_PIXELS) currentPixel=0;
  //and set it blue
  r.setPixelRGB(currentPixel, 0,0,255);
  //Alternatively, you could use :
  // r.setPixelColor(currentPixel, 0x0000FF);
  // or
  // r.setPixelColor(currentPixel, BLUE);
  // you can use the following named colors:
  // RED, GREEN, BLUE, YELLOW, WHITE, OFF
  r.showPixel(); //this must be called to push the new colors to the actual neopixels
  //blink the built-in LED of the feather board 
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  delay(500);
}
