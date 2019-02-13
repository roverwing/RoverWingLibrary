/*
RoverWing basic servos and motors test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board.

For this test, RoverWing must be  powered through the main power connector,
using 7-14V power source. It is assumed that you have connected some servos
to the servo ports, and motors rated for the voltage provided by the power source.

It is possible to use a motor with  voltage  rating  slightly lower than the power
source by adjusting  MAX_POWER constant below. E.g., for 6V motor with 7.2V
power supply, set MAX_POWER  to 0.8f; then 7.2*0.8=5.76, which is less than 6V.



Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>
#define MAX_POWER 100

Rover r; //this is the name of the rover!
bool blink=false;
//array to set servo posiitons. Each poisition must be a float between -1.0..1.0
float servoPos[]={0.0f, 0.0f, 0.0f, 0.0f};
//motor power
int power=0; //range  -100..100
//step to change motor at each loop
int pwrDelta=20;


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
  Serial.println("Starting Servos and Motors test");
  //now, initilaize the servos
  r.setAllServo(servoPos); //sets all 4 servos to given positions at once
  //reverse one of the motors
  r.reverseMotor(MOTOR2);
}
void loop(){

  power+=pwrDelta;
  Serial.print("Setting power to "); Serial.print(power);Serial.println("%");
  //check if we reached the limits
  if (abs(power)>=MAX_POWER) pwrDelta =-pwrDelta;
  //set motor power,
  // arguments must be floats between  -1...1
  r.setAllMotorPwr(0.01f*power,0.01f*power);
  //now, set servo power
  for (int i=0; i<4; i++){
    servoPos[i]=0.01f*power;
  }
  r.setAllServo(servoPos);
  //Alternatively, you can also set power of one motor/servo  at a time:
  //r.setMotorPwr(MOTOR1, power);
  //r.setServo(SERVO1, power);


  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  //now, wait
  delay(500);

}
