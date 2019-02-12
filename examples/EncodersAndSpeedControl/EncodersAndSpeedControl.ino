/*
RoverWing encoders and speed control  test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board.

For this test, RoverWing must be  powered through the main power connector,
using 7-14V power source. You need to connect a motor (or two motors) to the motor
port of the RoverWing; the motors must be equipped with encoders whoich can operate
at 3.3V.

Finally, you need to adjust the motor confiuration data to suit  your motors


Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>

#define MAX_POWER 1.0f


Rover r; //this is the name of the rover!
bool blink=false;
//array to set servo posiitons. Each poisition must be a float between -1.0..1.0
float servoPos[]={0.0f, 0.0f, 0.0f, 0.0f};
//motor power
float power=0.0;
//step to change motor at each loop
float pwrDelta=0.2;

motorconfig_t myMotor;

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
  Serial.println("Starting encoders and speed control  test");
  //now, configure the motors.
  //This data is for NeveRest 40 motors
  myMotor.encoderCPR = 1120;
  myMotor.noloadRPM = 160;
  r.configureMotor(MOTOR1, myMotor);
  r.configureMotor(MOTOR2, myMotor);
}
void loop(){
  //get current encoder values
  r.getAllPosition();
  r.getAllSpeed();
  Serial.print("Current motor position (rotations): "); Serial.print(r.position[MOTOR1]);
  Serial.print(" "); Serial.println(r.position[MOTOR2]);
  Serial.print("Current motor speed (RPM): "); Serial.print(r.speed[MOTOR1]);
  Serial.print(" "); Serial.println(r.speed[MOTOR2]);
  // set new power
  power+=pwrDelta;
  Serial.print("Setting power to "); Serial.println(power);

  if (abs(power)>=MAX_POWER) pwrDelta =-pwrDelta;
  //set motor power
  r.setAllMotorPwr(power,power);

  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  //now, wait
  delay(500);

}
