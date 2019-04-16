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
#define MAX_POWER 1.0f

Rover r; //this is the name of the rover!
bool blink=false;
//array to set servo posiitons. Each poisition must be a float between -1.0..1.0
float servoPos[]={0.0f, 0.0f, 0.0f, 0.0f};
//motor power
float power=0; //range  -1.0 ... 1.0
//step to change motor at each loop
float pwrDelta=0.12;
motorconfig_t myMotor; //to hold configuration data for the motor - for encoder enabled motors


void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); //use fast mode (400 kHz)
  Serial.begin(9600); //debugging terminal
  delay(1000); //wait for 1 second, so that roverwing initializes
  //activates RoverWing and prints basic info to Serial
  r.beginVerbose();
  Serial.println("Starting Servos and Motors test");
  //now, initilaize the servos
  // set the range of PWM signal duration  accepted by servos, in us
  // this is optional; if you do no t set it explicitly, default value of 1000-2000
  // will be used
  r.setServoRange(SERVO1, 1900,2100); //this is  the range for HiTec servos,
  r.setServoRange(SERVO2, 1900,2100); //see https://hitecrcd.com/faqs/servos/general-servos
  r.setServoRange(SERVO3, 1900,2100);
  r.setServoRange(SERVO4, 1900,2100);
  r.setAllServo(servoPos); //sets all 4 servos to given positions at once
  //reverse one of the motors
  r.reverseMotor(MOTOR2);
  //if your motors are equipped with encoders, uncommnet the lines below and change as necessary
  myMotor.encoderCPR = 1440;  //encoder counts per revolution of output shaft
                              //"count" refers to any observable change - rise/fall on channel A or B
  r.configureMotor(MOTOR1, myMotor);
  r.configureMotor(MOTOR2, myMotor);
}
void loop(){
  //go to next value for power
  power+=pwrDelta;
  Serial.print("Setting power to "); Serial.print((int)(power*100));Serial.println("%");
  //set motor power,
  // arguments must be floats between  -1.0...1.0
  r.setAllMotorPwr(power, power);
  //now, set servo power. Again, the range should be between -1.0 and 1.0
  for (int i=0; i<4; i++){
    servoPos[i]=power;
  }
  r.setAllServo(servoPos);
  //Alternatively, you can also set power of one motor/servo  at a time:
  //r.setMotorPwr(MOTOR1, power);
  //r.setServo(SERVO1, power);

  //check if the next step would exceed the range - if so, reverse the sweep direction
  // fabs is the absolute value function for floats; regular abs doesn't work on floats,
  // see https://github.com/arduino/reference-en/issues/362
  if (fabs(power+pwrDelta)>MAX_POWER) pwrDelta =-pwrDelta;
  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  //now, wait
  delay(500);
  //uncomment if your motors are equipped with encoders
  //get current encoder values - position (in revolutions) and speed (RPM)
  r.getAllPosition();
  r.getAllSpeed();
  Serial.print("Current motor position (revolutions): "); Serial.print(r.position[MOTOR1]);
  Serial.print(" "); Serial.println(r.position[MOTOR2]);
  Serial.print("Current motor speed (RPM): "); Serial.print(r.speed[MOTOR1]);
  Serial.print(" "); Serial.println(r.speed[MOTOR2]);

}
