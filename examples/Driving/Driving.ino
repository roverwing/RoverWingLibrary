/*
RoverWing Drive  test

This code is part of RoverWing library: https://github.com/roverwing

This code demonstrates higher level driving function such as "drive forward" or "turn 90 degrees "
It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board. RoverWing board must  be powered through the main power connector,
or it can get power from the Feather.

Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>

Rover r; //this is the name of the rover!
float yaw, pitch, roll;
motorconfig_t myMotor; //to hold configuration data for the motors
driveconfig_t drivetrain; // to hold configuration data about robot
bool blink=false;
//IMU calibration
int16_t accelOffset[]={656, 41, 300};
int16_t gyroOffset[]={-140, 145, -64};



void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  //Wire.setClock(400000); //use fast mode (400 kHz)
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
  //initilaize the IMU. It is required for course corrections
  Serial.println("Initializing and calibrating the IMU");
  r.IMUbegin();
  delay(500);
  if (r.IMUisActive() ){
    r.IMUsetOffsets(accelOffset, gyroOffset);
    Serial.println("IMU initialized");
  } else{
    //if connecting fails, wait and try again...
    Serial.print("Failed to initialize IMU");
    delay(1000);
  }

  //configure motors
  //Change the values to match your setup!!!
  myMotor.encoderCPR=1440; //if not using encoders, put 0
  myMotor.noloadRPM=250;   //motor RPM
  r.configureMotor(MOTOR1, myMotor);
  r.configureMotor(MOTOR2, myMotor);
  //now, setup the drivetrain conifuration
  drivetrain.leftMotor=MOTOR1;
  drivetrain.rightMotor=MOTOR2;
  drivetrain.leftMotorReversed=true;
  drivetrain.wheelDiameter=70;//in mm
  drivetrain.wheelBase=140;   //in mm
  drivetrain.minPower=0.05;   // need at least 5% to move
  //now save these values
  r.configureDrive(drivetrain);
  r.setDriveRampTime(1000); //set ramping up time to be 1 sec

}
void loop(){
  r.goForward(0.2, 400); //20% power, 400mm
  /*r.startForward(0.1);
  for (int i=0; i<16; i++){
    r.getDebug();
    Serial.print("Debug info: ");
    Serial.print(r.debug[0]); Serial.print(" ");
    Serial.print(r.debug[1]); Serial.print(" ");
    Serial.println(r.debug[2]);
    delay(250);
  }*/
  r.stop();
  delay(1000);
  r.turn(0.2, 90); //turn clockwise by 90 degree at 50% power
  Serial.print("Yaw: "); Serial. print(r.getYaw());  Serial.println(" ");
  delay(1000);
}
