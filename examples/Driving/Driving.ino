/*
RoverWing Drive  test

This code is part of RoverWing library: https://github.com/roverwing

This code demonstrates higher level driving function such as "drive forward" or "turn 90 degrees "
It assumes that your rover is using "tank drive", with two identical motors,
one of which powers the left side, and the other, the right side. See the user guide for details.

You need to change various pieces of configuration to match your robot;
they are indicate by CHANGE AS NEEDED in comments

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
// CHANGE AS NEEDED
//IMU calibration. You must know the IMU calibration data;
//if you do not yet have it, run the IMU example sketch
int16_t accelOffset[]={656, 41, 300};
int16_t gyroOffset[]={-140, 145, -64};



void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  //Wire.setClock(400000); //use fast mode (400 kHz)
  Serial.begin(9600); //debugging terminal
  r.IMUcalibrate(accelOffset, gyroOffset);
  delay(1000); //wait for 1 second, so that roverwing initializes
  //activates RoverWing and prints basic info to Serial
  r.beginVerbose();
  Serial.print("Initializing and calibrating the IMU ");
  r.IMUbegin();
  delay(500);
  while (! r.IMUisActive() ) {
    Serial.print(".");
    delay(200);
  }

  r.IMUsetOffsets(accelOffset, gyroOffset);
  Serial.println(" ");
  Serial.println("IMU initialized");

  // CHANGE AS NEEDED
  //configure motors
  //Change the values to match your setup!!!
  myMotor.encoderCPR=1440; //if not using encoders, put 0
  myMotor.noloadRPM=250;   //motor RPM
  r.configureMotor(MOTOR1, myMotor);
  r.configureMotor(MOTOR2, myMotor);
  //now, setup the drivetrain conifuration. Read the user guide for detailed explanations 
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
  r.goForward(0.4, 400); //40% power, 400mm
  /* Alternatively, you could use:
  r.startForward(0.4);
  delay(2000);
  r.stop(); */
  delay(1000);
  r.turn(0.2, 90); //turn clockwise by 90 degree at 50% power
  Serial.print("Yaw: "); Serial. print(r.getYaw());  Serial.println(" ");
  delay(1000);
}
