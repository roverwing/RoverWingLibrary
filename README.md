# RoverWing library
An Arduino library for use with RoverWing, a robotics controller for Adafruit Feather boards. 

# Instalation
Download the zip file of this repository and then follow the [Turtorial](https://www.arduino.cc/en/guide/libraries) on how to install arduino ide libraries.
## RoverWing library Tutorial
  **Using the IMU**
  
  The Roverwing IMU allows the user to gain information about the movements of the device, the IMU can return values for yaw, pitch and roll.
  
  The Pitch, Roll, and Yaw of the device can be accessed using r.getPitch(), r.getRoll(), r.getYaw().
```
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
```
**Controlling motors and servos using the Roverwing:**

  The Roverwing allows the user to control motors and servos.
  
  To use servos you must set the pwm range for the servo's that you are using by using "*rovername*".setServoRange("*servoname*", "*pwm min value*","*pwm max value*")
  
  Servo position is set by "*rovername*".setServo("*servoname*", "*value between -1 and 1*");
  
  Motors can be used by using "*rovername*".setMotorPwr("*motorname*", "*value between -1 and 1*");
  
  
```
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
                              //"count" refers to any observable chnage - rise/fall on channel A or B
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
---
```
**Encoders with the Roverwing**

  The Roverwing allows the user to connect encoders on each motor.
  
  For each motor that you use encoders for you must set up some motor specific parameters   myMotor.encoderCPR = "*value*";
  myMotor.noloadRPM = "*value*"; These values are motor specific and can most likely be found on your motor's data sheet.
  
  In order to retrieve encoder values you must use r.getAllPosition(); and r.getAllSpeed(); the values can than be acessed by typing "*rovername*".position["*motorname*"] or with "*rovername*".speed["*motorname*"] 

```
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


Rover r; //this is the name of the rover!
bool blink=false;
int loopCount=0;
//motor power
float power=0.5; //motor power for testing
motorconfig_t myMotor; //to hold configuration data for the motor

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
  if (loopCount%40==0){
    //change power mode - every 40 cycles, or 10 sec
    power=-power;
    r.resetAllEncoder();
    Serial.print("Resetting encoders and setting new motor power: ");
    Serial.println(power);
    r.setAllMotorPwr(power,power);
  }

  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  //now, wait
  delay(250);

  //get current encoder values - position (in rotations) and speed (RPM) 
  r.getAllPosition();
  r.getAllSpeed();
  Serial.print("Current motor position (rotations): "); Serial.print(r.position[MOTOR1]);
  Serial.print(" "); Serial.println(r.position[MOTOR2]);
  Serial.print("Current motor speed (RPM): "); Serial.print(r.speed[MOTOR1]);
  Serial.print(" "); Serial.println(r.speed[MOTOR2]);
  loopCount++;
}

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


Rover r; //this is the name of the rover!
bool blink=false;
int loopCount=0;
//motor power
float power=0.5; //motor power for testing
motorconfig_t myMotor; //to hold configuration data for the motor

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
  if (loopCount%40==0){
    //change power mode - every 40 cycles, or 10 sec
    power=-power;
    r.resetAllEncoder();
    Serial.print("Resetting encoders and setting new motor power: ");
    Serial.println(power);
    r.setAllMotorPwr(power,power);
  }

  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  //now, wait
  delay(250);

  //get current encoder values - position (in rotations) and speed (RPM) 
  r.getAllPosition();
  r.getAllSpeed();
  Serial.print("Current motor position (rotations): "); Serial.print(r.position[MOTOR1]);
  Serial.print(" "); Serial.println(r.position[MOTOR2]);
  Serial.print("Current motor speed (RPM): "); Serial.print(r.speed[MOTOR1]);
  Serial.print(" "); Serial.println(r.speed[MOTOR2]);
  loopCount++;
}
```


## Additional information

<a href="https://roverwing.readthedocs.io/en/latest//">Roverwing Home Page</a>

<a href="https://roverwing.github.io/RoverWingHardware/">Hardware</a>

<a href="https://roverwing.github.io/RoverWingFirmware/">Firmware</a>

Detailed user guides are available in `extras` directory of this repository

