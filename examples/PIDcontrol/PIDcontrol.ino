/*
RoverWing PID speed control  test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board.

For this test, RoverWing must be  powered through the main power connector,
using 7-14V power source. You need to connect a motor (or two motors) to the motor
port of the RoverWing; the motors must be equipped with encoders which can operate
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
float speed=100; //motor speed in RPM
motorconfig_t myMotor; //to hold configuration data for the motor

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); //use fast mode (400 kHz)
  Serial.begin(9600); //debugging terminal
  delay(1000); //wait for 1 second, so that roverwing initializes
  //activates RoverWing and prints basic info to Serial
  r.beginVerbose();
  Serial.println("Starting PID  speed control  test");
  //now, configure the motors.

  //This data is for Pololu plasic gearmotors
  myMotor.encoderCPR = 1440;   // encoder counts per revolution of output shaft
                               // this should count all observable encoder events,
                               // rise/fall of channel A and rise/fall of channel B

  //If you do not know the best PID coefficicients for your motor,
  // try using the values below and adjust as needed. If you do know the values
  // already, you can comment these lines
  float noloadRPM = 250; // the motor RPM under maximal power; you can find it by running example sketch "Servos and Motors Basic"
  float maxspeed=myMotor.encoderCPR*noloadRPM/60.0; //max speed in encoder counts/s
  float Kp=0.6/maxspeed;  //suggested proportional  gain. If the motor is too slow to achieve desired speed, increase; if the motor starts oscillating, decrease.
  float Ti=0.3;           // time constant for integral gain, in seconds
  float Td=0.1;           // time constant for differential gain, in seconds
  float iLim = 1.0*Ti/Kp; // limit on integral error; this value guarantees that integral term will be at most 1.0*maxspeed
  // now, enter the PID values in motor configuration.
  // You can either use the values suggested above, or enter your own values
  myMotor.Kp=Kp;
  myMotor.Ti=Ti;         //can't be zero; to disable I term, make Ti large, e.g. 100000.0
  myMotor.Td=Td;         //to disable differential term, make Td=0
  myMotor.iLim=iLim;     // to disable integral limit, make iLim negative e.g. -1.0
  //finally, configure the motors
  r.configureMotor(MOTOR1, myMotor);
  r.configureMotor(MOTOR2, myMotor);
  r.reverseMotor(MOTOR2); //this reverses both speed control and
                          //encoder and speed readings for motor 2
}
void loop(){
  if (loopCount%40==0){
    //change direction  - every 40 cycles, or 8 sec
    r.stopMotors();
    r.resetAllEncoder();
    delay(250);
    speed=-speed;
    Serial.print("Resetting encoders and setting new motor speed: ");
    Serial.println(speed);
    r.setMotorSpeed(MOTOR1, speed);
    r.setMotorSpeed(MOTOR2, speed);
  }

  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  //now, wait
  delay(200);

  //get current encoder values - position (in revolutions) and speed (RPM)
  //r.getAllPosition();
  //Serial.print("Current motor position (in revolutions): "); Serial.println(r.position[MOTOR1]);
  //Serial.print(" "); Serial.println(r.position[MOTOR2]);
  r.getAllSpeed();
  Serial.print("Current motor speed (RPM): ");
  Serial.print(r.speed[MOTOR1]); Serial.print(" "); Serial.println(r.speed[MOTOR2]);
  loopCount++;
}
