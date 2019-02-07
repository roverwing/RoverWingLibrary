#include <Wire.h>
#include "rover.h"

Rover r; //this is the name ov my rover!

float v=0; //voltage
bool blink=false;
float pwrDelta=0.25;
float servoPos[]={0.0f, 0.0f, 0.0f, 0.0f};
float power=0.0;
uint8_t hue=0; //hue for pixels, 0-255
motorconfig_t NEVEREST40;

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); //use fast mode (400 kHz)
  Serial.begin(9600); //debugging terminal
  delay(1000); //wait for 1 second, so that roverwing initializes

  while (!r.init() ){
    //if connecting fails, try again...
    delay(200);
  }
  Serial.println("Roverwing is connected");
  r.setPixelBrightness(255);
  //see specs here: https://www.andymark.com/products/neverest-classic-40-gearmotor
  NEVEREST40.encoderCPR = 1120;
  NEVEREST40.noloadRPM = 160;
  r.configureMotor(MOTOR1, NEVEREST40);
  r.configureMotor(MOTOR2, NEVEREST40);
  //start the IMU
  r.initIMU();
  delay(5000);
  if (r.IMUisActive()) Serial.println("successfully started IMU");
  else Serial.println("Failed to start IMU");
  r.setPixelRGB(2,255,0,0);
  r.showPixel();
}
void loop(){
  /*v=r.getVoltage();
  Serial.print("Voltage: "); Serial.println(v);
  Serial.print("Encoder value: "); Serial.println(r.getPosition(MOTOR1));*/
  
  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  uint32_t start=micros();
  for (int i=0; i<10; i++) {
    r.getAllAnalog();
  }
  uint32_t duration=micros()-start;
  Serial.print("Average time for reading all analogs: ");
  Serial.println(duration/10);
  Serial.println(r.analog[1]);
  
  // set pixel color
  /*for (uint8_t i= 0; i<8; i++){
    //r.setPixelHSV(i, hue+32*i, 255, 255);
    r.setPixelRGB(i, 255,0,0);
  }
  //r.setPixelRGB(5,255,0,0);
  //r.showPixel();
  //hue+=32;



  //compute new power/position for motors and servos
  power+=pwrDelta;
  if ((power>=1.0) || (power<=-1.0) ) pwrDelta *=-1.0;
  //set motors
  r.setMotorPwr(MOTOR1, power);

  //move servos
  if (millis() <10000) {
    for (uint8_t i =0; i<4; i++) {
      servoPos[i]=power;
    }
    r.setAllServo(servoPos);
  } else {
    //after 10 seconds, just disable all servos
    //and change brightness
    for (uint8_t i =0; i<4; i++) {
      servoPos[i]=0;
    }
    r.setAllServo(servoPos);
    //r.setPixelBrightness(32);
    //r.showPixel();
  }*/
  //add delay
  delay(500);
}
