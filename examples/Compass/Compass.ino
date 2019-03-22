/*
RoverWing Compass  test

This code is part of RoverWing library: https://github.com/roverwing

It can be used with any Adafruit Feather board (or compatible) plugged into
RoverWing  board.

For this test, RoverWing must be  powered through the main power connector,
using 7-14V power source, and a HMC5883L magnetometer (compass) sensor
connected to the compass port.
If you are using a combined GPS/compass sensor such as are used for quadcopters,
make sure that BOTH GPS and compass are plugged in - usually power to the
sensor is provided by GPS port.




Written in 2019 by David Bershadsky, Alexander Kirillov

This example code is in the public domain.
*/
#include <Wire.h>
#include <RoverWing.h>

Rover r; //this is the name of the rover!
bool blink=false;
int16_t m[3]; //raw magnetometer data
float h;//heading
// CHANGE AS NEEDED
// if you already know calibration data, set this to true:
bool haveCalibration = false;
//and enter the correct offsets here
int16_t offsets[]= {8, 352, 1};
//if you have it, also enter the transformation matrix
float magSoftIronMatrix[3][3] = { { 0.99f, -0.008f, -0.013 },
                                    { -0.008f, 0.999f, 0.002f },
                                    { -0.013f, 0.002f, 1.0132f } };
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); //use fast mode (400 kHz)
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
  Serial.print("Voltage: "); Serial.println(r.getVoltage());

  Serial.println("Starting compass");
  r.magBegin();
  while (r.magStatus()!=MAG_OK){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Compass connected");
  if (haveCalibration) {
    //set calibration data
    r.magSetCalData(offsets, magSoftIronMatrix);
    Serial.println("Applying calibration - done");
  } else {
    //do not have calibration data yet - need to calibrate
    r.magStartCalibration();
    Serial.println("Starting calibration. Please move the rover in a figure 8. Please note that you need to move in 3d, lifting and flipping it.");
    Serial.println("The process will take about 20 seconds.");
    delay(2000);//give user time to read it
    Serial.print("Starting calibration now");
    while (r.magStatus()!=MAG_OK){
      //calibration still going
      Serial.print(".");
      delay(500);
    }
    //calibration complete!
    Serial.println("");
    r.magGetOffsets(offsets);
    Serial.print("Calibration complete. The found offstes are: ");
    for (int i=0; i<3; i++) {
      Serial.print(" "); Serial.print(offsets[i]);
    }
    Serial.println("Please save these offsets for later use");
    delay(3000); //give time to read and understand
  }
  Serial.println("Let us now test the calibrated compass...");
  delay(1000);
}
void loop(){
  r.getMagData(m);
  Serial.print("Magnetometer reading (calibrated):");
  for (int i=0; i<3; i++) {
    Serial.print(" "); Serial.print(m[i]);
  }
  h=r.getHeading();
  Serial.print("  Heading: "); Serial.println(h);
  delay(500);
  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
}
