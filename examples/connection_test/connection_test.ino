#include <Wire.h>
#include <RoverWing.h>

Rover r; //this is the name of the rover!

float v=0; //voltage
bool blink=false;


void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000); //use fast mode (400 kHz)
  Serial.begin(9600); //debugging terminal
  delay(1000); //wait for 1 second, so that roverwing initializes
  Serial.println("Connecting to RoverWing");
  while (!r.init() ){
    //if connecting fails, wait and try again...
    delay(200);
  }
  Serial.println("Roverwing is connected");
}
void loop(){
  v=r.getVoltage();
  Serial.print("Voltage: "); Serial.println(v);
  
  //do the blink
  digitalWrite(LED_BUILTIN, blink);
  blink=!blink;
  //get and print analog inputs 
  for (int i=0; i<10; i++) {
    r.getAllAnalog();
  }
  Serial.print("Analog inputs: ");
  for (int i=1; i<=6; i++){
    Serial.print("  ");
    Serial.print(r.analog[i]); //note that index i runs 1..6, not 0..5 !!
  }
  Serial.println(" ");
  delay(500);
}