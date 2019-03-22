#include "RoverWing.h"
const float voltageScale=(3.3f/1023.0f)*(122.0f/22.0f); //voltage divider uses 100k and 22k resistors
const float analogScale=(3.3f/1023.0f);
const float aScale=1/16384.0f; //acceleration scale: this is the value of LSB of accel data, in g
const float gScale=250.0f / 32768.0f; //gyro resolution, in (deg/s)/LSB
const float  microsToMm=0.171f; //half of speed of sound, which is 343 m/s = 0.343 mm/us

/* *******************************************
 *  PUBLIC FUNCTIONS
 *********************************************
 */

 //general functions
bool Rover::begin(){
   int i;
   //try getting response from the roverwing
   if (readByte(REGA_WHO_AM_I) == ROVERWING_ADDRESS) {
     //get firmware version
      uint8_t fw[2];
      readByte(REGA_FW_VERSION,2,fw);
      fwVersionMinor=fw[0];
      fwVersionMajor=fw[1];
      //set up motors
      for (i=0; i<2; i++){
        motorIsReversed[i] = false;
      }
      //set up servos
      for (i=0; i<4; i++){
        //set servo range 1000...2000 us
        servoCenterPos[i]=1500;
        servoHalfRange[i]=500;
      }
      //if top cover is present, configure the top display
      return true;
   } else {
     //failed to get correct response
     //Serial.print("Received WHOAMI value: "); Serial.println((uint8_t)readByte(REGA_WHO_AM_I));
     return false;
   }

 }
String Rover::fwVersion(){
  return(String(fwVersionMajor)+"."+String(fwVersionMinor));
}
 //analog inputs
 float Rover::getAnalog(uint8_t input){
   float result=(float)read16(REGA_ANALOG+2*input)*0.1f*analogScale;
   analog[input]=result;
   return result;
 }
 float Rover::getVoltage(){
   uint16_t rawVoltage;
   rawVoltage = read16(REGA_ANALOG); //voltage = analog0;
   voltage=(float)rawVoltage*0.1f*voltageScale;
   return voltage;
 }
 void Rover::getAllAnalog(){
   uint16_t raw[7];
   read16(REGA_ANALOG,7,raw);
   for (uint8_t i =0; i<7; i++){
     analog[i]=(float)raw[i]*0.1f*analogScale;
   }
 }
 //sonars
 void Rover::activateSonars(uint8_t bitmask, int maxDistance){
   activeSonarsBitmask = bitmask;
   //Serial.println(bitmask, BIN);
   write16(REGB_SONAR_TIMEOUT, maxDistance/microsToMm);
   writeByte(REGB_SONAR_BITMASK, bitmask);
 }
 void Rover::stopSonars(){
   activeSonarsBitmask=0x00;
   writeByte(REGB_SONAR_BITMASK, 0x00);
 }
 void Rover::getAllSonar(){
   uint16_t raw[3];
   read16(REGA_SONAR, 3, raw);
   for (uint8_t i=0; i<3; i++){
     sonar[i]=(float)raw[i]*0.1f;
   }
 }
 float Rover::getSonar(sonar_t s){
   float result=(float)read16(REGA_SONAR+2*s)*0.1f;
   sonar[s]=result;
   return result;
 }

 //servos
 void Rover::setServo(servo_t s, float pos){
   uint16_t pulseWidth;
   pulseWidth = servoCenterPos[s] + (uint16_t)(pos*servoHalfRange[s]);
   write16(REGB_SERVO+s, pulseWidth);
 }
 void Rover::setAllServo(float* pos){
   uint16_t pulseWidth[4];
   for (uint8_t i =0; i<4; i++) {
     pulseWidth[i]=1500 + (uint16_t)(pos[i]*1000.0f);
   }
   write16(REGB_SERVO, 4, pulseWidth);
 }
 void Rover::setServoRange(servo_t s, int minPulse, int maxPulse){
   servoCenterPos[s]=(minPulse+maxPulse)/2;
   servoHalfRange[s]=(maxPulse-minPulse)/2;
 }


//motors
void Rover::setMotorPwr(motor_t m, float pwr){
  int16_t power= (int16_t)(pwr*500.0f);
  //set mode
  writeByte(REGB_MOTOR_MODE+m,(uint8_t)MOTOR_MODE_POWER);
  //
  if (power>500) power=500;
  else if (power<-500) power=-500;
  if (motorIsReversed[m]) power=-power;
  write16(REGB_MOTOR_POWER +2*m, (uint16_t)power );
}
void Rover::setAllMotorPwr(float pwr1, float pwr2){
  float m = max (fabs(pwr1), fabs(pwr2));
  //set mode
  write16(REGB_MOTOR_MODE, (uint16_t)(MOTOR_MODE_POWER<<8|MOTOR_MODE_POWER));
  //rescale power
  if (m>1.0f) {
   pwr1/=m;
   pwr2/=m;
  }
  if (motorIsReversed[MOTOR1]) pwr1=-pwr1;
  if (motorIsReversed[MOTOR2]) pwr2=-pwr2;

  int16_t power[2]= {(int16_t)(pwr1*500.0f), (int16_t)(pwr2*500.0f)} ;
  write16(REGB_MOTOR_POWER,2,(uint16_t *)power );
}
void Rover::stopMotors(){
  //FIXME: add "float" mode
  //set mode
  write16(REGB_MOTOR_MODE, (uint16_t)(MOTOR_MODE_POWER<<8|MOTOR_MODE_POWER));
  //set power
  int16_t power[]={0,0};
  write16(REGB_MOTOR_POWER,2,(uint16_t *)power );
}
void Rover::configureMotor(motor_t m, motorconfig_t c){
  motorsConfig[m].encoderCPR=c.encoderCPR;
  if (c.Kp>0){
    motorsConfig[m].Kp=c.Kp;
    motorsConfig[m].Ti=c.Ti;
    motorsConfig[m].Td=c.Td;
    motorsConfig[m].iLim=c.iLim;
  } else {
    motorsConfig[m].Ti=1.0;
    /*//let us try some defaults
    float maxspeed=c.encoderCPR*c.noloadRPM/60.0f; //maximal speed, encoder ticks/s
    float Kp=1.4/maxspeed; //thus, error of 0.5 maxspeed makes proportional term be 25% of maximal power
    Serial.print("Maxspeed: "); Serial.println(maxspeed);
    Serial.print("Kp: "); Serial.println(Kp,4);
    motorsConfig[m].Kp=Kp;
    motorsConfig[m].Ti=1.0; //set Ti=0.5 s
    motorsConfig[m].Td=0.1; //set Td=0.1 s
    motorsConfig[m].Ilim=500; //FIXME*/
  }
  //

  float PIDcoef[4]={motorsConfig[m].Kp,
                    motorsConfig[m].Kp/motorsConfig[m].Ti, //Ki
                    motorsConfig[m].Kp*motorsConfig[m].Td, //Kd
                    motorsConfig[m].iLim};
  if (m==MOTOR1) {
    write32(REGB_MOTOR1_PID, 4, (uint32_t *)PIDcoef);
    //Serial.print("Setting PID coef1: Ilim=");
    //Serial.println(PIDcoef[3],5);
  } else {
    write32(REGB_MOTOR2_PID, 4, (uint32_t *)PIDcoef);
    //Serial.print("Setting PID coef2: Ilim=");
    //Serial.println(PIDcoef[3],5);
  }
}
void Rover::reverseMotor(motor_t m){
  motorIsReversed[m]=true;
}
void Rover::setMotorSpeed(motor_t m, float s){ //speed in rpm
  //
  int32_t speed = (int32_t) (s*motorsConfig[m].encoderCPR/60.0); //convert rpms to encoder ticks/s
  if (motorIsReversed[m]) speed=-speed;
  write32(REGB_MOTOR_TARGET + 4*m, speed );
  //set mode
  writeByte(REGB_MOTOR_MODE+m,(uint8_t)MOTOR_MODE_SPEEDPID);
}



//encoders
float Rover::getPosition(motor_t m){
  int32_t encoderCount=(int32_t)read32(REGA_ENCODER+4*m);
  position[m]=(float)encoderCount/(float)motorsConfig[m].encoderCPR;
  if (motorIsReversed[m]) position[m]=-position[m];
  return position[m];
}
void Rover::getAllPosition(){
  int32_t encoderCount[2];
  read32(REGA_ENCODER,2,(uint32_t*)encoderCount); //read array of two numbers in one operation
  for (uint8_t m=0;m<2; m++){
    position[m]=(float)encoderCount[m]/(float)motorsConfig[m].encoderCPR;
    if (motorIsReversed[m]) position[m]=-position[m];
  }
}

float Rover::getSpeed(motor_t m){
  int16_t encoderSpeed=(int16_t)read16(REGA_SPEED+2*m); //speed in encoder counts/s
  speed[m]=(float)encoderSpeed*60.0f/(float)motorsConfig[m].encoderCPR; //convert to RPM
  if (motorIsReversed[m]) speed[m]=-speed[m];
  return speed[m];
}
void Rover::getAllSpeed(){
  int16_t encoderSpeed[2];
  read16(REGA_SPEED,2,(uint16_t*)encoderSpeed); //read array of two numbers in one operation
  for (uint8_t m=0;m<2; m++){
    speed[m]=(float)encoderSpeed[m]*60.0f/(float)motorsConfig[m].encoderCPR;
    if (motorIsReversed[m]) speed[m]=-speed[m];
  }

}
void Rover::resetEncoder(motor_t m){
  byte reset=0x01<<(uint8_t)m;
  writeByte(REGB_ENC_RESET, reset);
}
void Rover::resetAllEncoder(){
  writeByte(REGB_ENC_RESET,0x03);
}

//imu
void  Rover::IMUbegin(){
  writeByte(REGB_IMU_CONFIG, IMU_CONFIG_BEGIN);
}
void  Rover::IMUend(){
  writeByte(REGB_IMU_CONFIG, 0x00);
}
void Rover::IMUcalibrate(int16_t * aOffsets, int16_t * gOffsets){
  uint8_t i;
  writeByte(REGB_IMU_CONFIG, IMU_CONFIG_CALIBRATE);
  delay(500);
  //wait until completed
  while (readByte(REGA_IMU_STATUS) != IMU_OK) delay(100);
  //get calculated offsets
  read16(REGA_ACCEL_OFFSET,3, (uint16_t *) aOffsets);
  read16(REGA_GYRO_OFFSET,3, (uint16_t *) gOffsets);
}
void Rover::IMUsetOffsets(int16_t * aOffsets, int16_t * gOffsets){
  write16(REGB_ACCEL_OFFSET, 3, (uint16_t *)aOffsets);
  write16(REGB_GYRO_OFFSET, 3, (uint16_t *)gOffsets);
}
bool Rover::IMUisActive(){
  byte b=readByte(REGA_IMU_STATUS);
  Serial.println(b);
  return (b==IMU_OK);
}
void Rover::getOrientationQuat(){
  read32(REGA_QUAT,4,(uint32_t*)quat);
}
void Rover::getAccel(){
  int16_t accel[3];
  read16(REGA_ACCEL,3, (uint16_t*)accel);
  ax=(float)accel[0]*aScale;
  ay=(float)accel[0]*aScale;
  az=(float)accel[0]*aScale;
}
void Rover::getGyro(){
  int16_t gyro[3];
  read16(REGA_GYRO,3, (uint16_t*)gyro);
  gx=(float)gyro[0]*gScale;
  gy=(float)gyro[1]*gScale;
  gz=(float)gyro[2]*gScale;
}
float Rover::getYaw(){
  int16_t yawRaw=(int16_t)read16(REGA_YAW);
  yaw=(float)yawRaw/10.0f;
  return yaw;
}
float Rover::getPitch(){
  int16_t pitchRaw=(int16_t)read16(REGA_PITCH);
  pitch=(float)pitchRaw/10.0f;
  return pitch;
}
float Rover::getRoll(){
  int16_t rollRaw=(int16_t)read16(REGA_ROLL);
  roll=(float)rollRaw/10.0f;
  return roll;
}
//GPS
void Rover::GPSbegin(){
  writeByte(REGB_GPS_CONFIG, 0x01);
}
void Rover::GPSend(){
  writeByte(REGB_GPS_CONFIG, 0x00);
}
uint8_t Rover::GPSstatus(){
  return readByte(REGA_GPS_STATUS);
}
void Rover::getGPSlocation(){
  int32_t loc[2];
  uint32_t timestamp;
  read32(REGA_GPS_LAT,2,(uint32_t*)loc);
  location.latitude=loc[0];
  location.longitude=loc[1];
  location.timestamp=read32(REGA_GPS_TIMESTAMP);
}
void Rover::saveGPSlocation(location_t & l){
  l.latitude=location.latitude;
  l.longitude=location.longitude;
  l.timestamp=location.timestamp;
}
float Rover::distanceTo(const location_t & l ){
  // Equirectangular calculation from http://www.movable-type.co.uk/scripts/latlong.html
  double y = (l.latitude - location.latitude) * RAD_PER_DEG * LOC_SCALE;
  double dLon = (l.longitude- location.longitude) * RAD_PER_DEG * LOC_SCALE;
  double x    = dLon * cosf( location.latitude * RAD_PER_DEG * LOC_SCALE + y/2 );
  float angle=sqrt( x*x + y*y );//angle in radians
  return (angle * EARTH_RADIUS_KM * 1000.0f);
}
float Rover::bearingTo(const location_t & l ){
  double y = (l.latitude - location.latitude) * RAD_PER_DEG * LOC_SCALE;
  double dLon = (l.longitude- location.longitude) * RAD_PER_DEG * LOC_SCALE;
  double x    = dLon * cosf( location.latitude * RAD_PER_DEG * LOC_SCALE + y/2 );
  return (90.0-DEG_PER_RAD*atan2(y,x));
}

//magnetometer
void Rover::magBegin(){
  int i,j;
  writeByte(REGB_MAG_CONFIG, MAG_CONFIG_BEGIN);
}
void Rover::magEnd(){
  writeByte(REGB_MAG_CONFIG, MAG_CONFIG_END);
}
uint8_t Rover::magStatus(){
  return readByte(REGA_MAG_STATUS);
}
void Rover::magSetCalData (int16_t  offsets[3], float matrix[3][3]) {
  int16_t buf[9]; //
  write16(REGB_MAG_OFFSET,3,(uint16_t *)offsets);
  //now the matrix
  for (int i=0;i<3;i++){
    for (int j=0;j<3; j++){
      //convert float to integers
      buf[3*i+j]=(int16_t)(matrix[i][j]*1000.0f);
    }
  }
  write16(REGB_MAG_MATRIX,9, (uint16_t *)buf);
}
void Rover::magStartCalibration(){
  writeByte(REGB_MAG_CONFIG, MAG_CONFIG_CALIBRATE);
}
void Rover::magGetOffsets(int16_t offsets[3]){
  read16(REGA_MAG_OFFSET, 3, (uint16_t *)offsets);
}
void Rover::setDeclination(float d){
  declination=d;
}
void Rover::getMagData(int16_t m[3]){
  read16(REGA_MAG, 3, (uint16_t *)m);
}
float Rover::getHeading(){
  int16_t mag[2];
  read16(REGA_MAG,2,(uint16_t *)mag);
  //Serial.print(mag[0]); Serial.print(" "); Serial.println(mag[1]);
  float h=atan2(-mag[0],-mag[1])*DEG_PER_RAD; //heading relative to magnetic North: 0=N, 90=E
  h+=declination;
  if (h>360.0f) h-=360.0;
  else if (h<0.0f) h+=360.0;
  return h;
}
//neopixels
void Rover::setLowVoltage(float v){
  writeByte(REGB_LOW_VOLTAGE, (uint8_t)(v*10.0f));
}
void Rover::setPixelCount(uint8_t n){
  writeByte(REGB_NUM_PIXELS,n);
}
void Rover::setPixelBrightness(uint8_t b){
  writeByte(REGB_PIXEL_BRIGHTNESS, b);
}
void Rover::setPixelColor(uint8_t i, uint32_t c){
  uint8_t color[]={c, (c>>8), (c>>16), i}; //B, G, R components
  writeByte(REGB_PIXEL_COLOR, 4, color);
}
void Rover::setPixelRGB(uint8_t i, uint8_t R, uint8_t G, uint8_t B){
  uint8_t color[]={B, G, R,i};
  writeByte(REGB_PIXEL_COLOR, 4, color);
}
void Rover::setPixelHSV(uint8_t i, uint8_t H, uint8_t S, uint8_t V){
  //algorithm is borrowed from
  //https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
  uint8_t region, remainder, p, q, t, R, G, B;

  if (S == 0) {
    R=V; G=V; B=V;
  } else {
    region = H / 43;
    remainder = (H - (region * 43)) * 6;

    p = (V * (255 - S)) >> 8;
    q = (V * (255 - ((S * remainder) >> 8))) >> 8;
    t = (V * (255 - ((S * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
      case 0:
      R = V; G = t; B = p;
      break;
      case 1:
      R = q; G = V; B = p;
      break;
      case 2:
      R = p; G = V; B = t;
      break;
      case 3:
      R = p; G = q; B = V;
      break;
      case 4:
      R = t; G = p; B = V;
      break;
      default:
      R = V; G = p; B = q;
      break;
    }

  } //of (if S==0)
  //now that we have R, G, B values, let us use them
  uint8_t color[]={B, G, R,i};
  writeByte(REGB_PIXEL_COLOR, 4, color);
}
void Rover::showPixel(){
  writeByte(REGB_PIXEL_COMMAND, 0x01);
}
//drive
void Rover::configureDrive(driveconfig_t d){
  drive=d;
  //get motor directions//turns
  byte mConfig=0x00;
  bool motor1dir,motor1turnDir, motor2dir, motor2turnDir;
  if (d.leftMotor==MOTOR1){
    //we assume that in this case, rightMotor is MOTOR2
    motor1dir=d.leftMotorReversed;
    motor2dir=d.rightMotorReversed;
    motor1turnDir=motor1dir;
    motor2turnDir=!motor2dir;
  } else {
    motor1dir=d.rightMotorReversed;
    motor2dir=d.leftMotorReversed;
    motor1turnDir=!motor1dir;
    motor2turnDir=motor2dir;
  }
  mConfig=motor1dir|(motor1turnDir<<1)|(motor2dir<<2)|(motor2turnDir<<3);
  //send it to roverwing
  writeByte(REGB_DRIVE_MOTORCONFIG, (uint8_t)mConfig);
  //now, let us compute max speed in encoder ticks/s, using configuration for motor1
  uint16_t maxSpeed=( (uint32_t)motorsConfig[0].encoderCPR*motorsConfig[0].noloadRPM)/60;
  //compute the turning speed
  float maxSpeedMm=(float)motorsConfig[0].noloadRPM*PI*d.wheelDiameter/60.0; //maximal speed in mm/s
  float maxTurnSpeed = 360.0*maxSpeedMm/(d.wheelBase*PI);
  write16(REGB_DRIVE_MAXSPEED, maxSpeed);
  write16(REGB_DRIVE_MAXTURNSPEED, (uint16_t)maxTurnSpeed);
  //now, write or compute PID coef
  //FIXME
}
void Rover::startForward(float power){
  int16_t p=power*MOTOR_MAX_POWER;
  p=abs(p);//make sure it is positive!
  write16(REGB_DRIVE_TARGETPOWER, (uint16_t)p);
  writeByte(REGB_DRIVE_MODE, DRIVE_STRAIGHT);
}
void Rover::startBackward(float power){
  int16_t p=power*MOTOR_MAX_POWER;
  p=-abs(p);//make sure it is negative
  write16(REGB_DRIVE_TARGETPOWER, (uint16_t)p);
  writeByte(REGB_DRIVE_MODE, DRIVE_STRAIGHT);
}
void Rover::stop(){
  writeByte(REGB_DRIVE_MODE, DRIVE_OFF);
}
int32_t Rover::distanceTravelled(){
  //FIXME
}
void Rover::turn(float power, float degrees){
  int16_t p=power*MOTOR_MAX_POWER;
  p=abs(p);//make sure it is positive!
  write16(REGB_DRIVE_TARGETPOWER, (uint16_t)p);
  int16_t d=degrees*10.0f;
  write16(REGB_DRIVE_TURNANGLE, (uint16_t)d);
  writeByte(REGB_DRIVE_MODE, DRIVE_TURN);
  while(driveInProgress()) delay(10);
}

bool Rover::driveInProgress(){
  return (readByte(REGA_DRIVE_STATUS)==DRIVE_STATUS_INPROGRESS);
}





/* *******************************************
 *  PRIVATE FUNCTIONS
 *********************************************
 */
 // byte level operations
uint8_t Rover::readByte(uint8_t regAddress){
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(ROVERWING_ADDRESS);         // Initialize the Tx buffer
  Wire.write(regAddress);                   // Put slave register address in Tx buffer
  Wire.endTransmission();             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom((uint8_t)ROVERWING_ADDRESS, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void Rover::readByte(uint8_t regAddress, uint8_t count, uint8_t * dest){
  Wire.beginTransmission(ROVERWING_ADDRESS);   // Initialize the Tx buffer
  Wire.write(regAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission();       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  if (Wire.requestFrom((uint8_t)ROVERWING_ADDRESS, count)){
    // Read bytes from slave register address
    while (Wire.available()) {
      dest[i++] = Wire.read();
    }         // Put read results in the Rx buffer
  } else {
    Serial.print("Failed to get "); Serial.print(count); Serial.print(" bytes from wing at register "); Serial.println(regAddress);
  }
}

void Rover::writeByte(uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(ROVERWING_ADDRESS);  // Initialize the Tx buffer
  Wire.write(regAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void Rover::writeByte(uint8_t regAddress, uint8_t count, uint8_t * data){
  uint8_t i;
  Wire.beginTransmission(ROVERWING_ADDRESS);  // Initialize the Tx buffer
  Wire.write(regAddress);           // Put slave register address in Tx buffer
  for (i=0;i<count; i++){
    Wire.write(data[i]);                 // Put data in Tx buffer
  }
  Wire.endTransmission();           // Send the Tx buffer
}
// operations on 16-bit values
uint16_t Rover::read16(uint8_t regAddress){
  uint8_t data[2];
  uint16_t v;
  readByte(regAddress,2, data);
  v= ( (data[1]<<8)| data[0]); // all multibyte registers use little endian convention, LSB first
  return v;
}
void Rover::read16(uint8_t regAddress, uint8_t count, uint16_t * dest){
  readByte(regAddress, count*2, (uint8_t *) dest);
}

void Rover::write16(uint8_t regAddress, uint16_t data){
  writeByte(regAddress, 2, (uint8_t *) &data);
}
//writes an array of 16-bit values, starting at offset regAddress
void Rover::write16(uint8_t regAddress, uint8_t count, uint16_t * data){
  writeByte(regAddress, count*2, (uint8_t *) data);
}
// operations on 32-bit values
uint32_t Rover::read32(uint8_t regAddress){
  uint8_t data[4];
  uint32_t v;
  readByte(regAddress,4, data);
  v= ( (data[3]<<24)|(data[2]<<16)|(data[1]<<8)| data[0]); // all multibyte registers use little endian convention, LSB first
  return v;
}
void Rover::read32(uint8_t regAddress, uint8_t count, uint32_t * dest){
  readByte(regAddress, count*4, (uint8_t *) dest);
}
void Rover::write32(uint8_t regAddress, uint32_t data){
  writeByte(regAddress, 4, (uint8_t *) &data);
}
//writes an array of32-bit values, starting at offset regAddress
void Rover::write32(uint8_t regAddress, uint8_t count, uint32_t * data){
  writeByte(regAddress, count*4, (uint8_t *) data);
}
