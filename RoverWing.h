#include "Arduino.h"
#include "Wire.h"
#define ROVERWING_ADDRESS 0x11
enum motor_t {
  MOTOR1=0, MOTOR2
};
enum servo_t {
  SERVO1=0, SERVO2, SERVO3, SERVO4
};
enum sonar_t {
  SONAR1=0, SONAR2, SONAR3
};
#define SONAR1_ACT (0x01u)
#define SONAR2_ACT (0x02u)
#define SONAR3_ACT (0x04u)
struct motorconfig_t{
  uint16_t encoderCPR;
  float noloadRPM;
  float Kp;
  float Ki;
  float Kd;
  float Ilim;
};
// Motor configuration modes
#define MOTOR_MODE_POWER 0 //includes brake
#define MOTOR_MODE_COAST 1
#define MOTOR_MODE_SPEEDPID 2
// For coasting a motor:
// Normally, motor power is an int between -500...500
// this special value indicates that the motor should be stopped in coast state
#define POWER_COAST 1000



//register A - read only
#define REGA_FW_VERSION    0
#define REGA_ANALOG_RAW    2
#define REGA_ANALOG        16
#define REGA_SONAR_RAW     30
#define REGA_SONAR         36
#define REGA_WHO_AM_I      42
#define REGA_ENCODER       44
#define REGA_SPEED         52
#define REGA_IMU_STATUS    56
#define REGA_ACCEL         60
#define REGA_GYRO          66
#define REGA_QUAT          72
#define REGA_YAW           88
#define REGA_PITCH         90
#define REGA_ROLL          92
#define REGA_MAG_STATUS    94
#define REGA_MAG           96
#define REGA_HEADING       102
#define REGA_GPS_STATUS    104
#define REGA_GPS_LAT       106
#define REGA_GPS_LONG      110
#define REGA_GPS_TIMESTAMP 114


//Register B - write only
#define REGB_ANALOG_BITMASK    0
#define REGB_SONAR_BITMASK     1
#define REGB_SONAR_TIMEOUT     2
#define REGB_SERVO             4
#define REGB_MOTOR1_PID        12
#define REGB_MOTOR2_PID        28
#define REGB_ENC_RESET         42
#define REGB_MOTOR_MODE        43
#define REGB_MOTOR_POWER       48
#define REGB_MOTOR_TARGET      52
#define REGB_IMU_CONFIG        60
#define REGB_MAG_CONFIG        61
#define REGB_GPS_CONFIG        62
#define REGB_LOW_VOLTAGE       63
#define REGB_NUM_PIXELS        64
#define REGB_PIXEL_BRIGHTNESS  65
#define REGB_PIXEL_COMMAND     66
#define REGB_PIXEL_COLORS      68

class Rover {
  public:
    /////////////////
    // Class members
    ////////////////
    String fwVersion;
    uint8_t fwVersionMajor; //major version #
    uint8_t fwVersionMinor;
    float analog[7];      //averaged readings of analog sensors, in volts
    //uint16_t analogRaw[7];//raw readings of analog sensors, 0-1023
    float sonar[3];       //averaged distances in mm
    //uint16_t sonarRaw[4]; //raw distances in mm
    float position[2];    //current motor positions (since last encoder reset), in revolutions
    float speed[2];       //current motor speed, in RPM
    float voltage;        //current voltage, in volts
    float quat[4];        //current orientation as a quaternion
    float yaw;
    float pitch;
    float roll;
    float ax;             //Acceleration, in g
    float ay;
    float az;
    float gx;             //gyro rotation speeds, in deg/s
    float gy;
    float gz;

    /////////////////
    // Public functions
    ////////////////

    //general functions
    bool init(); //returns true if init was successfull
    //analog inputs
    float getAnalog(uint8_t input);
    float getVoltage();
    void getAllAnalog();
    //sonars
    void activateSonars(uint8_t bitmask);
    void stopSonars();
    void getAllSonar();
    float getSonar(sonar_t s);

    //servos
    void setServo(servo_t s, float pos); // -1<= pos <=1
    void setAllServo(float* pos);

    //motors
    void setMotorPwr(motor_t m, float pwr);
    void setAllMotorPwr(float pwr1, float pwr2);
    void stopMotors();
    void configureMotor(motor_t m, motorconfig_t c);
    void reverseMotor(motor_t m);

    //encoders
    float getPosition(motor_t m);
    void getAllPosition();
    float getSpeed(motor_t m);
    void getAllSpeed();
    void resetEncoder(motor_t m);
    void resetAllEncoder();

    //imu
    void  initIMU();   //initializes IMU. Note: part of initialization is calibration
                      // of gyro and accelerometer sensors, which takes some time (about 1sec)
                      //During this time, robot should be absolutely still and even
                      //Avoid using other commnads that communicate with the robot during this time
    bool IMUisActive();
    void getOrientationQuat();
    void getAccel();
    void getGyro();
    float getYaw();    //return yaw, pitch, and roll angles, in degrees
    float getPitch();
    float getRoll();

    //neopixels
    void setPixelBrightness(uint8_t b);
    void setPixelRGB(uint8_t i, uint8_t R, uint8_t G, uint8_t B);
    void setPixelHSV(uint8_t i, uint8_t H, uint8_t S, uint8_t V);
    void showPixel();


  private:
    /////////////////////////////////////////////
    // members
    ////////////////////////////////////////////
    uint8_t activeSonarsBitmask;
    motorconfig_t motorsConfig[2];
    bool motorIsReversed[2];



    //////////////////////////////////////////////
    //     I2C helper functions
    /////////////////////////////////////////////
    //  BYTE LEVEL FUNCTIONS
    ////////////////////////
    // read single byte at given register address
    uint8_t readByte(uint8_t regAddress);
    // read multiple bytes and save to dest[] array
    void readByte(uint8_t regAddress, uint8_t count, uint8_t * dest);
    //write single byte
    void writeByte(uint8_t regAddress, uint8_t data);
    // write multiple bytes  from data[] array
    void writeByte(uint8_t regAddress, uint8_t count, uint8_t * data);
    /////////////////////////////////
    // OPERATIONS ON 2-BYTE (16 bit) values
    // note that register offset is still measured in 1-byte units
    uint16_t read16(uint8_t regAddress);
    void read16(uint8_t regAddress, uint8_t count, uint16_t * dest);
    void write16(uint8_t regAddress, uint16_t data);
    void write16(uint8_t regAddress, uint8_t count,  uint16_t * data);
    /////////////////////////////////
    // OPERATIONS ON 4-BYTE (32 bit) values
    uint32_t read32(uint8_t regAddress);
    void read32(uint8_t regAddress, uint8_t count, uint32_t * dest);
    void write32(uint8_t regAddress, uint32_t data);
    void write32(uint8_t regAddress, uint8_t count,  uint32_t * data);
};
