/************************************************************
* REGISTERS
*************************************************************/

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
#define REGA_GPS_LAT       108
#define REGA_GPS_LONG      112
#define REGA_GPS_TIMESTAMP 116


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
