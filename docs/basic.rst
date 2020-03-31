===============
Basic Functions
===============
This section describes basic functions provided by RoverWing library. This
documentation assumes that you have connected and initialized the RoverWing
as described in Installation and Initialization document.

All the functions and member variables described below are part of class Rover.
Thus, if you named your Rover object :code:`myRover`, then to access, say, function
:func:`begin()` you should use :func:`myRover.begin()`. Similarly, to access
class member variable :code:`fwVersionMajor`, use :code:`myRover.fwVersionMajor`.


General Functions
-----------------
.. function:: boolean begin()

   Initializes the RoverWing. Returns true if RoverWing was successfully
   initialized, false otherwise.

.. member:: uint8_t fwVersionMajor

   Firmware major version. Note that it is a class member, not a function, so
   do not use parentheses.

.. var:: uint8_t fwVersionMinor

   Firmware minor version.

.. function:: String fwVersion()

   Returns firmware version (major.minor) as a string, e.g. "1.27"

Voltage Sensing
---------------

RoverWing has a built in voltage sensing circuit which measures the supply
voltage. You can access the measurement results using the function below.
Measurements are accurate to about 2% (this is determined by the resistors
used in the circuit).

In addition, the built-in NeoPixel LED, which normally blinks green to indicate
that the firmware is loaded, can be programmed to turn yellow when the supply
voltage drops below specified threshold. To set the low voltage threshold, use
:func:`setLowVoltage` function described below.

.. function::   float getVoltage()

   Returns the RoverWing supply voltage, in volts.

.. function::  void setLowVoltage(float v)

   Sets the low voltage threshold: when the supply voltage drops below this
   value, the built-in NeoPixel LED blinks yellow.



Servos
------

RoverWing provides connection for up to four servos, using standard PWM pulse
signal. By default, RoverWing uses pulses of duration 1000us-2000us for
controlling the servos; if you are using servos with a different range, you
can change it using setServoRange command.

RoverWing provides a data type for servo names, :code:`servo_t`. It is an
enumerated type, with four different values, :code:`SERVO1, ... SERVO4`.

.. function:: void setServoRange(servo_t s, int minPulse, int maxPulse)

   Sets servo pulse operating range.

   :param int minPulse,  maxPulse:  minimal and maximal pulse duration, in
     microseconds. These values can be found in  documentation for your servo;
     typically :code:`minPulse` is between 500-1000, and
     :code:`maxPulse` is between 2000-2500. For example, for Hitec servos the range is
     1900-2100 us.

   :param s: servo name, should be one of the four values :code:`SERVO1... SERVO4`.

.. function:: void setServo(servo_t s, float pos)

   Sets servo to a given position.

   :param s: servo name, should be one of the four values :code:`SERVO1... SERVO4`.

   :param pos: should be between -1.0 and 1.0; value 0.0 corresponds to neutral (middle) position.

.. function:: void setAllServo(float* pos)

   Sets all four servos in a single operation.

   :param pos: an array of 4 floats: :code:`pos[0]`` will be used for
   :code:`SERVO1`, :code:`pos[1]` for :code:`SERVO2`, etc.


RoverWing library includes an example sketch :file:`MotorsAndServosBasic`,
which illustrates the use of these functions.

Motors - Basic Usage
--------------------

RoverWing provides connections for two brushed DC motors. In this section, we
describe basic functions for controlling the motors; more advanced operations,
using closed loop control based on encoders and IMU, is described in Advanced
Motor Control section

RoverWing provides a data type for motor names, motor_t. It allows just two
different values, MOTOR1 and . MOTOR2.

.. function::   void setMotorPwr(motor_t m, float pwr)

   Sets the power sent to a motor.

   :param m: either MOTOR1 or MOTOR2,
   :param pwr: a number  between -1.0 (full power backwards) and 1.0
               (full power forwards). Setting power to 0 stops the motor (brake).

.. function:: void setAllMotorPwr(float pwr1, float pwr2)

   Sets power of both motors in a single operation. pwr1, pwr2 should be between -1.0 and 1.0; they determine the power sent to MOTOR1 and MOTOR2 respectively.

   This function checks that the inputs are between -1.0 and 1.0; if they are not, it automatically rescales both values to make sure they are within range while keeping their ratio. For example, calling setAllMotors(2.0,1.0) has the same effect as setAllMotors(1.0,0.5).

.. function::   void stopMotors()

   Stop both motors.

.. function::   void reverseMotor(motor_t m)

   Reverses motor direction. After using this function, power sent to the motor, as well as encoder readings (see below) will be multiplied by -1.
   RoverWing library includes an example sketch MotorsAndServosBasic, which illustrates the use of these functions.

Encoders
--------

RoverWing supports quadrature encoders for each motor: if your motor is equipped
with an encoder, you can use it to get current motor position (in revolutions)
or motor speed.

Before using encoders, you need to provide some basic info about the motor
and encoder. To do that:

   1. create motor configuration data, as object of class motorconfig_t, and
      set appropriate class members

   2. apply the configuration data to the motor(s)

Below is a sample code::

   motorconfig_t myMotor;
   myMotor.encoderCPR = 1440;  //encoder counts per revolution of output shaft
   myMotor.noloadRPM = 240;    //rotation speed, in revolutions per minute (RPM) under no load
   r.configureMotor(MOTOR1, myMotor);
   r.configureMotor(MOTOR2, myMotor);


Note that "encoder counts per revolution" (CPR) should count all four types of
events generated by a quadrature encoder (rise and fall on channels A and B),
and it should be per revolution of output shaft. For example, for the Pololu
micro gear motor https://www.pololu.com/product/3051 combined with the magnetic
encoder https://www.pololu.com/product/3081, the encoder provides 12 counts per
revolution of motor shaft, and the motor contains a 75:1 gearbox, so 1
revolution of the output shaft equals 75 revolutions of the motor shaft. Thus,
the correct encoder CPR value that should be used in the configuration data is
12*75=900.

The no-load RPM is optional; it is only used for the PID speed control algorithm
as discussed in ??

After the configuration data has been applied to the motors, you can access the
motor position and speed using the functions below.

.. function::    float getPosition(motor_t m)

   Returns current motor position, in revolutions since the last encoder reset.

.. function::   void getAllPosition()

   Gets from RoverWing and saves positions of both motors. These positions can
   be accessed later via property position as described below. Using this
   function instead of getPosition(MOTOR1); getPosition(MOTOR2) ensures that
   both positions are taken at the same moment.

   float position[2]:
   Positions of motors, in revolution, fetched by getAllPosition() function. position[0] holds position of MOTOR1, and position[1] holds position of MOTOR2. Note that these values are not updated automatically: you need to call getAllPosition() to update them.
   float getSpeed(motor_t m):
   Returns current speed of motor m, in revolutions per minute (RPM).
   void getAllSpeed():
   Gets from RoverWing and saves speeds of both motors. These speeds can be accessed later via property speed as described below. Using this function instead of getSpeed(MOTOR1); getSpeed(MOTOR2); ensures that both speeds are taken at the same moment.
   float speed[2]:
   Speeds of motors, in RPM, fetched by getAllSpeed() function. speed[0] holds speed of MOTOR1, and speed[1] holds speed of MOTOR2. Note that these values are not updated automatically: you need to call getAllSpeed() to update them.
   void resetEncoder(motor_t m):
   Resets the encoder for motor m.
   void resetAllEncoder():
   Resets the encoders for both motors.
   Analog sensors
   RoverWing provides 6 analog inputs, using 10-bit analog to digital converter. You can access these values using the functions below.

   float getAnalog(uint8_t i):
   Returns reading of analog input i, in volts. Note: index i ranges from 1-6, not 0-5!
   float getAllAnalog():
   Gets from RoverWing and saves readings of all 6 analog inputs. These readings can be later accessed using property analog below. Using this function is faster than using six different getAnalog(i) calls; it also ensures that all readings are taken at the same moment, which is important if you want to compare them.
   float analog[]:
   Array of analog readings fetched by getAnalog() function. analog[1] holds the reading of analog input 1 (in volts), etc. Note that these values are not automatically updated: you must call getAllAnalog() function to update them. Also, note that you should use indexes starting with 1, not 0.
   Note

   The values returned by these functions are not raw values: RoverWing uses "low-pass" filter. Slightly simplifying, one can say that this filter, instead of returning the results of last reading, returns an average of several last readings. This helps reduce random noise but also introduces a small delay (about 1 ms) in registering changes in analog readings.
   Sonars
   RoverWing supports up to three HC-SR04 ultrasonic sensors (sonars). These inexpensive sensors are available from a variety of sources, for example Sparkfun and Amazon. The firmware operates the sonars in continuous mode, cycling all active sonars: after one sonar receives an echo, the distance is to the object is computed and saved and the next available sonar is triggered and sends a sound ping. This process repeats until stopSonars() command is received.

   At startup, no sonars are active; to activate some of the sonars, you need to call activateSonars() function.

   RoverWing provides a data type for sonar names, sonar_t. It allows three different values, SONAR1, SONAR2, SONAR3.

   As with the analog inputs, sonar readings are passed through a low-pass filter, to smooth out random noise.

   void activateSonars(uint8_t bitmask, int maxDistance=6000):
   Activates sonars. Bitmask describes which sonars should be activated (bit 0 for SONAR1, bit1 for SONAR2, etc). The easiest way is to use predefined values SONAR1, SONAR2, SONAR3 which are defined in such a way that calling activateSonars(SONAR1) activates SONAR1. Moreover, they can be added together to form any combination: for example, to activate sonars 1 and 3, use activateSonars(SONAR1+SONAR3).

   Optional parameter maxDistance specifies maximal distance to an object in mm and is used to determine the timeout time: if no echo is received in the time required for the sound to reach object at this distance and return back, then we stop waiting for echo and return distance maxDistance.

   void stopSonars():
   Stops all sonars.
   float getSonar(sonar_t s):
   Get latest distance reading of sonar s.
   void getAllSonar():
   Gets the latest readings of all active sonars from the RoverWing and saves them. These values can be later accessed using sonar[] property
   float sonar[3]:
   Array of sonar readings fetched by getAllSonar() function, in mm. sonar[0] holds reading for sonar 1, etc. Note that these values are not automatically updated: you must call getAllAnalog() function to update them.
   NeoPixel
   RoverWing allows connecting a strip of "smart" LEDs, using WS2812b or SK6812 chips. These LEDs, commonly called "NeoPixels", contain small chips which make them individually addressable: you can independently set colors of different LEDs using just one data line. You can read more about them in Adafruit's Uberguide: https://learn.adafruit.com/adafruit-neopixel-uberguide. Note that RoverWing only allows the RGB NeoPixels; RGB W NeoPixels, which add white LED to the usual RGB, are not supported.

   RoverWing allows you to connect a strip of up to 255 NeoPixels. However, the more NeoPixels you connect, the more power they consume, and the longer it takes to update the whole strip, so please check the power requirements specified in RoverWing user guide if you intend to use more than 20-30 NeoPixels.

   Note that RoverWing also contains an internal NeoPixel LED, which blinks green to indicate normal operation, or yellow to indicate low supply voltage. This NeoPixel can not be directly controlled by the user (other than setting the low voltage threshold).

   void setPixelCount(uint8_t n):
   Sets the number of NeoPixels connected to the RoverWing (up to 255)
   void setPixelBrightness(uint8_t b):
   Sets brightness for all NeoPixels (including the internal one). Brightness can range from 0-255; usually, brightness of 32 (1/8 of maximum) is bright enough
   void setPixelRGB(uint8_t i, uint8_t R, uint8_t G, uint8_t B):
   Sets color of i-th pixel, using three values for red, blue, and green colors, each ranging 0-255. Note that index i ranges from 1-255, not from 0.

   This color is not applied immediately: see description of showPixel command below.

   void setPixelColor(uint8_t i, uint32_t c):
   Sets color of i-th pixel. c should be a color in the usual hexadecimal notation: c=0xRRGGBB (see, e.g., https://www.w3schools.com/colors/colors_hexadecimal.asp). As before, i starts with 1 and the color change is not applied immediately.

   You can also use one of the named values for color: RED, GREEN, BLUE, WHITE, YELLOW, OFF.

   void setPixelHSV(uint8_t i, uint8_t H, uint8_t S, uint8_t V):
   Sets color of i-th pixel, using Hue, Saturation, and Value (see, e.g., https://www.w3schools.com/colors/colors_hsl.asp), each ranging 0-255. As before, i starts with 1 and the color change is not applied immediately.
   void showPixel():
   After setting individual pixel colors using any combination of functions above, call this function to apply all changes at once.
