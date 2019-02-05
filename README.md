#RoverWing
Introduction
RoverWing is a shield (or "wing", following Adafruit's terminology) for Adafruit's Feather boards. This wing provides motor drivers, Inertial Motion Unit (IMU), and connection ports for servos, sonars, GPS, and other peripherals commonly used by mobile robots. It also contains a microcontroller preloaded with firmware to control these peripherals, which communicates with the Feather board using I2C protocol, thus freeing resources of the Feather board for other purposes.

The RoverWing was heavily influenced by Adafruit's CRICKIT board (in particular, it has exact same dimensions and mounting holes as the CRICKIT board). However, unlike CRICKIT, it is intended for use with more powerful 12V motors and provides a slightly different set of peripherals.

Below is the list of key features of the RoverWing:

Can be powered by 7-14V power source; contains a voltage regulator providing power to the Feather board
Only uses 2 pins (SDA and SCL) of the Feather board.
Contains on-board microcontroller, which takes care of low-level operations such as counting motor encoder pulses, using preloaded firmware
Contains on-board 6DOF Inertial Motion Unit (IMU), based on MPU6050 chip, which can be used for tracking robot orientation in space
Provides the hardware and firmware support for connecting the following external peripherals
Motors: two brushed DC motors, at up to 2.9A at 14V per motor
Quadrature encoders for each motor
Sonars: support for three HC-SR04 or compatible ultrasonic sensors (sonars)
Servos: four servos (5V) (see note on power limit below)
Six analog inputs (3.3V)
Neopixel smart LED (see note on power limit below)
GPS and magnetometer (compass) sensors
two additional I2C sensors
The RoverWing uses same connectors for power supply, motors, encoders, and I2C sensors as the REV Robotics Expansion hub used in First Tech Challenge robotics competition, so it can be easily used with the same motors and sensors.

Hardware
Below is the description of the RoverBoard hardware.

Power
The board can be powered by a 7-14V DC power supply such as 2 or 3 cell LiPO battery or a 10-cell NiMH battery. The battery port uses JST VH male connector. We recommend using 18 AWG or larger cables for power supply; see section Cables for list of compatible cables and adapters. There is also a power indicator LED next to the power connector.

Warning

The VH connectors are polarized, so they can only be plugged in one way. Because of this, there is no reverse polarity protection on the board. If you are making your own power cables, make sure to use the same polarity convention as the RoverWing, otherwise you will permanently damage the board!!

The board has a 5V high-efficiency buck voltage converter, which provides power to a plugged in Feather board via the USB bus pin of the Feather board. It also provides power to sonars, Neopixel LEDs, servos, and a 3.3V line regulator, which powers the built-in microcontroller and IMU.

Note that 5V converter is capable of producing 2.5A output. Some of it is used by on-board electronics, leaving about 2A available for Neopixels and servos.

Microcontroller
The brains of the board is the SAMD21G microcontroller - same MCU used by Arduino ZERO and Adafruit Feather M0 boards. It comes preloaded with firmware, which is described in Firmware section below. Normally there is no need to change it.

The MCU communicates with the Feather board via I2C bus.

Inertial Motion Unit
RoverWing contains a Inertial Motion Unit, based on MPU6050 chip by Invensense. This is a 6 degree of freedom sensor (3 axis gyro and 3 axis accelerometer), which can be used for determining robot orientation in space. Provided firmware contains appropriate data fusion algorithm, combining the sensor data and filtering out noise to return the robot orientation.

Note

Even with noise filtering, data obtained from this sensor alone will always suffer from accumulating error (drift); to compensate for it, you need to use an additional magnetometer (compass) sensor as described in GPS and compass section.

Motors and encoders
The RoverWing provides connections for two brushed DC motors, at the same voltage as the main power supply (7-14V). Each motor is controlled by DRV8871 motor driver by Texas Instruments, which can provide up to 2.9A per motor. The drivers are current limited, so the current will not exceed 2.9A even if the motor is stalled, which helps prevent motor burnout. The motor ports use JST VH connectors; see section Cables for list of compatible cables and adapters.

To avoid overheating, it is recommended to attach additional heatsinks to the motor drivers if you intend to run the motors at more than 2A continuous.

In addition, the RoverWing provides two ports for connecting quadrature encoders, one for each motor. The encoder ports use JST PH4 connectors, and pinouts are shown below. These are the same connectors and pinouts as used by REV Robotics hubs, so one can use the same encoder cables.

Servos
RoverWing provides four servo connections. They can be used for any servo which are controlled by standard PWM signal (500 us - 2500 us pulse duration) and 5V power.

Note

That the total current available for servos and Neopixel LEDs is about 2A. This is sufficient for micro scale servos, but might not be enough for standard size or larger servos used under heavy load. For example, for a popular HS485HB standard size servo, no-load current draw is 0.3A, but the stall draw can be as high as 1.2A.

Sonars
RoverWing provides connections for three ultrasonic distance sensors (HC-SR04 or compatible). These sonar sensors are very popular with hobby robot builders due to their low price (about $2.50/piece) and reliability. Note that these sonars use 5V power, so they can not be directly connected to 3.3V boards such as Adafruit Feather boards. RoverWing solves this problem by including voltage level shifter chip (TX1004EWR).

The sonars ports use JST PH4 connectors; see Cables for advice on choosing connector cables.

Analog inputs
RoverWing provides connectors for 6 analog sensors, together with 3.3V power and ground connectors. Note that the analog signal shoudl not exceed 3.3V, otherwise you might damage the board!

Neopixel
RoverWing provides a port for connecting Neopixel smart LEDs. This port uses JST PH3 connector; the pinout is given below.

GPS and compass
RoverWing provides connectors for external GPS and magnetometer (compass) sensors. It uses the same connectors (Hirose DF13) and pinouts as popular Pixhawk flight controller board used in quadcopters. Thus, you can use any GPS and compass combination sensor which is compatible with Pixhawk 2.4. Such sensors can be found on eBay or AliExpress for as little as $15 (here is an example).

The provided firmware takes care of reading the GPS and magnetometer sensors, providing an easy to use interface for the user. It can also combine the data from the IMU and magnetometer to provide a more reliable orientation data.

Note

TO avoid interference, it is recommended to place the magnetometer at least 15 cm (6 in) away from the motors and other electronics. A GPS+compass sensor with a stand intended for quadcopters should work well.

Additional I2C ports
Software
Add-ons
Cables
License
