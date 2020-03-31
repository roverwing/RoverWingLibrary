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
