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

.. var:: uint8_t fwVersionMajor
   Firmware major version. Note that it is a class member, not a function, so do not use parentheses.

.. var:: uint8_t fwVersionMinor
   Firmware minor version.

.. function:: String fwVersion()
   Returns firmware version (major.minor) as a string, e.g. "1.27"
