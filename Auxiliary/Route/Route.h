
/*
    Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
    Hayley Spencer Leavitt 2022
    Version 1.0
    Built for Sphero RVR, Arduino, with Adafruit mini spy camera module,
    Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor.


    Route
    Header FFile

    The Route object is a class written to contain all of the information about
    the path that we want AARVIN to take. The Route object will contain a series
    of Waypoints that AARVIN will stop at to perform a function, such as taking
    pictures or gathering altitude data. To be used in conjunction with the
    Waypoint class.

    Sources:
       https://www.arduino.cc/reference/en/
       https://github.com/sphero-inc/sphero-sdk-arduino-cpp
       https://github.com/mikalhart/TinyGPSPlus
       https://bitbucket.org/rmerriam/rvr-cpp/src/master/
*/

// -------------------------------- Header Info ----------------------------------

#ifndef Route_h
#define Route_h


// --------------------------------- Libriaries ----------------------------------

#include <Arduino.h>
#include "Waypoint.h"

// --------------------------------- Class Info ----------------------------------

/*
   CLASS: Route()
   ARGUMENTS: None

   DESCRIPTION:
   Route contains a series of ordered Waypoint objects to form a single Route. 
   Must include our Waypoint library for the Route object to work. 
*/
class Route
{
  public:
    // the class
    Route();

    // the variables
    int       numWaypoints; 
    bool      complete;
    Waypoint  startingPoint;
    Waypoint  waypoints[];
};

#endif
