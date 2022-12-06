
/*
    Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
    Hayley Spencer Leavitt 2022
    Version 1.0
    Built for Sphero RVR, Arduino, with Adafruit mini spy camera module,
    Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor.


    Waypoint
    Source File

    The Waypoint object is a class written to contain all of the information
    about a waypoint that we want AARVIN to visit along its route. Each waypoint
    will consist of a latitude and longitude.
    To be used in conjuction with the Route class.

    Sources:
       https://www.arduino.cc/reference/en/
       https://github.com/sphero-inc/sphero-sdk-arduino-cpp
       https://github.com/mikalhart/TinyGPSPlus
       https://bitbucket.org/rmerriam/rvr-cpp/src/master/
*/

// --------------------------------- Libriaries ----------------------------------

#include "Waypoint.h"     // header file for this library


// ------------------------------ Class Definition -------------------------------

/*
   CLASS: Waypoint()
   ARGUMENTS: None

   DESCRIPTION:
   Waypoint contains all the information about a single waypoint, including the
   latitude and longitude of the point, and if it has been visited.
*/
Waypoint::Waypoint()
{
  // variables
  float  wlat;                // latitude
  float  wlong;               // longitude
  float  waltitude;           // altitude
  bool   isVisited = false;   // boolean for if the point has been visited or not
}
