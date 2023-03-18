
/*
    Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
    Hayley Spencer Leavitt 2023
    Version 1.0
    Built for Sphero RVR, Arduino, with Adafruit mini spy camera module,
    Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor.


    Drive
    Header File

    The Drive object is a class written to contain all of the information
    about how we want AARVIN to move.

    Sources:
       https://www.arduino.cc/reference/en/
       https://github.com/sphero-inc/sphero-sdk-arduino-cpp
*/

// -------------------------------- Header Info ----------------------------------

#ifndef Tank_h
#define Tank_h


// --------------------------------- Libriaries ----------------------------------

#include "Arduino.h"

/*
   CLASS: Tank()
   ARGUMENTS: None

   DESCRIPTION:
   Tank is the class that contains all of the information and functions regarding AARVIN's
   chassis and driving. This includes two motors, one on the starboard side, and one on the
   port side. 
*/
class Tank
{
  public:
    // the class
    Tank();

    // the functions
    void tankprep(); 
    
    void brake();
    void brake(int duration);
     
    void forward(int fspeed); 
    void forward(int fspeed, int duration); 
    
    void reverse(int rspeed); 
    void reverse(int rspeed, int duration);
    
    void turn_right(int tspeed); 
    void turn_right(int tspeed, int duration);
    
    void turn_left(int tspeed); 
    void turn_left(int tspeed, int duration);

  private: 

};

#endif 
