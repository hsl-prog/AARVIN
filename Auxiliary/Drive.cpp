
/*
    Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
    Hayley Spencer Leavitt 2023
    Version 1.0
    Built for Sphero RVR, Arduino, with Adafruit mini spy camera module,
    Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor.


    Drive
    Source File

    The Drive object is a class written to contain all of the information
    about how we want AARVIN to move using the Sparkfun Qwiic Motor Driver. 

    Tank requires the use of Wire and Serial. Suggested baud is 9600. 

    Pin 8 can be grounded to disable motor movement, for debugging.

    To initialize: 
      while( myMotorDriver.begin() != 0xA9) 
      {
        Serial.println(myMotorDriver.begin());
        Serial.println("ID mismatch, trying again"); 
        delay(500); 
      }
      Serial.println("ID matches 0xA9"); 

      while(myMotorDriver.read() = false)
      {
        Serial.println("Searching for secondary devices."); 
        delay(500); 
      }
      Serial.println("Ready!");
      Serial.println(); 


    Sources:
       https://www.arduino.cc/reference/en/
       https://www.sparkfun.com/products/15451
*/

// --------------------------------- Libriaries ----------------------------------

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"
#include "Drive.h"


// ------------------------------------ Macros ------------------------------------

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1


// ---------------------------------- Objects ------------------------------------

SCMD myMotorDriver; 


// ------------------------------ Class Definition -------------------------------

/*
   CLASS: Tank()
   ARGUMENTS: None

   DESCRIPTION:
   Tank is the class that contains all of the information and functions regarding AARVIN's
   chassis and driving. This includes two motors, one on the starboard side, and one on the
   port side
*/
Tank::Tank()
{
}


// ---------------------------------- Functions ----------------------------------

/*
 * CLASS: Tank
 * FUNCTION: tankprep()
 * ARGUMENTS: None
 * RETURN: None
 */
void Tank::tankprep()
{
  // configure I2C mode
  myMotorDriver.settings.commInterface = I2C_MODE; 

  // set the I2C Address 
  myMotorDriver.settings.I2CAddress = 0x5D; 

  // chip select for SPI
  myMotorDriver.settings.chipSelectPin = 10; 

  while(myMotorDriver.begin() != 0xA9) 
  {
    Serial.println("Waiting for ID match"); 
    delay(500);
  }
  
  while(myMotorDriver.ready() == false); 
  Serial.println("Ready!"); 
}


/*
   CLASS: Tank
   FUNCTION: brake()
   ARGUMENTS: None
   RETURN: None
*/
void Tank::brake()
{
  myMotorDriver.setDrive(LEFT_MOTOR, 0, 0); 
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
}
void Tank::brake(int duration)
{
  myMotorDriver.setDrive(LEFT_MOTOR, 0, 0); 
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); 

  delay(duration);
}


/*
   CLASS: Tank
   FUNCTION: forward()
   ARGUMENTS:
      int speed - how fast should the motor go?
      int duraction - how long should the motor run?
   RETURN: None

   DESCRIPTION:
   Drives the tank forward in the current direction using 0 to 255 speed, modulated by the Offset variable
*/
void Tank::forward(int fspeed)
{
  Serial.println("Driving forward"); 
  
  myMotorDriver.setDrive(LEFT_MOTOR, 0, fspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, fspeed); 
  delay(5);
}
void Tank::forward(int fspeed, int duration)
{
  // drive forward at speed for specified duration
  forward(fspeed);
  delay(duration);

  // brake
  brake();
}


/*
   CLASS: Tank
   FUNCTION: reverse()
   ARGUMENTS:
      int speed - how fast should the motor go?
      int duraction - how long should the motor run?
   RETURN: None

   DESCRIPTION:
   Drives the tank in the backwards direction using a 0 to 255 speed to keep main code cleaner
*/
void Tank::reverse(int rspeed)
{
  myMotorDriver.setDrive(LEFT_MOTOR, 1, rspeed); 
  myMotorDriver.setDrive(RIGHT_MOTOR, 1, rspeed); 
  delay(5);
}
void Tank::reverse(int rspeed, int duration)
{
  // drive forward at speed for specified duration
  reverse(rspeed);
  delay(duration);

  // brake
  brake();
}


/*
   CLASS: Tank
   FUNCTION: turn_right()
   ARGUMENTS:
      int speed - how fast should the motor go?
      int duraction - how long should the motor run?
   RETURN: None

   DESCRIPTION:
   Turns the tank in place rightwards, or clockwise, using 0 to 255 speed, modulated by the Offset variable
*/
void Tank::turn_right(int tspeed)
{
  myMotorDriver.setDrive(RIGHT_MOTOR, 1, tspeed); 
  myMotorDriver.setDrive(LEFT_MOTOR, 0, tspeed); 
  delay(5);
}
void Tank::turn_right(int tspeed, int duration)
{
  // drive forward at speed for specified duration
  turn_right(tspeed);
  delay(duration);

  // brake
  brake();
}


/*
   CLASS: Tank
   FUNCTION: turn_left()
   ARGUMENTS:
      int speed - how fast should the motor go?
      int duraction - how long should the motor run?
   RETURN: None

   DESCRIPTION:
   Turns the tank in place leftwards, or counter-clockwise, using 0 to 255 speed, modulated by the Offset variable
*/
void Tank::turn_left(int tspeed)
{
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, tspeed); 
  myMotorDriver.setDrive(LEFT_MOTOR, 1, tspeed); 
  delay(5);
}
void Tank::turn_left(int tspeed, int duration)
{
  // drive forward at speed for specified duration
  turn_right(tspeed);
  delay(duration);

  // brake
  brake();
}
