
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

#define DRIVESPEED 100
#define DRIVETIME 2000
#define TURNSPEED 115
#define TURNTIME 1000


// ---------------------------------- Objects ------------------------------------


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
  SCMD myMotorDriver; 
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
  pinMode(8, INPUT_PULLUP); //Use to halt motor movement (ground)

  Serial.begin(9600);
  Wire.begin();
  Serial.println("Starting sketch.");

  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face is I2C_MODE 
  myMotorDriver.settings.commInterface = I2C_MODE;

  //  set address if I2C configuration selected with the config jumpers, 1000
  myMotorDriver.settings.I2CAddress = 0x5D; 

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println(myMotorDriver.begin());
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  //Uncomment code for motor 0 inversion
  //while( myMotorDriver.busy() );
  //myMotorDriver.inversionMode(0, 1); //invert motor 0

  //Uncomment code for motor 1 inversion
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1
  
  while ( myMotorDriver.busy() );
  myMotorDriver.enable(); //Enables the output driver hardware
}


/*
   CLASS: Tank
   FUNCTION: brake()
   ARGUMENTS: None
   RETURN: None

   DESCRIPTION: 
   brake() sends a signal to stop to each motor, cause the robot to stop moving
*/
void Tank::brake()
{
  myMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
  delay(5);
}
void Tank::tbrake(int duration)
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
  myMotorDriver.setDrive(LEFT_MOTOR, 0, fspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, fspeed);
  delay(5);
}
void Tank::tforward(int fspeed, int duration)
{
  // drive forward at speed for specified duration
  myMotorDriver.setDrive(LEFT_MOTOR, 0, fspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, fspeed);
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
void Tank::treverse(int rspeed, int duration)
{
  // drive forward at speed for specified duration
  myMotorDriver.setDrive(LEFT_MOTOR, 1, rspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 1, rspeed);
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
  myMotorDriver.setDrive(LEFT_MOTOR, 1, tspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, tspeed);
  delay(5);
}
void Tank::tturn_right(int tspeed, int duration)
{
  // drive forward at speed for specified duration
  myMotorDriver.setDrive(LEFT_MOTOR, 1, tspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, tspeed);
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
void Tank::turn_left(int lspeed)
{
  myMotorDriver.setDrive(LEFT_MOTOR, 0, lspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 1, lspeed);
  delay(5);
}
void Tank::tturn_left(int lspeed, int duration)
{
  // drive forward at speed for specified duration
  myMotorDriver.setDrive(LEFT_MOTOR, 0, lspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 1, lspeed);
  delay(duration);

  // brake
  brake();
}


/*
 * CLASS: Tank
 * FUNCTION: square_dance()
 * ARGUMENTS: 
 * RETURN: None
 * 
 * DESCRIPTION: 
 * Square dance makes the robot go in a little dance 
 */
 void Tank::square_dance()
 {
   for (int i = 0; i < 4; i++)
   {
     Serial.print("Dancing to the right");
     Serial.println(i); 
     forward(DRIVESPEED);
     delay(DRIVETIME);
     brake();
     delay(5);
     
     turn_right(TURNSPEED);
     delay(TURNTIME);

     brake();
     delay(5);
   }
   for (int i = 0; i < 4; i++)
   {
     Serial.print("Dancing to the left");
     Serial.println(i); 
     turn_left(TURNSPEED);
     delay(TURNTIME);

     brake();
     delay(5);
    
     reverse(DRIVESPEED);
     delay(DRIVETIME);

     brake(); 
   }
 }
