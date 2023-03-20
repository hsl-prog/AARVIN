/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2023

   Square Dance
   Version 1.0

   Sources:
       https://www.arduino.cc/reference/en/
       https://www.sparkfun.com/products/15451 
*/

// ----------------------------------- Libraries ----------------------------------
#include <Arduino.h>
#include "Wire.h"
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values


// ------------------------------------ Macros ------------------------------------
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define DRIVESPEED 100
#define DRIVETIME 2000
#define TURNSPEED 115
#define TURNTIME 1000


// ------------------------------------ Objects -----------------------------------
SCMD myMotorDriver; 


// ----------------------------------- Functions ----------------------------------
//  Pin 8 can be grounded to disable motor movement, for debugging.

void setup()
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
 * FUNCTION: brake()
 * ARGUMENTS: None
 * RETURNS: None 
 * 
 * DESCRIPTION: 
 * Brake sends a stop signal to both motors, causing the robot to stop
 */
void brake()
{
  myMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
  delay(5);
}

/*
 * FUNCTION: forward()
 * ARGUMENTS: None
 * RETURNS: None 
 * 
 * DESCRIPTION: 
 * Forward sends a forward signal to one motor and a reverse signal to the other 
 * motor, causing the robot to stop
 */
void forward(int fspeed)
{
  myMotorDriver.setDrive(LEFT_MOTOR, 0, fspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, fspeed);
  delay(5);
}

void reverse(int rspeed)
{
  myMotorDriver.setDrive(LEFT_MOTOR, 1, rspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 1, rspeed);
  delay(5);
}

void turn_right(int tspeed)
{
  myMotorDriver.setDrive(LEFT_MOTOR, 1, tspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 0, tspeed);
  delay(5);
}

void turn_left(int lspeed)
{
  myMotorDriver.setDrive(LEFT_MOTOR, 0, lspeed);
  myMotorDriver.setDrive(RIGHT_MOTOR, 1, lspeed);
  delay(5);
}


void loop()
{
  //pass setDrive() a motor number, direction as 0(call 0 forward) or 1, and level from 0 to 255
  brake();
  while (digitalRead(8) == 0); //Hold if jumper is placed between pin 8 and ground

  //***** Operate the Motor Driver *****//
  //  This walks through all 34 motor positions driving them forward and back.
  //  It uses .setDrive( motorNum, direction, level ) to drive the motors.

  //Smoothly move one motor up to speed and back (drive level 0 to 255)
  for (int i = 0; i < 4; i++)
  {
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
