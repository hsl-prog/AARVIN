/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2023

   Version 2.0

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
#include "Drive.h"


// ------------------------------------ Macros ------------------------------------
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define DRIVESPEED 100
#define DRIVETIME 2000
#define TURNSPEED 115
#define TURNTIME 1000


// ------------------------------------ Objects -----------------------------------
SCMD myMotorDriver; 
Tank myTank;


// ----------------------------------- Functions ----------------------------------
//  Pin 8 can be grounded to disable motor movement, for debugging.

void setup()
{ 
  myTank.tankprep();
}


void loop()
{
  //pass setDrive() a motor number, direction as 0(call 0 forward) or 1, and level from 0 to 255
  myTank.brake();
  delay(5);
  while (digitalRead(8) == 0); //Hold if jumper is placed between pin 8 and ground

  myTank.square_dance();  
}
