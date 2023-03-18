/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2023

   Version 2.0

   Sources:
       https://www.arduino.cc/reference/en/
       https://www.sparkfun.com/products/15451 
*/

// ----------------------------------- Libraries -----------------------------------

#include <Wire.h>
#include <Arduino.h>
#include <stdint.h> 
#include "SCMD.h"
#include "SCMD_config.h"
#include "Drive.h"


// ------------------------------------ Macros ------------------------------------

#define DRIVESPEED 100
#define DRIVETIME 15
#define TURNSPEED 200
#define TURNTIME 5


// ----------------------------------- Objects -----------------------------------

Tank myTank; 


// ---------------------------------- Functions ----------------------------------

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Starting Drive Code."); 
  
  myTank.tankprep();

}


void square_dance()
{
  for (int i = 0; i < 5; i ++)
  {
    myTank.forward(DRIVESPEED, DRIVETIME);
    myTank.turn_right(TURNSPEED, TURNTIME);
  }

  for (int i = 0; i < 5; i++)
  {
    myTank.turn_left(TURNSPEED, TURNTIME); 
    myTank.reverse(DRIVESPEED, DRIVETIME); 
  }
  
}


void loop() 
{
  // put your main code here, to run repeatedly:

}
