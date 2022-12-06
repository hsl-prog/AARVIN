
/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2022

   Drive Device - Due Primary Device
   Version 1.0

   Built for Sphero RVR, Arduino Due, and Arduino Uno with Arducam Arduino camera
   module, Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor.

   https://www.arduino.cc/reference/en/
   https://github.com/sphero-inc/sphero-sdk-arduino-cpp
   https://github.com/mikalhart/TinyGPSPlus

*/

// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>


// ----------------------------------- Macros ------------------------------------

#define SECOND 1000  // 1000 ms = 1 sec

/* Modulate the speed of the RVR */
#define TURNSPEED 15           // turn at speed 10 / 100
#define DRIVESPEED 40          // drive at speed 30 / 100

/* Modulate the time the RVR will drive */
#define TURNTIME 1             // turn for 2 seconds
#define DRIVETIME 30           // drive for 20 seconds

/* Drive Control */
#define DRIVEADDR 9     // Drive control secondary device address for the RVR
#define NAVADDR 6       // Navigation primary device
#define TURN 1
#define DRIVE 2


// ------------------------------ Global Variables -------------------------------

int headingOffset = 0;             // offset for the internal RVR compass vs ours


// ---------------------------------- Functions ----------------------------------

/*
   FUNCTION: sendInfo()
   ARGUMENTS: int iHeading, int iDrive
   RETURNS: None

   DESCRIPTION:
   sendInfo() uses Wire to transfer instructions from the Due primary device to
   the Uno secondary device, which controls the RVR. iHeading is the heading we
   want to drive in, and iDrive specifies whether TURNSPEED and TURNTIME should
   be use, or if DRIVESPEED and DRIVETIME should be used.

   SOURCES: 
   https://docs.arduino.cc/learn/communication/wire
*/
void sendInfo(int info)
{ 
  // send info 
  Wire.beginTransmission(DRIVEADDR);
  Wire.write(info);
  Wire.endTransmission();
  delay(500);
}


// ------------------------------------ Setup ------------------------------------

void setup()
{
  // Initialize Wire communications
  Wire.begin();

  // pretend to get heading 
  // pretend to calculate heading offset
  headingOffset = 180;

  // send the heading Offset
  sendInfo(headingOffset);
}


// ------------------------------------ Main -------------------------------------

void loop()
{
  // send heading 
  sendInfo(180);

  // send drive
  sendInfo(DRIVE);
  delay(DRIVETIME + 1000);

  // send heading 
  sendInfo(90);

  // send turn
  sendInfo(TURN);
  delay(TURNTIME + 1000);

}
