
/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2022

   Drive Device - Uno Secondary Device
   Version 1.0

   Built for Sphero RVR, Arduino Due, and Arduino Uno with Arducam Arduino camera 
   module, Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor.

   https://www.arduino.cc/reference/en/
   https://github.com/sphero-inc/sphero-sdk-arduino-cpp
   https://github.com/mikalhart/TinyGPSPlus   

*/

// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>
#include <SpheroRVR.h>                           // Library for the Sphero RVR


// ----------------------------------- Macros ------------------------------------

#define SECOND 1000  // 1000 ms = 1 sec

/* Modulate the speed of the RVR */
#define TURNSPEED 15           // turn at speed 10 / 100
#define DRIVESPEED 40          // drive at speed 30 / 100

/* Modulate the time the RVR will drive */
#define TURNTIME 1             // turn for 2 seconds
#define DRIVETIME 30           // drive for 20 seconds

/* Drive Control */
#define DRIVEADDR 9  // Drive control address for the RVR
#define NAVADDR 6    // Navigation primary device
#define TURN 1
#define DRIVE 2


// ------------------------------ Global Variables -------------------------------

static DriveControl driveControl;  

int driveHeading = -1;
int driveSpeed = -1;
int driveTime = -1;

int iHeading = -1;
int iMove = -1;

int headingOffset = -1;             // offset for the internal RVR compass vs ours


// ---------------------------------- Functions ----------------------------------

/*
   FUNCTION: okayBlink()
   ARGUMENTS: None
   RETURNS: None

   DESCRIPTION:
   Turns an LED on for one second, then off for one second, repeatedly.

   On the UNO the built-in LED is attached to digital pin 13.
   LED_BUILTIN is set to the correct LED pin independent of which board is used.

   SOURCES:
   https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
void okayBlink()
{
  for (uint8_t i = 0; i < 5; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH v)
    delay(0.5 * SECOND);              // wait for 1/2 of a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW v)
    delay(0.5 * SECOND);              // wait for 1/2 of a second
  }
}

/*
  FUNCTION: drive()
  ARGUMENTS: None
  RETURNS: None

  DESCRIPTION:
  drive() will drive the rover at the heading, speed, and amount of time
  given by the arguments driveHeading, driveSpeed, and driveTime
*/
void drive()
{
  // calculate heading offset
  int heading = (driveHeading + headingOffset) % 360;

  // drive at driveHeading at speed driveSpeed for driveTime seconds
  driveControl.rollStart(heading, driveSpeed);
  delay(driveTime * SECOND);

  // stop driving
  driveControl.rollStop(heading);

  // reset global variables
  driveHeading = -1;
  driveSpeed = -1;
  driveTime = -1;
}

/*
   FUNCTION: receiveInfo()
   ARGUMENTS: int howMany
   RETURNS: None

   DESCRIPTION:
   Reads 2 pieces of data from the Primary Navigation Device: the heading, 
   and the type of movement we want to use with the RVR. 

   SOURCES:
   https://docs.arduino.cc/learn/communication/wire   
*/
void receiveInfo() 
{ 
  if (headingOffset == -1)
  {
    headingOffset = Wire.read();
  }
  else if (iHeading == -1)
  {
    iHeading = Wire.read();
  }
  else if (iMove == -1)
  {
    iMove = Wire.read();
  }

  if (iMove == TURN)
  {
    driveHeading = iHeading;
    driveSpeed = TURNSPEED;
    driveTime = TURNTIME;
    drive();
    
    iHeading = -1;
    iMove = -1;
  }
  else if (iMove == DRIVE)
  {
    driveHeading = iHeading;
    driveSpeed = DRIVESPEED;
    driveTime = DRIVETIME;
    drive();

    iHeading = -1;
    iMove = -1;
  }
}


// ------------------------------------ Setup ------------------------------------

/*
 * FUNCTION: setup()
 * ARGUMENTS: None
 * RETURNS: None
 * 
 * DESCRIPTION: 
 * setup() gets everything set up, initialized, and ready for looping
 */
void setup() 
{
  // Initialize communications with the RVR
  rvr.configUART(&Serial);                  // configure UART
  delay(2 * SECOND);                        // wait for the RVR to "wake up"
  driveControl = rvr.getDriveControl();     // get RVR's DriveControl
  driveControl.setHeading(0);    

  // Initialize built-in LED on the Arduino for output
  pinMode(LED_BUILTIN, OUTPUT);

  // Blink
  okayBlink();
  
  // Start the I2C Bus as secondary device on address 9
  Wire.begin(DRIVEADDR); 
  
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveInfo); 
}


// ------------------------------------ Main -------------------------------------

void loop() 
{
  delay(100);
}
 
