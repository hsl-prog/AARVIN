
/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2022

   Drive Device - Uno Slave Device
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


// --------------------------------- Libriaries ----------------------------------

#define SECOND 1000 


// ------------------------------ Global Variables -------------------------------

static DriveControl driveControl;  // Drive control for the RVR
int drive_device_addr = 9;         // what address is the drive device on
int driveInfo[] = {0, 0, 0};       // array to hold drive info


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
  for (uint8_t i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH v)
    delay(0.5 * SECOND);                       // wait for 1/2 of a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW v)
    delay(0.5 * SECOND);                       // wait for 1/2 of a second
  }
}


/*
   FUNCTION: receiveWire()
   ARGUMENTS: None
   RETURNS: None

   DESCRIPTION:
   Reads a data transmission from I2C Wire

   SOURCES:
   https://www.instructables.com/I2C-between-Arduinos/
*/
void receiveWire() 
{
  // read from IC2
  driveInfo[0] = Wire.read(); 
  driveInfo[1] = Wire.read();
  driveInfo[2] = Wire.read();  

}


/*
  FUNCTION: drive()
  ARGUMENTS: None
  RETURNS: None

  DESCRIPTION: 
  drive() will drive the rover at the heading, speed, and amount of time 
  described in driveInfo. 

  driveInfo[] = {wHeading, wSpeed, wTime}
 */
 void drive()
 {
   // extract info from driveInfo
   int wHeading = driveInfo[0];
   int wSpeed = driveInfo[1];
   int wTime = driveInfo[2];
  
   // drive forward at speed 40 for 10 seconds
   driveControl.rollStart(wHeading, wSpeed);
   delay(wTime * SECOND);

   // stop driving
   driveControl.rollStop(wHeading);  
   delay(2 * SECOND);

   // reset drive Info
   driveInfo[0] = 0;
   driveInfo[1] = 0;
   driveInfo[2] = 0;
 }


// ------------------------------------ Setup ------------------------------------

/*
   FUNCTION: setup()
   ARGUMENTS: None
   RETURNS: None
   
   DESCRIPTION: 
   setup() gets everything set up, initialized, and ready for looping
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
  
  // Start the I2C Bus as Slave on address 9
  Wire.begin(drive_device_addr); 
  
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveWire); 
}


// ------------------------------------ Main -------------------------------------

void loop() 
{
  drive();
}
 
