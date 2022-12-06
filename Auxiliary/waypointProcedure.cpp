
// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>                                // I2C/TWI Lib
#include <SpheroRVR.h>                           // Library for the Sphero RVR


// ----------------------------------- Macros ------------------------------------

/* Modulate the speed of the RVR */
#define TURNSPEED 15           // turn at speed 10 / 100
#define DRIVESPEED 40          // drive at speed 30 / 100

/* Modulate the time the RVR will drive */
#define TURNTIME 1             // turn for 2 seconds
#define DRIVETIME 30           // drive for 20 seconds

/* Convert milliseconds to seconds */
#define SECOND 1000            // 1000ms = 1s

/* Camera Pin */
#define CAMERA 3               // the PWM pin that the camera module is on


// ------------------------------ Global Variables -------------------------------

/* RVR */
static DriveControl driveControl;  // Drive control for the RVR

/* Heading */
int currentHeading = 0;  // the compass direction AARVIN is facing, in degrees


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
  FUNCTION: takePicture()
  ARGUMENTS: None
  RETURNS: None

  DESCRIPTION:
  takePicture() runs the Adafruit Mini Spy Camera to take a photo.

  The spy camera is capable of taking up to 1000 pictures, labeled PIC000
  through PIC999. It has a built-in SD card system that is encapsulated, so the
  Arduino never handles the photo information. The spy camera can also take a
  video, limited by your SD card size. The camera is compatible with up to 32gb
  microSD cards. The microSD card must be formatted in FAT32 format to be
  compatible for usage with the spy camera

  The spy camera has 3 cords: power, ground, and control. The control wire is
  the green wire. Older versions of the camera use a white wire instead.

  To connect the camera to your board, wire the red power wire to 5V power, the
  black ground wire to ground (GND), and wire the white or green control wire to
  a digital PWM pin or a GPIO pin. For AARVIN, I am using the PWM digital
  pin 3 on the Arduino. To set the camera up, the pin must be initialized to be
  outputing HIGH using digitalWrite(), and then to trigger the camera, you'll
  output LOW, also using digitalWrite().

  To take a photo, pulse the power LOW on the control wire for less than 100ms
  but more than 30ms, then reset power to HIGH to release the trigger.

  To take a video, pulse power LOW for over 100ms and wait, keeing the power on
  LOW to allow the camera to film your video for however long you want, within
  the limits of your microSD card, and then when you're done filming pulse the
  power HIGH again to stop the video.
*/
void takePicture()
{
  delay(0.5 * SECOND);
  // Take a picture with the spy camera; low = "button press" or "trigger on"
  digitalWrite(CAMERA, LOW);   // press the camera trigger
  delay(40);
  digitalWrite(CAMERA, HIGH);  // release the camera trigger
  delay(0.5 * SECOND);
}

/*
  FUNCTION: drive()
  ARGUMENTS: None
  RETURNS: None

  DESCRIPTION:
  drive() will drive the rover at the heading, speed, and amount of time
  given by the arguments driveHeading, driveSpeed, and driveTime
*/
void drive(int driveHeading, int driveSpeed, int driveTime)
{
  // drive at driveHeading at speed driveSpeed for driveTime seconds
  driveControl.rollStart(driveHeading, driveSpeed);
  delay(driveTime * SECOND);

  // stop driving
  driveControl.rollStop(driveHeading);
  delay(1 * SECOND);
}

/*
   FUNCTION: waypoint_procedure()
   ARGUMENTS: None
   RETURN: None

   DESCRIPTION:
   waypoint_procedure() directs the rover to turn 90 degrees 4 times and stop at
   each position, allowing the RVR to retreive data such as altitude, photos, or
   distance information, and store it.
*/
void waypointProcedure()
{
  // variables
  // int wdirection = getCurrentHeading();  // AARVIN's starting heading
  int wdirection = currentHeading;  // delete this line and uncomment line above

  // Turn 90 degrees to the right 4 times and take a picture each time
  for (int i = 0; i < 4; i++)
  {
    // Calculate turn and drive that direction
    wdirection = (wdirection + 90) % 360;     // calculate direction
    drive(wdirection, TURNSPEED, TURNTIME);   // turn that direction

    // Perform Waypoint Tasks
    takePicture();                         // take a picture
  }
}


// ----------------------------------- Setup ------------------------------------

void setup() 
{
  // Initialize communications with the RVR
  rvr.configUART(&Serial);                  // configure UART
  delay(2 * SECOND);                        // wait for the RVR to "wake up"
  driveControl = rvr.getDriveControl();     // get RVR's DriveControl

  driveControl.setHeading(0);               // set RVR compass to 0

  
  // Initialize Arduino pins for output
  pinMode(LED_BUILTIN, OUTPUT);            // built-in led on the board
  pinMode(CAMERA, OUTPUT);                 // the PWM pin for the camera trigger

  // Initialize the camera module
  digitalWrite(CAMERA, HIGH);


  // start I2C communications and Serial communications
  Wire.begin();                            // start I2C for the distance sensors
  Serial.begin(115200);                    // start Serial at speed 115200 baud


  // run okayBlink() to show that all systems are a go :)
  okayBlink();

  delay(1 * SECOND);
}


// --------------------------------- Main Code -----------------------------------

void loop() 
{
  drive(currentHeading, DRIVESPEED, DRIVETIME);
  waypointProcedure();
}
