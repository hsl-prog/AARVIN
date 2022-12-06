// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>                                // I2C/TWI Library
#include <SparkFun_MMC5983MA_Arduino_Library.h>  // Micro Magenetometer Library
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


// ------------------------------ Global Variables -------------------------------

/* Compass */
SFE_MMC5983MA myMag;               // Create Magnetometer object

/* RVR */
static DriveControl driveControl;  // Drive control for the RVR

/* Heading */
int currentHeading = 0;  // the compass direction AARVIN is facing, in degrees
int headingOffset = 0;   // the offset from where the RVR compass initializes


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
   FUNCTION: getCurrentHeading();
   ARGUMENTS: None
   RETURNS: double - heading

   DESCRIPTION:
   get_Heading() reads from the Mini Magnetometer to get the direction the RVR is
   currently facing

   SOURCES:
   Magnetometer Example Code
*/
void getCurrentHeading()
{
  // variables
  uint32_t rawX = 0;
  uint32_t rawY = 0;
  uint32_t rawZ = 0;
  int normX = 0;
  int normY = 0;
  int normZ = 0;

  // read all 3 channels simultaneously
  myMag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

  // Normalize each channel; zero, the midpoint, is 2^17 = 131072
  normX = (int)rawX - 131072;
  normX /= 131072.0;

  normY = (int)rawY - 131072;
  normY /= 131072.0;

  normZ = (int)rawZ - 131072;
  normZ /= 131072.0;

  // Convert X and Y into heading using Arc Tangent 2 (atan2)
  currentHeading = atan2(normX, 0 - normY);

  // Convert heading into degrees
  currentHeading /= PI;
  currentHeading *= 180;
  currentHeading += 180;
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
  // calculate heading offset
  int heading = (driveHeading + headingOffset) % 360;
  
  // drive at driveHeading at speed driveSpeed for driveTime seconds
  driveControl.rollStart(heading, driveSpeed);
  delay(driveTime * SECOND);

  // stop driving
  driveControl.rollStop(heading);
}


// ----------------------------------- Setup ------------------------------------

/*
    FUNCTION: void setup()
    ARGUMENTS: None
    RETURNS: None

    DESCRIPTION:
    The setup function runs once when you start the board. It initializes all
    sensors and prepares for the loop() function to run. Among the initialization
    processes, setup() intializes communication between the Arduino Uno and the
    Sphero RVR, intializes the GPS, the distance sensors, and the magnetometer.

    The Arduino Uno communicates with the RVR via a USB-A to USB-B cable
    with Serial communications. Sphero's RVR Library is proprietary, and has
    4 different methods of controling the drive function of the RVR, along
    with functions to control the LEDs on the RVR, to check on the battery
    state, and sleep monitoring.

    The GPS module is the XA1110 Qwiic GPS+GLONASS module from Sparkfun.
    It is accurate to about a 5 foot radius.
    The default I2C address is 0x10.

    The Compass module is the MMC5983MA Qwiic Micro Magnetometer, also from
    Sparkfun. It is a 3-axis magentic sensor with on-chip signal processing.
    It communicates via I2C communication, and has an accuracy of half a degree.
    The default I2C address is 0x30.

    The distance sensor is a VL53L1X Qwiic I2C distance sensor module from
    Sparkfun. It functions using an infrared laser time-of-flight system.
    The accuracy of the sensor is around 5mm, with a minimum read distance
    of 1cm. The max read distance varies depending on the object being
    detected. The default address for the distance sensor is 0x29.
*/
void setup()
{
  // Initialize communications with the RVR
  rvr.configUART(&Serial);                  // configure UART
  delay(2 * SECOND);                        // wait for the RVR to "wake up"
  driveControl = rvr.getDriveControl();     // get RVR's DriveControl

  driveControl.setHeading(0);               // set RVR compass to 0


  // Initialize Arduino pins for output
  pinMode(LED_BUILTIN, OUTPUT);            // built-in led on the board
  
  
  // start I2C communications and Serial communications
  Wire.begin();                            // start I2C for the distance sensors
  Serial.begin(115200);                    // start Serial at speed 115200 baud


  // Initialize MMC5983MA Micro Magnetometer sensor
  while (myMag.begin() == false)
  {
    Serial.println("Magnetometer did not respond");
    delay(500);
  }
  Serial.println("Magnetometer found!");
  myMag.softReset();  // reset magnetometer for accuracy


  // Set the RVR compass to due North
  getCurrentHeading();
  headingOffset = (360 - currentHeading) % 360;


  // run okayBlink() to show that all systems are a go :)
  okayBlink();
  delay(1 * SECOND);
}


// --------------------------------- Main Code ----------------------------------

void loop()
{
  // drive at the current heading (RVR 0)
  drive(headingOffset, DRIVESPEED, DRIVETIME); 

  // drive due North
  drive(0, DRIVESPEED, DRIVETIME);

  // blink
  okayBlink();
}
