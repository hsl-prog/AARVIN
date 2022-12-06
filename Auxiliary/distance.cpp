
// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>                                // I2C/TWI Library
#include <SparkFun_VL53L1X.h>                    // Distance Sensor Library
#include <SparkFun_MMC5983MA_Arduino_Library.h>  // Micro Magenetometer Library
#include <SpheroRVR.h>                           // Library for the Sphero RVR

// ----------------------------------- Macros ------------------------------------

/* Modulate the speed of the RVR */
#define TURNSPEED 15           // turn at speed 10 / 100
#define DRIVESPEED 40          // drive at speed 30 / 100

/* Modulate the time the RVR will drive */
#define TURNTIME 1             // turn for 1 second
#define DRIVETIME 30           // drive for 20 seconds

/* Drive distance */
#define DRIVEDISTANCE 600      // approximately how many mm AARVIN moves / drive

/* Obstacle distance */
#define OBSDISTANCE 650        // how many mm for AARVIN to be "blocked" ; ~2ft

/* Obstacle turn amount */
#define OBSTURN 30

/* Convert milliseconds to seconds */
#define SECOND 1000


// ------------------------------ Global Variables -------------------------------

/* Distance Sensor */
SFEVL53L1X distanceSensor;         // Create distance sensor object

/* RVR */
static DriveControl driveControl;  // Drive control for the RVR

/* Heading */
int currentHeading = 0;  // the compass direction AARVIN is facing, in degrees

/* Obstacle Distance */
int currentDistance = 0;    // approximately how far away any object is


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
void drive(int driveHeading, int driveSpeed, int driveTime)
{
  // drive at driveHeading at speed driveSpeed for driveTime seconds
  driveControl.rollStart(driveHeading, driveSpeed);
  delay(driveTime * SECOND);

  // stop driving
  driveControl.rollStop(driveHeading);
}

/*
   FUNCTION: getObsDistance()
   ARGUMENTS: None
   RETURNS: int distance

   DESCRIPTION:
   checks too see how far away anything in the view range is
*/
int getObsDistance()
{
  distanceSensor.startRanging();   // initiate measurement

  // Get results of measurement
  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  return distance;
}

/*
  FUNCTION: avoidObstacle()
  ARGUMENTS: None
  RETURNS: None

  DESCRIPTION:
  avoid_obstacle() keeps the RVR from running into things if we are blocked.
  It will check to see if it is blocked, and while it is blocked, it will turn
  AARVIN OBSTURN degrees until it is no longer blocked. Then AARVIN will drive a
  few feet in that direction.
*/
void avoidObstacle()
{
  // get current heading
  // ----- int currH = getCurrentHeading();

  // get current distance
  currentDistance = getObsDistance();

  // turn OBSTURN degrees while AARVIN has a blockage in its path
  while (currentDistance <= OBSDISTANCE)
  {
    // update the current heading
    // ----- currH = getCurrentHeading();
    currentHeading += OBSTURN;

    // turn OBSTURN degrees
    drive(currentHeading, TURNSPEED, TURNTIME);

    // update current distance
    currentDistance = getObsDistance();
  }

  // drive forward a few feet
  drive(currentHeading, DRIVESPEED, DRIVETIME);
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


  // Initialize distance sensor
  while (distanceSensor.begin() != 0)
  {
    Serial.println("Distance sensor did not respond");
    delay(0.5 * SECOND);
  }
  Serial.println("Distance sensor found!");


  // run blink to show that systems are working
  okayBlink();


  // get currentDistance 
  currentDistance = getObsDistance();
}

// --------------------------------- Main Code ----------------------------------

/*
   FUNCTION: loop()
   ARGUMENTS: None
   RETURNS: None

   DESCRIPTION:
   loop() is the Arduino's "main" code. It will repeat infinitely until an exit
   code is given or the Arduino is unpowered.
*/
void loop()
{
  if (currentDistance <= OBSDISTANCE)
  {
    // turn until no longer blocked to avoid any obstacles
    avoidObstacle();
  }

  currentDistance -= DRIVEDISTANCE;
  
  // drive forward a few feet
  drive(currentHeading, DRIVESPEED, DRIVETIME);

  okayBlink();
  delay(0.5 * SECOND);
}
