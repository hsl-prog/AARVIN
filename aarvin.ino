
/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2022

   Version 3.0

   Built for Sphero RVR, Arduino, with Arducam Adafruit mini spy camera module,
   Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor.

  References:
     1 - https://www.arduino.cc/reference/en/
     2 - https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
     3 - https://www.adafruit.com/product/3202
     4 - https://youtu.be/4Q1d2S6oVrw (skip to 3:11)
     5 - https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
     6 - https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/blob/main/examples/Example2-I2C_Digital_compass/Example2-I2C_Digital_compass.ino
     7 - https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/blob/master/examples/Example1_ReadDistance/Example1_ReadDistance.ino
     8 - https://github.com/sphero-inc/sphero-sdk-arduino-cpp-library-manager/blob/master/examples/driving/drive_with_helper/drive_with_helper.ino
     9 - https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
*/

// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>                                // I2C/TWI Lib
#include <SparkFun_I2C_GPS_Arduino_Library.h>    // GPS XA1110 Lib
#include <TinyGPS++.h>                           // TinyGPS Lib
#include <SparkFun_VL53L1X.h>                    // Distance Sensor Lib
#include <SparkFun_MMC5983MA_Arduino_Library.h>  // Micro Magenetometer Lib
#include <SpheroRVR.h>                           // Library for the Sphero RVR
#include "Waypoint.h"                            // AARVIN's Waypoint object


// ----------------------------------- Macros ------------------------------------

/* Convert milliseconds to seconds */
#define SECOND 1000            // 1000ms = 1s

/* Camera Pin */
#define CAMERA 3               // the PWM pin that the camera module is on

/* Modulate moving speed of the RVR */
#define TURNSPEED 15           // turn at speed 10 / 100
#define DRIVESPEED 40          // drive at speed 30 / 100

/* Modulate moving time the RVR */
#define TURNTIME 1             // turn for 2 seconds
#define DRIVETIME 30           // drive for 20 seconds

/* Distances for Obstacle Avoidance */
#define DRIVEDISTANCE 600      // Approximately how many mm AARVIN drives 
#define OBSDISTANCE 680        // how many mm for AARVIN to be "blocked"
#define OBSTURN 30             // degrees AARVIN turns when avoiding obstacles

/* Number of Waypoints in the route */
#define ROUTELEN 2             // (number of waypoints) + 1 = ROUTLEN

/* Coordinate Offset to provide a range of success for "reaching" a Waypoint */
#define COORDOFFSET 0.00005    // + or - degrees to offset coordinates by


// ---------------------------------- Objects ------------------------------------

/* GPS */
I2CGPS myI2CGPS;                   // Create I2C GPS object
TinyGPSPlus gps;                   // Create TinyGPS gps object

/* Compass */
SFE_MMC5983MA myMag;               // Create Magnetometer object

/* Distance Sensor */
SFEVL53L1X distanceSensor;         // Create distance sensor object

/* RVR */
static DriveControl driveControl;  // Drive control object for the RVR

/* Waypoints */
Waypoint startingPoint;            // AARVIN's starting point
Waypoint currentPoint;             // Waypoint to hold AARVIN's current location
Waypoint waypoint1;                // first waypoint, first stop on the route

/* Route - myWaypoints */
Waypoint myWaypoints[ROUTELEN] = {waypoint1, startingPoint};


// ------------------------------ Global Variables -------------------------------

/* Heading */
int currentHeading = 0;  // the compass direction AARVIN is facing, in degrees
int headingOffset = 0;   // the offset for internal RVR compass vs magnetometer
int headingToDrive = 0;  // the heading AARVIN will take to go to a waypoint

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

  REFERENCES:
  1 - Adafruit Mini Spy Camera Product Page
      https://www.adafruit.com/product/3202

  2 - Adafruit New Products 11/2/2016 video (embedded on product page)
      https://youtu.be/4Q1d2S6oVrw
      Skip to 3:11 for the Spy Camera
*/
void takePicture()
{
  delay(0.5 * SECOND);

  // "Press" the camera trigger
  digitalWrite(CAMERA, LOW);   // press the camera trigger
  delay(40);

  // Release the camera trigger
  digitalWrite(CAMERA, HIGH);  // release the camera trigger
  delay(0.5 * SECOND);
}


/*
   FUNCTION: getCurrentHeading();
   ARGUMENTS: None
   RETURNS: double - heading

   DESCRIPTION:
   get_Heading() reads from the Mini Magnetometer to get the direction the RVR is
   currently facing

   REFERENCES:
   1 - Sparkfun Mini Magnetometer Example Code by Nathan Seidle and Ricardo Ramos
      Link:
      https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/blob/main/examples/Example2-I2C_Digital_compass/Example2-I2C_Digital_compass.ino
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
   FUNCTION: getCurrentCoordinates()
   ARGUMENTS: None
   RETURNS: None

   DESCRIPTION:
   getCoordinates() checks the gps and retrieves latitude and longitute in the
   given argument array coordinates[], the first element of coordinates is the
   latitude, and the second element is the longitude

   REFERENCES:
   1 - TinyGPS++ Device Example Code by Mikhal Hart
       https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/DeviceExample/DeviceExample.ino
*/
void getCurrentCoordinates()
{
  // Feed the GPS parser
  gps.encode(myI2CGPS.read());

  // wait for location to be valid
  while (!gps.location.isValid())
  {
    Serial.println("Waiting on location to validate");
    delay(500);
  }

  // get the latitude
  int clat = gps.location.lat();

  // get the longitute
  int clong = gps.location.lng();

  // plug the latitude and longitude into currentPoint
  currentPoint.wlat = clat;
  currentPoint.wlong = clong;
}

/*
  FUNCTION: drive()
  ARGUMENTS: None
  RETURNS: None

  DESCRIPTION:
  drive() will drive the rover at the heading, speed, and amount of time
  given by the arguments driveHeading, driveSpeed, and driveTime

  REFERENCES:
  1 - Sphero RVR Arduino Library Examples
      https://github.com/sphero-inc/sphero-sdk-arduino-cpp-library-manager/blob/master/examples/driving/drive_with_helper/drive_with_helper.ino
*/
void drive(int driveHeading, int driveSpeed, int driveTime)
{
  // calculate heading offset
  driveHeading = (driveHeading + headingOffset) % 360;

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
   getObsDistance() checks the distance sensor to see how far away any potential
   obstacles are and returns the integer amount in mm.

   REFERENCES:
   1 - Sparkfun Qwiic Distance Sensor Example Code
       https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/blob/master/examples/Example1_ReadDistance/Example1_ReadDistance.ino
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
  avoid_obstacle() keeps AARVIN from running into things if we are blocked.
  It will check to see if it is blocked, and while it is blocked, it will turn
  AARVIN OBSTURN degrees until it is no longer blocked. Then AARVIN will drive a
  few feet in that direction.
*/
void avoidObstacle()
{
  // turn OBSTURN degrees while AARVIN has a blockage in its path
  while (currentDistance <= OBSDISTANCE)
  {
    // update the current heading
    getCurrentHeading();
    currentHeading += OBSTURN;

    // turn OBSTURN degrees
    drive(currentHeading, TURNSPEED, TURNTIME);

    // update current distance
    currentDistance = getObsDistance();
  }

  // drive forward a few feet
  drive(currentHeading, DRIVESPEED, DRIVETIME);
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
  // update current heading
  getCurrentHeading();  // AARVIN's starting heading

  // Turn 90 degrees to the right 4 times and take a picture each time
  for (int i = 0; i < 4; i++)
  {
    // Calculate turn and drive that direction
    currentHeading = (currentHeading + 90) % 360;     // calculate direction
    drive(currentHeading, TURNSPEED, TURNTIME);   // turn that direction

    // Perform Waypoint Tasks
    takePicture();                         // take a picture
  }
}

/*
    FUNCTION: calculateHeading()
    ARGUMENTS: waypoint Destination
    RETURNS: int heading

    DESCRIPTION:
    calculateHeading() will use currentPoint waypoint and the given waypoint as
    the destination. currentPoint will be point A and destination will be point B.

    Using trigonometry, we will calulate the direct heading that AARVIN will
    take to get to its destination. The math is as follows:

    deltaLong = longeB - longA
    Y = sin(deltaLong) * cos(latB)
    X = cos(latA) * sin(latB) - sin(latA) * cos(latB) * cos(deltaLong)
    heading = atan2(Y, X)
    heading = heading * 180/pi

    REFERENCES:
    1 - TinyGPS++ Library Source Code by Mikhal Hart
        https://github.com/mikalhart/TinyGPSPlus/blob/master/src/TinyGPS%2B%2B.cpp
*/
int calculateHeading(Waypoint destination)
{
  // delta of Longitude
  double dLong = radians(destination.wlong - currentPoint.wlong);

  double lat1 = radians(currentPoint.wlat);
  double lat2 = radians(destination.wlat);

  // X, Y
  double y = sin(dLong) * cos(lat2);

  double x = (cos(lat1) * sin(lat2)) - (sin(lat1) * cos(lat2) * cos(dLong));

  // atan2 y, x
  double dheading = atan2(y, x);

  // offset for cosine graph
  if (dheading < 0.0)
  {
    dheading += TWO_PI;
  }

  // convert radians to degrees
  dheading = dheading * 180 / 3.141592;

  // cast to an int
  int heading = dheading;

  return heading;
}

/*
    FUNCTION: navigate()
    ARGUMENTS: None
    RETURN: None

    DESCRIPTION:
    navigate() directs the rover to navigate along the coordinates
    contained by the route[] array.

    We start at point start, then navigate to Waypoint[0]. Then we
    proceed to navigate from 0 to 1, 1 to 2, 2 to 3, 3 to 4, and then
    from 4 back to start.
*/
void navigate()
{
  // iterate through each waypoint in the array myWaypoints
  for (int i = 0; i < ROUTELEN; i++)
  {
    // get our current coordinates
    getCurrentCoordinates();

    // calculate the heading at which we need to drive to reach our destination
    headingToDrive = calculateHeading(myWaypoints[i]);

    // turn to that heading
    drive(headingToDrive, TURNSPEED, TURNTIME);

    // Check the distance to see if we are blocked
    currentDistance = getObsDistance();

    // drive to myWaypoints[i] with accuracy of +- 0.00005 degrees
    while ((currentPoint.wlat < (myWaypoints[i].wlat - COORDOFFSET))   ||
           (currentPoint.wlat > (myWaypoints[i].wlat + COORDOFFSET))   ||
           (currentPoint.wlong < (myWaypoints[i].wlong - COORDOFFSET)) ||
           (currentPoint.wlong > (myWaypoints[i].wlong + COORDOFFSET)))
    {
      // if we are not blocked, then drive
      if (currentDistance > OBSDISTANCE)
      {
        // drive at our heading
        drive(headingToDrive, DRIVESPEED, DRIVETIME);

        // subtract the amount we moved from the distance to our obstacle
        currentDistance -= DRIVEDISTANCE;
      }

      // if we are blocked, avoid and reset heading
      if (currentDistance <= OBSDISTANCE)
      {
        // go around the obstacle
        avoidObstacle();

        // reset our coordinates and heading
        getCurrentCoordinates();
        headingToDrive = calculateHeading(myWaypoints[i]);
        drive(headingToDrive, TURNSPEED, TURNTIME);
        currentDistance = getObsDistance();
      }
    }

    // Once we reach the waypoint, follow the waypoint procedure
    waypointProcedure();
  }
}


// ------------------------------------ Setup ------------------------------------

/*
    FUNCTION: void setup()
    ARGUMENTS: None
    RETURNS: None

    DESCRIPTION:
    The setup() function runs once when AARVIN is turned on. setup() initializes
    all sensors and prepares any necessary variables for the main program to
    start running.

    The Arduino communicates with the RVR via a USB cable and Serial
    communications. Sphero's RVR Library for Arduino has 4 different methods of
    controling the drive function of the RVR, along with functions to control the
    LEDs on the RVR, to check on the battery state, and perform sleep monitoring.

    The GPS module is the XA1110 Qwiic GPS+GLONASS module from Sparkfun.
    It is accurate to approximately a 5 foot radius, or 10 foot diameter.

    The compass module is the MMC5983MA Qwiic Micro Magnetometer, also from
    Sparkfun. It is a 3-axis magentic sensor with on-chip signal processing.
    It communicates via I2C communication, and has an accuracy of 0.5 degrees.

    The distance sensor is a VL53L1X Qwiic I2C distance sensor module from
    Sparkfun. It functions using an infrared laser time-of-flight system.
    The accuracy of the sensor is around 5mm, with a minimum read distance
    of 1cm. The maximum read distance varies depending on the object being
    detected. The further away an object is, the longer it takes for the
    distance sensor to be able to detect it.

    The camera module is the Adafruit Mini Spy Camera, and it is essentially
    the guts of a cheap keychain camera. It is controlled on a PWM-capable
    digital I/O pin via the green wire. Older versions of the camera use a white
    wire. The module requires 5V power via the red wire, and the black wire is
    used to connect to Ground. To initialize the camera, digitalWrite HIGH to the
    PWM digital I/O pin it is wired to. In AARVIN, the camera is on Pin 9, as
    stored in the CAMERA macro. To take a photo, digitalWrite LOW to the camera
    for less than 100ms then digitalWrite HIGH to release. To take a video,
    digitalWrite LOW to the camera for at least 100ms, and digitalWrite HIGH once
    the video is your desired length in ms.
*/
void setup()
{
  // Initialize communications with the RVR
  rvr.configUART(&Serial);                  // configure UART
  delay(2 * SECOND);                        // wait for the RVR to "wake up"
  driveControl = rvr.getDriveControl();     // get RVR's DriveControl
  driveControl.setHeading(0);               // set RVR compass to 0

  // Initialize LED and Camera
  pinMode(LED_BUILTIN, OUTPUT);            // built-in led on the board
  pinMode(CAMERA, OUTPUT);                 // the PWM pin for the camera trigger
  digitalWrite(CAMERA, HIGH);              // start the camera module

  // Begin communications
  Wire.begin();                            // start I2C for the distance sensors
  Serial.begin(115200);                    // start Serial at speed 115200 baud

  // Initialize I2C GPS XA1110 module
  while (myI2CGPS.begin() == false)
  {
    delay(500);
  }

  // Initialize MMC5983MA Micro Magnetometer sensor
  while (myMag.begin() == false)
  {
    delay(500);
  }
  myMag.softReset();  // reset magnetometer for accuracy

  // Initialize distance sensor
  while (distanceSensor.begin() != 0)
  {
    delay(0.5 * SECOND);
  }

  // Reset the RVR compass to due North
  getCurrentHeading();  // get the current heading
  headingOffset = (360 - currentHeading) % 360; // calculate offset from North

  // setup currentPoint Waypoint and starting point Waypoint
  getCurrentCoordinates();
  startingPoint.wlat = currentPoint.wlat;
  startingPoint.wlong = currentPoint.wlong;

  // setup route of Waypoint(s)
  waypoint1.wlat = 33.979511;        // latitude of waypoint1
  waypoint1.wlong = -84.628333;      // longitude of waypoint1

  // run blink to show that systems are working
  okayBlink();
  delay(1 * SECOND);
}


// ---------------------------------- Main Code ----------------------------------

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
  navigate();
  exit(0);
}
