
// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>                                // I2C/TWI Lib
#include <SparkFun_I2C_GPS_Arduino_Library.h>    // GPS XA1110 Lib
#include <TinyGPS++.h>                           // TinyGPS Lib
#include <SparkFun_MMC5983MA_Arduino_Library.h>  // Micro Magenetometer Lib

#include "Waypoint.h"                            // My Waypoint Library


// ----------------------------------- Macros ------------------------------------

/* Convert milliseconds to seconds */
#define SECOND 1000            // 1000ms = 1s


// ------------------------------ Global Variables -------------------------------

/* GPS */
I2CGPS myI2CGPS;                   // Create I2C GPS object
TinyGPSPlus gps;                   // Create TinyGPS gps object

/* Compass */
SFE_MMC5983MA myMag;               // Create Magnetometer object

/* Heading */
int currentHeading = 0;  // the compass direction AARVIN is facing, in degrees
int headingOffset = 0;   // the offset for internal RVR compass vs magnetometer

/* Waypoints */
Waypoint startingPoint;            // AARVIN's starting point
Waypoint currentPoint;             // Waypoint to hold AARVIN's current location


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
    Serial.println("blink"); 
    
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH v)
    delay(0.5 * SECOND);              // wait for 1/2 of a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW v)
    delay(0.5 * SECOND);              // wait for 1/2 of a second
  }
  
  Serial.println("Done blinking!");
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
  Serial.println("Calculating Current Heading..."); 
  
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
  currentHeading = degrees(currentHeading);

  Serial.print("Current Heading: ");
  Serial.println(currentHeading);
}

/*
   FUNCTION: getCurrentCoordinates()
   ARGUMENTS: None
   RETURNS: None

   DESCRIPTION:
   getCoordinates() checks the gps and retrieves latitude and longitute in the
   given argument array coordinates[], the first element of coordinates is the
   latitude, and the second element is the longitude

   SOURCES:
   GPS example code
*/
void getCurrentCoordinates()
{
  Serial.println("Getting Current Coordinates...");
  
  // Feed the GPS parser
  gps.encode(myI2CGPS.read());

  // wait for location to be valid
  while (!gps.location.isValid())
  {
    Serial.println("Waiting on location to validate");
    delay(1 * SECOND);
  }

  // get the latitude
  int clat = gps.location.lat();

  // get the longitute
  int clong = gps.location.lng();

  // plug the latitude and longitude into currentPoint
  currentPoint.wlat = clat; 
  currentPoint.wlong = clong; 

  Serial.println("Current Coordinates:");
  Serial.println(clat);
  Serial.println(clong);
}


// ------------------------------------ Setup ------------------------------------

void setup() 
{
  delay(5 * SECOND);
  
  // Initialize Arduino pins for output
  pinMode(LED_BUILTIN, OUTPUT);            // built-in led on the board
  
  
  // start I2C communications and Serial communications
  Wire.begin();                            // start I2C
  Serial.begin(115200);                    // start Serial at speed 115200 baud
  

  // Initialize I2C GPS XA1110 module
  while (myI2CGPS.begin() == false)
  {
    Serial.println("GPS failed to respond! Please check wiring!");
    delay(1 * SECOND);
  }
  Serial.println("GPS module found!");
  

  // Initialize MMC5983MA Micro Magnetometer sensor
  while (myMag.begin() == false)
  {
    Serial.println("Magnetometer did not respond");
    delay(1 * SECOND);
  }
  Serial.println("Magnetometer found!");
  myMag.softReset();  // reset magnetometer for accuracy


  // setup Current Heading
  getCurrentHeading();
  

  // setup currentPoint Waypoint
  getCurrentCoordinates();
  

  // setup startingPoint Waypoint
  startingPoint.wlat = currentPoint.wlat;
  startingPoint.wlong = currentPoint.wlong;
  

  // Print out starting point data to Serial
  Serial.println("Starting Point:");
  Serial.println(startingPoint.wlat);
  Serial.println(startingPoint.wlong);

  // run okayBlink() to show that all systems are a go :)
  okayBlink();
  delay(1 * SECOND);
}


// ---------------------------------- Main Code ----------------------------------

void loop() 
{
  getCurrentCoordinates();
  delay(2 * SECOND);

  getCurrentHeading();
  delay(2 * SECOND);

  Serial.println("Starting Point:");
  Serial.println(startingPoint.wlat);
  Serial.println(startingPoint.wlong);

  okayBlink();

}
