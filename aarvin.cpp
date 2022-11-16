
/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2022

   Version 1.5

   Built for Sphero RVR and Arduino Uno R3 with Arducam Arduino camera module,
   Sparkfun Qwiic GPS, Distance Sensors, Mux Breakout, Qwiic Arduino Shield.

   https://www.arduino.cc/reference/en/
   https://github.com/sphero-inc/sphero-sdk-arduino-cpp
   https://github.com/mikalhart/TinyGPSPlus   

*/

// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>                                // I2C/TWI Lib
#include <SpheroRVR.h>                           // Library for the Sphero RVR
#include <SparkFun_I2C_GPS_Arduino_Library.h>    // GPS XA1110 Lib
#include <TinyGPS++.h>                           // TinyGPS Lib
#include <SparkFun_VL53L1X.h>                    // Distance Sensor Lib
#include <SparkFun_MMC5983MA_Arduino_Library.h>  // Micro Magenetometer Lib


// ----------------------------------- Macros ------------------------------------

#define SPEEDMULTIPLIER 1        // 1 = 100%, which is full speed 
#define SECOND 1000              // 1000 = 1 second


// ------------------------------ Global Variables -------------------------------

static DriveControl driveControl;  // Drive control for the RVR
I2CGPS myI2CGPS;                   // Create I2C GPS object
TinyGPSPlus gps;                   // Create TinyGPS gps object
SFE_MMC5983MA myMag;               // Create Magnetometer object
SFEVL53L1X distanceSensor;         // Create distance sensor object
int loop_count = 0;                // Count how many times loop() has looped

double start[2] = { 33.979140, -84.628120 };       // Starting location
double coordinates[2] = {0.0, 0.0};                // Current location

double route[5][2] = {                             // Create the route {lat, long} 
                      { 33.979146, -84.628131 },   // Waypoint[0], My house
                      { 33.978897, -84.628136 },   // Waypoint[1], second house
                      { 33.978568, -84.628125 },   // Waypoint[2], third house
                      { 33.978381, -84.628367 },   // Waypoint[3], fourth house
                      { 33.978376, -84.628678 }    // Waypoint[4], fifth house
                    }; 

int currentHeading = 0;
int distance =       0;

int n =  0;
int ne = 45;
int e =  90;
int se = 135;
int s =  180;
int sw = 225;
int w =  270;
int nw = 315; 


// ----------------------------------- Classes -----------------------------------



// ------------------------------------ Setup ------------------------------------

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
  // Initialize built-in LED on the Arduino for output
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize communications with the RVR
  rvr.configUART(&Serial);               // configure UART
  delay(2 * SECOND);                     // wait for the RVR to "wake up"
  driveControl = rvr.getDriveControl();  // get RVR's DriveControl

  Wire.begin();            // start I2C for the distance sensors
  Serial.begin(115200);    // start Serial at speed 115200 baud

  //Initialize I2C GPS XA1110 module
  while (myI2CGPS.begin() == false)
  {
    Serial.println("GPS failed to respond"); 
    delay(500);
  }
  Serial.println("GPS module found!");

  //Initialize MMC5983MA Micro Magnetometer sensor
  while (myMag.begin() == false)
  {
    Serial.println("Magnetometer did not respond");
    delay(500);
  }
  Serial.println("Magnetometer found!");
  myMag.softReset();  // reset magnetometer for accuracy

  // Initialize distance sensor
  while (distanceSensor.begin() != 0)
  {
    Serial.println("Distance sensor did not respond");
    delay(500);       
  }
  Serial.println("Distance sensor found!");
}


// ---------------------------------- Functions ----------------------------------

/*
   FUNCTION: okayBlink()
   ARGUMENTS: None
   RETURNS: None

   DESCRIPTION:
   Turns an LED on for one second, then off for one second, repeatedly.

   On the UNO the built-in LED is attached to digital pin 13.
   LED_BUILTIN is set to the correct LED pin independent of which board is used.

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
   FUNCTION: getHeading();
   ARGUMENTS: None
   RETURNS: double - heading

   DESCRIPTION:
   get_Heading() reads from the Mini Magnetometer to get the direction the RVR is
   currently facing
*/
int getHeading()
{
  // variables
  uint32_t rawX = 0;
  uint32_t rawY = 0;
  uint32_t rawZ = 0;
  int normX = 0;
  int normY = 0;
  int normZ = 0;
  int heading = 0;

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
  heading = atan2(normX, 0 - normY);

  // Convert heading into degrees
  heading /= PI;
  heading *= 180;
  heading += 180;

  Serial.print("Heading: ");
  Serial.println(heading, 1);
  delay(1000);

  return (heading);
}


/*
 * FUNCTION: getCoordinates()
 * ARGUMENTS: double coordinates[] 
 *    coordinates[0] is latitutde, coordinates[1] is longitute
 * RETURNS: None
 * 
 * DESCRIPTION: 
 * getCoordinates() checks the gps and retrieves latitude and longitute in the 
 * given argument array coordinates[], the first element of coordinates is the 
 * latitude, and the second element is the longitude
 */
 void getCoordinates(double coordinates[]) 
 {
   // Feed the GPS parser
   gps.encode(myI2CGPS.read()); 

   // wait for location to be valid
   while(!gps.location.isValid())
   {
      Serial.println("Waiting on location to validate");
      delay(500);
   }

   // get the latitude 
   coordinates[0] = gps.location.lat();

   // get the longitute
   coordinates[1] = gps.location.lng();
 }


/*
  FUNCTION: checkDist()
  ARGUMENTS: None
  RETURNS: int - 0 on all clear, 1 on blockage

  DESCRIPTION:
  avoid_obstacle() keeps the RVR from running into things
*/
int checkDist()
{
  // variables
  int distance = 0;

  distanceSensor.startRanging();   // initiate measurement

  // wait for info to be ready
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }

  // Get results of measurement
  distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  // If there is something less than 8in/203mm away, the RVR is blocked
  if (distance < 203)
  {
    return (1);
  }
  else return (0);
}


/*
  FUNCTION: avoidObstacle()
  ARGUMENTS: None
  RETURNS: None

  DESCRIPTION:
  avoid_obstacle() keeps the RVR from running into things
*/
void avoidObstacle()
{
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
  // turn to the right 90 degrees
  driveControl.rollStart(90, 10);
  delay(3 * SECOND);
  driveControl.rollStop(90);       // stop driving
  delay(3 * SECOND);

  // turn to the back, another 90 degrees
  driveControl.rollStart(180, 10);
  delay(3 * SECOND);
  driveControl.rollStop(180);      // stop driving
  delay(3 * SECOND);

  // turn to the left, another 90 degrees
  driveControl.rollStart(270, 10);
  delay(3 * SECOND);
  driveControl.rollStop(270);      // stop driving
  delay(3 * SECOND);

  // turn back to "front", another 90 degrees
  driveControl.rollStart(359, 10);
  delay(3 * SECOND);
  driveControl.rollStop(359);      // stop driving
  delay(3 * SECOND);

  // Actually turn back to the front, because 360 is out of range
  driveControl.rollStart(0, 10);
  driveControl.rollStop(0);
}


/*
   FUNCTION: drive()
   ARGUMENTS: None
   RETURN: None

   DESCRIPTION:
   Directs the rover to go a certain direction at a certain speed for a
   certain amount of time.

   Must provide direction and speed.
*/
void drive(int heading)
{
  // drive forward at speed 40 for 10 seconds
  driveControl.rollStart(heading, 40);
  delay(10 * SECOND);
  driveControl.rollStop(heading);  // stop driving
  delay(2 * SECOND);
}


/*
    FUNCTION: subnav()
    ARGUMENTS: None
    RETURN: None
    
    DESCRIPTION:
    subnav() calculates a path to the next waypoint if there is one, 
    if there is not one, it exits
*/
void subnav(double destination[])
{
  // variables
  double curr[2]; 
  
  getCoordinates(curr);

  while (curr[0] != destination[0] && curr[1] != destination[1])
  {
    // current latitude is greater, longitude is greater
    while (curr[0] > destination[0] && curr[1] > destination[1])
    {
      drive(se);
      getCoordinates(curr);
    }

    // current latitude is greater, longitude is lesser
    while (curr[0] > destination[0] && curr[1] < destination[1])
    {
      drive(ne);
      getCoordinates(curr);
    }

    // current latitude is lesser, longitude is greater
    while (curr[0] < destination[0] && curr[1] > destination[1])
    {
      drive(sw);
      getCoordinates(curr);
    }

    // current latitude is lesser, longitude is lesser
    while (curr[0] < destination[0] && curr[1] < destination[1])
    {
      drive(nw);
      getCoordinates(curr);
    }

    // current latitude is greater, longitude is correct
    while (curr[0] > destination[0] && curr[1] == destination[1])
    {
      drive(e);
      getCoordinates(curr);
    }

    // current latitude is lesser, longitude is correct
    while (curr[0] < destination[0] && curr[1] == destination[1])
    {
      drive(w);
      getCoordinates(curr);
    }

    // current latitude is correct, longitude is greater
    while (curr[0] == destination[0] && curr[1] > destination[1])
    {
      drive(s);
      getCoordinates(curr);
    }

    // current latitude is correct, longitude is lesser
    while (curr[0] == destination[0] && curr[1] < destination[1])
    {
      drive(n);
      getCoordinates(curr);
    }
  }
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
  for (int i = 0; i < 5; i++;) 
  {
    subnav(route[i]);  // navigate from where we are to this waypoint
    waypointProcedure();
  }

  subnav(start); // navigate back to the start
  waypointProcedure();
}


/*
    FUNCTION: set_cardinal()
    ARGUMENTS: int currentHeading
    RETURN: None

    DESCRIPTION: 
    Corrects the value of cardinal directions to account for starting
    heading. 

    To obtain the heading offset, subtract the current heading or starting
    heading from 360. Then, to obtain the new value for a cardinal direction, 
    add the offset to it's regular value. If the new value is greater than or
    equal to 360, subtract 360 to get the adjusted new value. 

    The cardinal directions and their regular numeric value are as follows: 
    North (n)       0
    Northeast (ne)  45
    East (e)        90
    Southeast (se)  135
    South (s)       180
    Southwest (sw)  225
    West (w)        270
    Northwest (nw)  315       
*/
void set_cardinal()
{
  // variables
  int currentHeading = getHeading();          // starting direction
  int headingOffset = 360 - currentHeading;   // offset to add to cardinals

  // obtain new cardinal values
  n  = (n  + headingOffset) % 360; 
  ne = (ne + headingOffset) % 360;
  e  = (e  + headingOffset) % 360;
  se = (se + headingOffset) % 360;
  s  = (s  + headingOffset) % 360;
  sw = (sw + headingOffset) % 360;
  w  = (w  + headingOffset) % 360;
  nw = (nw + headingOffset) % 360;

}


/*
    FUNCTION: first()
    ARGUMENTS: None
    RETURN: None

    DESCRIPTION: 
    first() sets up local variables for AARVIN to begin navigating
*/
void first()
{
  // show that all systems are working
  okayBlink();

  // Set the RVR's internal heading to zero
  driveControl.setHeading(0);

  // set starting coordinates
  start[0] = coordinates[0];
  start[1] = coordinates[1];

  // update the values of the cardinal directions to account for starting heading
  set_cardinal();

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
  // get current gps coordinates
  getCoordinates(coordinates);

  // if this is the first loop, setup for navigation
  if (loop_count == 0) 
  {
    first(); 

    // set loop_count to > 0
    loop_count = 1;
  }

  navigate();
}
