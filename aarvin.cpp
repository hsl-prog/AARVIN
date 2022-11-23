
/*
   Autonomous Arduino-based Research Vehicle In Nature (AARVIN)
   Hayley Spencer Leavitt 2022

   Version 2.2

   Built for Sphero RVR, Arduino Due, and Arduino Uno with Arducam Arduino camera 
   module, Sparkfun Qwiic GPS, Mini Magnetometer, and Distance Sensor. 

   https://www.arduino.cc/reference/en/
   https://github.com/sphero-inc/sphero-sdk-arduino-cpp
   https://github.com/mikalhart/TinyGPSPlus   

*/

// --------------------------------- Libriaries ----------------------------------

#include <Wire.h>                                // I2C/TWI Lib
#include <SparkFun_I2C_GPS_Arduino_Library.h>    // GPS XA1110 Lib
#include <TinyGPS++.h>                           // TinyGPS Lib
#include <SparkFun_VL53L1X.h>                    // Distance Sensor Lib
#include <SparkFun_MMC5983MA_Arduino_Library.h>  // Micro Magenetometer Lib


// ----------------------------------- Macros ------------------------------------

#define SPEEDMULTIPLIER 1        // 1 = 100%, which is full speed 
#define SECOND 1000              // 1000 = 1 second


// ------------------------------ Global Variables -------------------------------

I2CGPS myI2CGPS;                   // Create I2C GPS object
TinyGPSPlus gps;                   // Create TinyGPS gps object
SFE_MMC5983MA myMag;               // Create Magnetometer object
SFEVL53L1X distanceSensor;         // Create distance sensor object
int drive_device_addr = 9;         // What address is the drive device on
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
   FUNCTION: sendWire()
   ARGUMENTS: int wHeading, int wSpeed, int wTime
   RETURNS: None
   
   DESCRIPTION: 
   sendWire() sends an array of wHeading, wSpeed, and wTime to the drive device. 
   Arduino needs to multiply an integer by 1000 to get the amount of time in 
   seconds, so the slave device will receive wTime as "how many seconds" and will
   multiply wTime by 1000 when using it. 
   
   SOURCES: 
   https://www.instructables.com/I2C-between-Arduinos/
   GolamMostafa on Arduino forum: 
   https://forum.arduino.cc/t/solved-sending-array-data-with-i2c/537012/2
 */
 void sendWire(int wHeading, int wSpeed, int wTime)
 {
    Wire.beginTransmission(drive_device_addr); // transmit to drive device
    Wire.write(wHeading);                      // transmit heading info
    Wire.write(wSpeed);                        // transmit speed info
    Wire.write(wTime);                         // transmit time info
    Wire.endTransmission();                    // stop transmitting
    delay(wTime + (5 * SECOND));               // wait for the RVR to drive
 }


/*
   FUNCTION: getHeading();
   ARGUMENTS: None
   RETURNS: double - heading

   DESCRIPTION:
   get_Heading() reads from the Mini Magnetometer to get the direction the RVR is
   currently facing

   SOURCES: 
   Magnetometer Example Code
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
   FUNCTION: getCoordinates()
   ARGUMENTS: double coordinates[] 
      coordinates[0] is latitutde, coordinates[1] is longitute
   RETURNS: None
   
   DESCRIPTION: 
   getCoordinates() checks the gps and retrieves latitude and longitute in the 
   given argument array coordinates[], the first element of coordinates is the 
   latitude, and the second element is the longitude
   
   SOURCES: 
   GPS example code
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
  checkDist() checks to see how far away any obstacles are. Returns 0 if path is 
  clear, and 1 if the path is blocked. 
  
  SOURCES: 
  Distance Sensor example code
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
  avoid_obstacle() keeps the RVR from running into things if we are blocked
*/
void avoidObstacle()
{
  int currH = getHeading(); 

  sendWire(currH + 30, 10, 0.5);

  if(checkDist() == 0)
  {
    // drive in the unblocked direction
    sendWire(currH + 30, 50, 5); 
  }
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
  int currH = getHeading();
  
  // turn East
  sendWire(e, 10, 3);

  // turn South
  sendWire(s, 10, 3);

  // turn West
  sendWire(w, 10, 3);

  // turn North
  sendWire(n, 10, 3);

  // Turn back to the way we were facing
  sendWire(currH, 10, 1);
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
      sendWire(se, 40, 10);
      getCoordinates(curr);
      
      while(checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is greater, longitude is lesser
    while (curr[0] > destination[0] && curr[1] < destination[1])
    {
      sendWire(ne, 40, 10);
      getCoordinates(curr);

      while(checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is lesser, longitude is greater
    while (curr[0] < destination[0] && curr[1] > destination[1])
    {
      sendWire(sw, 40, 10);
      getCoordinates(curr);

      while(checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is lesser, longitude is lesser
    while (curr[0] < destination[0] && curr[1] < destination[1])
    {
      sendWire(nw, 40, 10);
      getCoordinates(curr);

      while(checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is greater, longitude is correct
    while (curr[0] > destination[0] && curr[1] == destination[1])
    {
      sendWire(e, 40, 10);
      getCoordinates(curr);

      while(checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is lesser, longitude is correct
    while (curr[0] < destination[0] && curr[1] == destination[1])
    {
      sendWire(w, 40, 10);
      getCoordinates(curr);

      while(checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is correct, longitude is greater
    while (curr[0] == destination[0] && curr[1] > destination[1])
    {
      sendWire(s, 40, 10);
      getCoordinates(curr);

      while(checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is correct, longitude is lesser
    while (curr[0] == destination[0] && curr[1] < destination[1])
    {
      sendWire(n, 40, 10);
      getCoordinates(curr);

      while(checkDist() == 1)
      {
        avoidObstacle();
      }
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
  for (int i = 0; i < 5; i++) 
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

  // run blink to show that systems are working
  okayBlink();

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

  navigate();
}
