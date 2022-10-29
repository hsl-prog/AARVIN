/*
 * Autonomous Arduino-based Research Vehicle In Nature (AARVIN) 
 * Hayley Spencer Leavitt 2022
 * 
 * Version 1.2
 * 
 * Built for Sphero RVR and Arduino Uno R3 with Arducam Arduino camera module, 
 * Sparkfun Qwiic GPS, Distance Sensors, Mux Breakout, Qwiic Arduino Shield.
 * 
 * https://www.arduino.cc/reference/en/
 * https://github.com/sphero-inc/sphero-sdk-arduino-cpp
 * 
 */

// --------------------------------- Libriaries ----------------------------------

#include <SpheroRVR.h>                          // Library for the Sphero RVR
#include <SPI.h>                                // SPI bus library
#include <SD.h>                                 // Library for use of SD card
#include <Wire.h>                               // Library for I2C/TWI devices
#include <SparkFun_I2C_GPS_Arduino_Library.h>   // Library for GPS XA1110


// ----------------------------------- Macros ------------------------------------

#define SPEEDMULTIPLIER 1  // 1 = 100%, which is full speed 
#define SECOND 1000        // 1000 = 1 second


// ------------------------------ Global Variables -------------------------------

static DriveControl driveControl;  // Drive control for the RVR
const int chipSelect = 4;          // chip pin for the SD card
static I2CGPS myI2CGPS;            //Hook GPS to the library


// ---------------------------------- Functions ----------------------------------

// the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  // configure UART port for Serial for use with RVR
  rvr.configUART(&Serial);

  // get the RVR's DriveControl
  driveControl = rvr.getDriveControl();

  // Mux break out: 
  //    - Arduino is connected to main
  //    - GPS is on port 0 (address 0x70)
  //    - Front distance sensor is on port 1 (address 0x71)
  //    - Back distance sensor is on port 4 (address 0x74)
  //    - Compass sensor *will* go on port 3 (address 0x73)
  //    - Front limit switch sensor *will* go on port 2 (address 0x72)
  //    - Back limit switch sensor *will* go on port 5 (address 0x75)
  //    - port 6 (address 0x76), port 7 (address 0x77), and port 8 (address 0x78) remain

  // set GPS update rate to 10Hz

  // set the SD Serial port
  Serial.begin(9600);
}


/*
 * verify_sensors()
 * 
 * 
 */
void verify_sensors()
{
}


/*
 * blink()
 *
 * Turns an LED on for one second, then off for one second, repeatedly.
 *
 * On the UNO the built-in LED is attached to digital pin 13. 
 * LED_BUILTIN is set to the correct LED pin independent of which board is used.
 *
 * https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
void okay_blink() 
{
  for (int i=0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH v)
    delay(0.5 * SECOND);                       // wait for 1/2 of a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW v)
    delay(0.5 * SECOND);                       // wait for 1/2 of a second
  }
}


/* 
 *  get_gps_location()  
 *  
 *  Finds the current location of AARVIN using the Sparkfun XA1110 module. 
 */
void get_gps_location()
{
}


/*
 * take_picture()
 * 
 * Captures an image with the camera and saves it to the micro sd card
 */
 void take_picture()
 {
 }


/* 
 *  calculate_subroute()  
 *  
 *  calculates a path to our next waypoint if there is one 
 */
void calculate_subroute()
{
}


/*
 * drive()
 * 
 * Directs the rover to go a certain direction at a certain speed for a 
 * certain amount of time.
 * 
 * Must provide direction and speed.
 */
void drive(int heading)
{  
  // drive forward at speed 10 for 5 seconds
  driveControl.rollStart(heading, 10);
  delay(8 * SECOND);
  driveControl.rollStop(heading);  // stop driving
  delay(SECOND);
}


/*
 * waypoint_procedure()
 * 
 * Directs the rover to turn 90 degrees each 
 */
void waypoint_procedure()
{  
  driveControl.rollStart(90, 10);  // turn to the right 90 degrees
  delay(3 * SECOND); 
  driveControl.rollStop(90);       // stop driving
  delay(3 * SECOND);
  driveControl.rollStart(180, 10); // turn to the reverse, another 90 degrees
  delay(3 * SECOND); 
  driveControl.rollStop(180);      // stop driving
  delay(3 * SECOND);
  driveControl.rollStart(270, 10); // turn to the left, another 90 degrees
  delay(3 * SECOND);
  driveControl.rollStop(270);      // stop driving
  delay(3 * SECOND);
  driveControl.rollStart(359, 10); // turn back to "start", another 90 degrees
  delay(3 * SECOND);
  driveControl.rollStop(359);      // stop driving
  delay(3 * SECOND);
  driveControl.rollStart(0, 10);   // Actually turn back to start
  driveControl.rollStop(0);        // because 360 is out of range
}


/* 
 *  navigate()  
 *  
 *  directs the rover to navigate along the designated sub-route
 */
void navigate()
{

}


// avoid_obstacle() function: keeps us from running into things
void avoid_obstacle()
{
  
}


// shut_down function: disconnects, cleans up and closes out
void shut_down()
{
  
}


// ---------------------------------- Main Code ----------------------------------

// loop() is the main function and it runs repeatedly
void loop()
{
  // show that all systems are working
  okay_blink();          

  // reset the heading to zero
  driveControl.setHeading(0);

  waypoint_procedure();

  drive(0); // drive with heading 0, aka forward

  waypoint_procedure();

  // exit(0);  // end loop()

}
