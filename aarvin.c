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
 * https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library
 * 
 */

// --------------------------------- Libriaries ----------------------------------

#include <SpheroRVR.h>                          // Library for the Sphero RVR
#include <SPI.h>                                // SPI bus library
#include <Wire.h>                               // Library for I2C/TWI devices

#include <SparkFun_I2C_Mux_Arduino_Library.h>   // Library for I2C Qwiix Mux ??

#include <SparkFun_I2C_GPS_Arduino_Library.h>   // Library for GPS XA1110
#include <SparkFun_VL53L1X.h>                   // Library for distance sensor


// ----------------------------------- Macros ------------------------------------

#define SPEEDMULTIPLIER 1      // 1 = 100%, which is full speed 
#define SECOND 1000            // 1000 = 1 second
#define NUMBER_OF_SENSORS 4    // how many sensors are we putting on the mux

#define MUX_GPS     0x70       // define address for mux port 0, GPS
#define MUX_FRONT   0x71       // define address for mux port 1, front dist.
#define MUX_COMPASS 0x72       // define address for mux port 2, compass
#define MUX_BACK    0x74       // define address for mux port 4, back dist. 


// ------------------------------ Global Variables -------------------------------

static DriveControl driveControl;  // Drive control for the RVR
const int chipSelect = 4;          // chip pin for the SD card
I2CGPS myI2CGPS;                   // Hook GPS object to the library


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

  /* 
   *  Initialize all sensors connected to the TCA9548A 8-channel Mux board
   *  
   * The Mux board has two "main" ports, and 8 ports that sensors can use.
   * The ports, numbered 0-7, have Serial addressed of 0x70 through 0x77.
   * The Arduino Uno is connected to the main port on the left side of the bot.
   * 
   * The GPS XA1110 module is connected on port 0, address 0x70.  
   * The front distance sensor, which is a VL53L1X module, is connected on port 
   * 1, with address 0x71. 
   * The compass module, which is a MMC5983MA micro magnetometer sensor, is 
   * connected on port 2, with address 0x72. 
   * The back distance sensor, which is also a VL53L1X module, is connected on 
   * port 5, with address 0x75. 
   * 
  */
  Serial.begin(9600);
  Serial.println("Qwiic Mux Shield Read Example");

  Wire.begin();

  //Initialize all the sensors
  for (byte x = 0 ; x < NUMBER_OF_SENSORS ; x++)
  {
    enableMuxPort(x);   //Tell mux to connect to port X
    //Initialize the sensor connected to this port
    disableMuxPort(x);
  }

  Serial.println("Mux Shield online");
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


// ---------------------------------- Main Code ----------------------------------

// loop() is the main function and it runs repeatedly
void loop()
{
  for (byte x = 0 ; x < NUMBER_OF_SENSORS ; x++)
  {
    enableMuxPort(x); //Tell mux to connect to this port, and this port only

    // connect to the sensor and take a reading

    disableMuxPort(x); //Tell mux to disconnect from this port
  }
  
  // show that all systems are working
  okay_blink();          

  // reset the heading to zero
  driveControl.setHeading(0);

  waypoint_procedure();

  drive(0); // drive with heading 0, aka forward

  waypoint_procedure();

  // exit(0);  // end loop()

}
