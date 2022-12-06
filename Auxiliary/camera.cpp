
#define CAMERA 3      // camera pin
#define SECOND 1000   // 1ms * 1000 = 1 second

// --------------------------------- Functions ----------------------------------

/*
   FUNCTION: okayBlink()
   ARGUMENTS: None
   RETURNS: None

   DESCRIPTION:
   Turns an LED on for one second, then off for one second, repeatedly.

   On the UNO the built-in LED is attached to digital pin 13.
   LED_BUILTIN is set to the correct LED pin independent of which board 
   is used.

   SOURCES:
   https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
void okayBlink()
{
  for (uint8_t i = 0; i < 10; i++)
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
  delay (0.5 * SECOND);
  
  // Take a picture with the spy camera
  digitalWrite(CAMERA, LOW);     // press trigger to take a picture
  delay(40);
  digitalWrite(CAMERA, HIGH);    // release the trigger

  delay(0.5 * SECOND);
}


// ------------------------------------ Setup ------------------------------------

void setup() {
  // setup pins for output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CAMERA, OUTPUT);

  // Initialize camera module
  digitalWrite(CAMERA, HIGH);

}


// ------------------------------------ Main -------------------------------------

void loop() 
{
  delay(2*SECOND);
  takePicture();
  delay(2*SECOND);

}
