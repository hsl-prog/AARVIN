Hayley Leavitt
Autonomous Arduino-Based Research Vehicle In Nature (AARVIN) 
AARVIN Version 3.0

INSTALLATION GUIDE


Required Software 
Arduino IDE 
https://www.arduino.cc/en/software
Libraries: 
Wire.h
SparkFun_I2C_GPS_Arduino_Library.h
TinyGPS++.h
SparkFun_VL53L1X.h
SparkFun_MMC5983MA_Arduino_Library.h
Waypoint.h
AARVIN code: 
Main code
https://github.com/hsl-prog/AARVIN/blob/main/aarvin.ino
Waypoint.h
https://github.com/hsl-prog/AARVIN/blob/main/Waypoint.h 
Waypoint.cpp
https://github.com/hsl-prog/AARVIN/blob/main/Waypoint.cpp 
 


Required Hardware
  Parts and Sensors:
    Sphero RVR or Sphero RVR++
      https://sphero.com/products/rvr 
    Arduino Uno
      https://store-usa.arduino.cc/products/arduino-uno-rev3?selectedStore=us 
    SparkFun Qwiic Shield for Arduino
      https://www.sparkfun.com/products/14352 
    SparkFun XA1110 Qwiic GPS Module
      https://www.sparkfun.com/products/14414 
    SparkFun VL35L1X Qwiic Distance Sensor Breakout
      https://www.sparkfun.com/products/14722 
    SparkFun MMC5983MA Micro Magnetometer Module (Qwiic)
      https://www.sparkfun.com/products/19921
    Adafruit Mini Spy Camera 
      https://www.adafruit.com/product/3202
    2 short Qwiic cables
      https://www.sparkfun.com/products/14426 
    1 long Qwiic cable
      https://www.sparkfun.com/products/14429 
    3 male-to-male jumper wires in black, red, and green (1 each)
      https://www.sparkfun.com/products/12795 
    1 6-12” right angle USB A - USB B Cable
      https://amazon.com/Inovat-Printer-Cable-USB-Scanner/dp/B01HB91CRM/ref=sr_1_9?crid=EMX40RHSG4EV&keywords=right+angle+usb+a+to+b&qid=1670371445&sprefix=right+angle+usb+a+to+b+%2Caps%2C138&sr=8-9 
Miscellaneous: 
  Various mounting hardware including screws, brackets, nuts, etc.
  Expansion mounting plate 
  Electrical tape
  Twist ties 
  8 - 32 GB microSD card
  microSD to SD card adapter
Tools: 
  Soldering iron, flux, and solder
  Wire strippers 
  SD card reader



To Install Arduino IDE 
AARVIN is an Arduino based project, so to use AARVIN, you’ll need to use the Arduino IDE. This is a free, open-source IDE built specifically for use with Arduinos. For the sake of ease and simplicity, this guide will use Arduino IDE 1.8.19.
These are the requirement specs for the Arduino IDE: 
Windows - Win 10 and newer, 64 bits
Linux - 64 bits
Mac OS X - Version 10.14: "Mojave" or newer, 64 bits
To download Arduino’s Arduino IDE, go to their website, www.arduino.cc/en/software,  and scroll down to “Legacy IDE (1.8.X).” From there, download the 

For Windows, download the Windows file and follow the installation guide. The default options should suffice. 
 
For MacOS, download the MacOS file, open the downloaded file, and wait for the installation window to pop up. Then, drag the software to your applications folder. 
 
For Linux, download the AppImage 64 bits file, and turn it into an executable file. To do this, right click on the file, choose properties, select the permissions tab, and check the box next to “Allow executing file as a program.”
If you have any difficulties or questions with this process, see Arduino’s Getting Started Guide or their Installation Guide for IDE 1. They also have an overview of their IDE.
Getting Started Guide: https://www.arduino.cc/en/Guide
Installation Guide: 
Linux: https://docs.arduino.cc/software/ide-v1/tutorials/Linux
Windows: https://docs.arduino.cc/software/ide-v1/tutorials/Windows
MacOS: https://docs.arduino.cc/software/ide-v1/tutorials/macOS
Overview of IDE 1: https://docs.arduino.cc/software/ide-v1/tutorials/Environment 
Once you have installed the IDE, open it. There may be some updates you need to install. Go ahead and install those, and restart the IDE. 
 


Install Required Libraries
The Wire.h library is an Arduino library, so there is no need to install it. The following libraries can be installed from the Arduino IDE: 
SparkFun I2C GPS Arduino Library
TinyGPSPlus by Mikhal Hart
SparkFun VL53L1X Library
SparkFun MMC5983MA Arduino Library
Installing most of these libraries should be quick and easy. Simply go to the “Tools” tab, and select “Manage Libraries…”. 

The Library Manager will then pop up. Type in the library name, such as “TinyGPS” and hit enter. The Library Manager will then show you a list of related libraries. Locate the appropriate library and click “Install.” Repeat for each library except Wire and Waypoint.

To include each of these libraries in a sketch, you can click “Sketch” and select “Include Library”. This will show you a list of your installed libraries. Select one and it will automatically insert the library into your sketch. There is no need for this with AARVIN, however, as all of the include statements are already in the file.  



Download AARVIN Code
To download the code for AARVIN, go to https://github.com/hsl-prog/AARVIN and take a look at all of the files. 

The “Auxiliary” file contains modularized versions of each function AARVIN performs. These are written so that each function is functional on its own. 
The “nowDefunct” file contains an old, but still functional version of AARVIN that solely used cardinal directions. 
The current files for AARVIN are listed at the bottom: Waypoint.cpp, Waypoint.h, and aarvin.ino. A .ino file is an Arduino file, but it functions the same way as a C++ file.
To create a local version of AARVIN on your computer, open the Arduino IDE and start a new sketch. Copy and paste the aarvin.ino file into the sketch and save it as “aarvin”. Then, click the drop down triangle at the top right to create a new tab, or type Shift+CTRL+N on Windows or Shift+Command+N on Mac.

Name the new tab Waypoint.h and paste the code from Waypoint.h on the github into the tab’s workspace. 
Repeat this process again, creating a new tab and naming it Waypoint.cpp and paste the code from Waypoint.cpp on the github into this new tab’s workspace. 
Your finished product should look like this: 

To verify the code, click the checkmark in the top left of the IDE. 

The terminal tab at the bottom will say “Done compiling” if everything compiled correctly. If there were any errors, inside that terminal tab will be red text with a description of the error. 
AARVIN is a large code, especially for the Uno. At the bottom of the terminal tab there will likely be a red message that says “Low memory available, stability problems may occur.” This is expected for this iteration of AARVIN and is not a code-breaking error. 



Hardware Assembly 
The assembly for AARVIN is fairly straightforward. First, gather all of your components (see page 3). Then, prepare your components.
Qwiic Shield
The Qwiic shield will arrive looking like this: 

You will need to solder the pins to the Qwiic Shield. To do so, place the pins through the board to figure out their placement. Then, set the shield down so it is resting on the black parts of the pins. The Qwiic connectors should be facing down at this point. Lightly brush flux onto the pins and, with a hot soldering iron, place a bead of solder onto each pin. Allow to cool. It should look like the below board now. 

Spy Camera
The spy camera will arrive with a connector on the end of its 3 wires. Cut that connector off and strip the loose end of each wire. 
Next, take your 3 male-to-male jumper cables and cut one pin off, and strip the loose end of your wire. 
Solder the loose end of your jumper wire to the loose end of the spy camera wire, green to green, black to black, and red to red. Then, wrap the soldered connection with electrical tape.

Assemble Components
Place the Qwiic Shield on top of your Arduino Uno. Each pin of the shield should line up with each pin on the Uno. Be gentle pressing them together, as it may be a tight fit. 
Then, using the developer plate from the RVR, attach your gps and your magnetometer to the plate. Use an expansion board to add a second level raised above the developer plate if you need more space. The expansion plate in the photos came with the Advanced Autonomous Kit for Sphero RVR from SparkFun, here: https://www.sparkfun.com/products/15303 

Mount the Arduino to the expansion plate so that the Arduino’s USB port is on the same side as the RVR’s USB port. 
Connect the camera module to a popsicle stick, piece of plastic, L-bracket, or some other mount using tape or glue, and attach it to the RVR plate.
Then, using L-brackets, mount the distance sensor so that it is perpendicular to the expansion plate. This is important so that the distance sensor will be able to see in front of AARVIN. Make sure to mount the distance sensor so it is facing “front”, which will be on the same end as the RVR’s USB port. Facing the distance sensor, the RVR’s USB port should be on the right. 

Next, wire the components together. Using the 2 short Qwiic cables, connect the GPs to the distance sensor and connect the distance sensor to the magnetometer. The magnetometer only has one Qwiic port, so it should be at the end of the daisy chain. 
Use the long Qwiic cable to connect the GPS to the Qwiic Shield, weave it underneath the board to keep it out of the way of the camera and the distance sensor. It does not matter which Qwiic port on the shield is used, they are all equal. 
If any parts need to be stabilized or if any cords need to be pulled to the side, use zip ties or a twist tie to do so. 
 
Lastly, connect the camera wires to the pins in the Qwiic shield, which directly connect to the Arduino pins. The green wire should go to digital I/O pin 9, black should go to ground GND, and red should go to 5V. See photo below. 
 


Uploading the Code
To run the code, plug the USB A to B cable into the Arduino and plug it into your computer. 

Open aarvin.ino in the Arduino IDE. Then, select the Arduino board used on the project, which is the Uno. 
Go to the menu bar and select “Tools”, then select “Board:”, and then“Boards Manager…”. This will open up a pop up window that is very similar to the Library Manager. 
 
Search for “Arduino AVR Boards” and click “Install”.

The Uno board is now installed, and should be selected as the board being used. Go to “Tools”, select “Board:”, then select “Arduino AVR Boards”, and lastly, select “Arduino Uno.” 

Then, in “Tools” again, select “Port: ” and select the USB port. This will connect the Arduino IDE with the Arduino that is plugged in.

Now, at the top left of the IDE, next to the Verify checkmark, click the arrow that is pointing to the right to upload the code to the Arduino. 

The IDE will show “Compiling….” and “Uploading…”, when it is done uploading it will say so. The Arduino can now be safely unplugged. 



Running AARVIN
Take the developer plate with the now-programmed Arduino and snap it back onto the RVR. 

Take the Arduino’s USB cord and plug it into the RVR’s USB port. AARVIN is now ready to run! 
On a clear, sunny day, take it outdoors and turn it on. The GPS will need a clear view of the sky, free from any obstruction such as trees or clouds in order to function properly. Be careful not to run AARVIN outside when it is wet as neither the Arduino circuitry nor the RVR are waterproof. 
AARVIN will follow its route autonomously and return home when it is finished. 
