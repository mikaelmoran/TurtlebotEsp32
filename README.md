TurtleRobot –  Hardware, Calibration, and Firmware Guide
 
This guide will help you assemble the TurtleRobot project (including connecting the motors, drivers, servo, and ESP32-C3 Mini), flash the firmware onto your ESP32, and perform the necessary calibrations via the web interface. 

**** I tried it on my floor, with a pen that goes away with water *** :-) 

 
Part 1: Hardware Wiring
Components You Need:
2 x 28BYJ-48 Stepper Motors Each motor is controlled via its own ULN2003 driver board.
1 x Hobby Servo
1 x ESP32-C3 Mini with Developer Board
1 x O-Ring (approximately 60 mm) – Used to improve wheel traction.
6 x m3 with nuts
Jumper Wires female-female (for easy connections)
Little superglue, to glue the 2 standoff's on the underside of the robot. 
Power Supply  i used a powerpack 

A USB Cable (for programming the ESP32) Power Connections 5V Supply:

Connect the 5V output from the Developer Board to the VCC pin on each ULN2003 driver board.
Connect the servo’s red (power) cable to 5V. Ground (GND): Connect the Developer Board GND to the GND
pins on both ULN2003 driver boards and the servo’s black cable. 
*** Ensure that all grounds are common.
 
Assembly:
Mount the Motors and Servo: Secure each 28BYJ-48 motor on the left and right side and the drivers. 
Mount the servo in its designated holder. Attach the ESP32-C3 Mini to its mounting bracket.
 
Wiring Diagram Motors (via ULN2003 Driver Boards)
Left Wheel: ULN2003 Input ESP32‑C3 GPIO Pin:
IN1 GPIO 1
IN2 GPIO 2
IN3 GPIO 3
IN4 GPIO 4
 
Right Wheel: ULN2003 Input ESP32‑C3 GPIO Pin:
IN1 GPIO 8 
IN2 GPIO 7 
IN3 GPIO 6 
IN4 GPIO 5
 
Servo Signal: Connect the servo’s signal wire (yellow/orange) to GPIO 10 on the ESP32‑C3.
Power: Connect the servo’s red cable to 5V. Ground: Connect the servo’s black cable to GND.
 
Part 2: Flashing the Firmware To flash the firmware.bin file onto your ESP32 (ESP32‑C3 Mini).
 
Download flasher tool, compile it with latest Arduino IDE.
Download firmware.bin

Download it from here : https://github.com/mikaelmoran/TurtlebotEsp32/


After compiling and flashing the flasher tool, then connect to :
ESP32-C3-TURTLE-AP-FLASHER
AP-Password : 12345678
 
Open http://192.168.4.2
Select the downloaded firmware file, and Upload Firmware.

The device will reset with the turtle firmware on it.

 
 
Part 3: Calibration via Web Interface (First Step After Flashing)
 
Before using the robot, you must calibrate it using the two calibration functions on the web interface.
This is the very first thing you should do. Follow these steps:
 
Connect to the Robot’s WiFi After flashing the firmware, the ESP32 will start a WiFi Access Point (e.g., TurtleRobot-AP).
Connect your computer or mobile device to this network.
 
Open the Web Interface Open your web browser and navigate to the robot’s web page.
You can use http://turtlebot.local; otherwise, use the IP address displayed by the ESP32 via the serial port.
 
Run the Calibration Tests The web interface includes two calibration functions:
a) Distance Calibration Click the button (e.g., "Drive (200 mm) forward") to run a distance test.
The robot will attempt to drive 200 mm. Measure the actual distance driven.
Enter the measured distance (in mm) into the provided field and click "Save Distance Calibration." Repeat until the calibration is accurate.
 
 
b) Rotation Calibration Click the button (e.g., "Run Rotation Test (360°)") to perform a 360° rotation test. Measure the actual rotation angle achieved by the robot. Enter the measured angle (in degrees) into the provided field and click "Save Rotation Calibration." Repeat this test until the rotation is accurate. Note: Calibrating both the distance and rotation is essential for the robot to accurately reproduce your drawings.


If you like it, support me if you want, with a coffee : -) https://buymeacoffee.com/mikaelmoran
![Skärmavbild 2025-03-29 kl  20 53 45](https://github.com/user-attachments/assets/d4c4d2f7-8d5c-4dee-a8cb-1bc66fa5a35a)
![IMG_0823](https://github.com/user-attachments/assets/08bcc28e-27ff-4223-a596-96def02cb8bb)
![IMG_0821](https://github.com/user-attachments/assets/29b3ff97-bd46-411f-8745-2b96f5c80bef)
![IMG_0819](https://github.com/user-attachments/assets/f8f78208-8720-45fa-8d9f-de358b9fdaeb)
![IMG_0816](https://github.com/user-attachments/assets/ac6e6bc4-15e6-404f-8bda-03d8914759ff)
