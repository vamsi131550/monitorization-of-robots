@Robot Code:

Relevant Files (4):
1. FLRU_BLF1.c is the code corresponding to the robot with ROBOT_ID 1.
2. FLRU_BLF2.c is the code corresponding to the robot with ROBOT_ID 2.
3. lcd.c and lcd.h are included in the above files for lcd interfacing.

NOTE: The two files named FLRU_BLF1.c and FLRU_BLF2.c contain the robot code, which along with lcd.c and/or lcd.h, need to be compiled and the generated hex file must be deployed on the robots. The two files contain the code specific to the two robots used for our demo. To use it on any other robot, ideally, only ROBOT_ID needs to be verified and made unique for each robot. However, the velocity of the robot, its forward movements and white line sensor readings varies from robot to robot, so this portion of the code is not guaranteed to follow the same behavior for each robot. But other portions of the code (packet communication protocol and line follower) guarantees to work on any robot. To make this code work for a robot, the velocities and sensor threshold need to be calibrated for that robot.

@GUI (Central Controller) Code:

Relevant Files (2):
1. BackgroundImageJframe.java contains the code for GUI (Central Controller).
2. GHModel1.png is required to display it in the GUI.

NOTE: The documentation of the source codes i.e. FLRU_BLF1.c, FLRU_BLF2.c and BackgroundImageJframe.java can be found within the file itself. Kindly go through it atleast once before execution.
