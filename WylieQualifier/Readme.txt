The biggest change between this codebase and refactoring is that many
functions in RobotDrive (especially those that perform corrections) 
stopped working properly.

Gamepad up and down was reversed so when up pressed the robot would 
moved back and when down was pressed the robot would move forward.
To fix this, Arpan made some code changes which broke all the correction
performing functions in RobotDrive.

Code in this directory have been fixed. So both gamepad direction is 
correct and also corrections performing functions in RobotDrive is now
good.
