#include "vex.h"
#include <cmath>
#include "pid/pid-controller.h"

//========================PURE PID========================

int DrivePID(){
  double kDrive[3] = {0.3, 0.0, 0.0}; //Starting Vals
  driveLeftVals.changePID(kDrive);
  driveRightVals.changePID(kDrive);

  while(enableDrivePID) {

    //===============================================================
    //--------------Drive PID--------------//

    double currentVelocityRight = (RightDrive.velocity(dps) * M_PI/180 * 4/2 * 7/5);  //wheel diameter/2
    double currentVelocityLeft = (LeftDrive.velocity(dps) * M_PI/180 * 4/2 * 7/5);  //wheel diameter/2

    driveLeftVals.updatePIDVals(currentVelocityLeft, false); //pass the left wheel velocity
    driveRightVals.updatePIDVals(currentVelocityRight, false); //pass the right wheel velocity

    double leftWheelPower = driveLeftVals.calculatePower(); 
    double rightWheelPower = driveRightVals.calculatePower(); 

    double rChangeL = driveLeftVals.desiredValue;
    double rChangeR = driveRightVals.desiredValue;
    leftWheelPower = lim.rateLimiter(leftWheelPower, rChangeL);
    rightWheelPower = lim.rateLimiter(rightWheelPower, rChangeR);

    leftWheelPower = clamp(leftWheelPower, -12, 12);
    rightWheelPower = clamp(rightWheelPower, -12, 12);

    //===============================================================

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("mVLR: %f, %f", currentVelocityLeft, currentVelocityRight);
    Controller1.Screen.newLine();
    Controller1.Screen.print("volts: %f, %f", leftWheelPower, rightWheelPower);
    Controller1.Screen.newLine();
    Controller1.Screen.print("error: %f, %f", driveLeftVals.error, driveRightVals.error);
    Controller1.Screen.newLine();

        
    LeftDrive.spin(forward, leftWheelPower, voltageUnits::volt);
    LeftDriveUp.spin(forward, leftWheelPower, voltageUnits::volt);

    RightDrive.spin(forward, rightWheelPower, voltageUnits::volt);
    RightDriveUp.spin(forward, rightWheelPower, voltageUnits::volt);

    vex::task::sleep(20);
  }

  return 1;
}


