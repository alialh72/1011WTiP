#include "vex.h"
#include <cmath>
#include "pid/pid-controller.h"

//========================ARM PID========================

int DrivePID(){

  while(enableDrivePID) {

    //===============================================================
    //--------------Drive PID--------------//

    double currentVelocityLeft = TWLeft.velocity(dps);
    double currentVelocityRight = TWRight.velocity(dps);

    driveLeftVals.updatePIDVals(currentVelocityLeft, false); //pass the left wheel velocity
    driveRightVals.updatePIDVals(currentVelocityRight, false); //pass the right wheel velocity

    double leftWheelPower = driveLeftVals.calculatePower(); 
    double rightWheelPower = driveRightVals.calculatePower(); 
    //===============================================================
        
    LeftDrive.spin(forward, leftWheelPower, voltageUnits::volt);
    LeftDriveUp.spin(forward, leftWheelPower, voltageUnits::volt);

    RightDrive.spin(forward, rightWheelPower, voltageUnits::volt);
    RightDriveUp.spin(forward, rightWheelPower, voltageUnits::volt);

    vex::task::sleep(20);
  }

  return 1;
}