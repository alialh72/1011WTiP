#include "vex.h"
#include <cmath>
#include "pid/pid-controller.h"

//========================PURE PID========================

int DrivePID(){
 /* double kDrive[3] = {0.3, 0.0, 0.0}; //Starting Vals
  limiter lim;
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
  }*/

  return 1;
}

//===================================

int PureDrive() {
  /*while (enablePureDrive) {
    
    double targetL = driveLeftVals.desiredValue; //inches per second
    double targetR = driveRightVals.desiredValue; //inches per second
    double targetDegL = ((targetL/2)*180/M_PI)/(7/5);
    double targetDegR = ((targetR/2)*180/M_PI)/(7/5);

    LeftDrive.spin(forward, targetDegL, velocityUnits::dps);
    LeftDriveUp.spin(forward, targetDegL, velocityUnits::dps);

    RightDrive.spin(forward, targetDegR, velocityUnits::dps);
    RightDriveUp.spin(forward, targetDegR, velocityUnits::dps);

    vex::task::sleep(20);
  }*/
  return 1;
}

//================================================

int driverOnlyPID(){
  driveVals.changePID(kDriveMotor);
  turnVals.changePID(kDriveMotor);

  while(enableDriverPID) {

    //Reset values switch
    if (resetDriveSensors) {
      resetDriveSensors = false;
      InertialLeft.setRotation(0, degrees);
      InertialRight.setRotation(0, degrees);
      LeftDrive.setPosition(0, degrees);
      RightDrive.setPosition(0, degrees);
    }

    //===============================================================
    //------------LATERAL PID------------//

  
    int averagePosition = (LeftDrive.position(degrees) + RightDrive.position(degrees))/2; //averages motor position of the 2 drive

    driveVals.updatePIDVals(averagePosition, false); //passes motor encoder val
    


    double lateralMotorPower = driveVals.calculatePower(); 
    //===============================================================
        


    //===============================================================
    //-------------TURN PID-------------//

    double actualHeading = getInertialReading();
    double turnMotorPower;

    //inertial sensor switch --> inertial must be turned off when using vision
    turnVals.updatePIDVals(actualHeading, false); //passes inertial sensor val
    turnMotorPower = turnVals.calculatePower(); 


    //===============================================================

    LeftDrive.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LeftDriveUp.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightDrive.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    RightDriveUp.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    vex::task::sleep(20);
  }

  return 1;
}