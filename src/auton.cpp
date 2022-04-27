#include "vex.h"
#include <cmath>
#include "auton.h"



/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void testAuto()  { //1
  //===Motion 1====
  //----Drive------
  vex::task driverPid(DriverPID);
  resetDriveSensors = true;
  enableInertial = false;
  driverVals.desiredValue = 2670;  
  turnVals.desiredValue = 0;

  FrontClamp.set(true);

  vex::task::sleep(1250);


  //===Motion 2====
  //----Drive------
  resetDriveSensors = true;
  driverVals.desiredValue = 0;  
  turnVals.desiredValue = 0;

  FrontClamp.set(false);

  vex::task::sleep(150);

 //===Motion 3====
  //----Drive------
  resetDriveSensors = true;
  driverVals.desiredValue = -2000;  
  turnVals.desiredValue = 0;
  
  vex::task::sleep(350);

}

int brainPrint() {
  while(1) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(finalPath.points.size());
    Brain.Screen.newLine();
    Brain.Screen.print("currentcoords: %f, %f", finalPosition.x, finalPosition.y);
    Brain.Screen.newLine();
    Brain.Screen.print("point vel: %f", closestPoint.targetVelocity);
    Brain.Screen.newLine();
    Brain.Screen.print("target vel: %f", targetVel);
    Brain.Screen.newLine();
    Brain.Screen.print("signed curvature: %f", signedCurvature);
    Brain.Screen.newLine();
    Brain.Screen.print("TWL & TWR: %f, %f", targetLW, targetRW);
    Brain.Screen.newLine();
    Brain.Screen.print("getinertial: %f", getInertialReading());
    Brain.Screen.newLine();
    Brain.Screen.print("closestpoint: %f, %f", closestPoint.x, closestPoint.y);
    Brain.Screen.newLine();
    Brain.Screen.print("lookahead: %f, %f", lookaheadPoint.x, lookaheadPoint.y);
    wait(20, msec);
  }
  return 1;
}

void skillsAuto()  { //2


    //Run all threads
  enableFollowPath = true;
  enableOdom = true;
  enableDrivePID = true;

  initOdom();
  setPathMotion(0);
 // setPathMotion(0);
  vex::task runOdom(RunOdom);
  vex::task followPath(FollowPath);
  vex::task brainprint(brainPrint);
  vex::task drivePID(DrivePID);




}



void tunePID()  { //2
  // enableDrivePID = true;
  // vex::task drivePID(DrivePID);
  // driveLeftVals.desiredValue = 20;
  // driveRightVals.desiredValue = 10;
  while(1) {
    double currentVelocityRight = (RightDrive.velocity(dps) * M_PI/180 * 4/2 * 7/5);  //wheel diameter/2
    double currentVelocityLeft = (LeftDrive.velocity(dps) * M_PI/180 * 4/2 * 7/5);  //wheel diameter/2

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("cV: %f, %f", currentVelocityLeft, currentVelocityRight);

    double maxVel = 41.5;
    double isperv = maxVel/12;

    double desiredVel = 10;

    LeftDrive.spin(forward, desiredVel / isperv, voltageUnits::volt);
    LeftDriveUp.spin(forward, desiredVel / isperv, voltageUnits::volt);

    RightDrive.spin(forward, (desiredVel / isperv), voltageUnits::volt);
    RightDriveUp.spin(forward, (desiredVel / isperv), voltageUnits::volt);

    vex::task::sleep(20);
  }

}