#include "vex.h"
#include <cmath>
#include "auton.h"



/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/


int brainPrint() {
  while(1) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(finalPath.points.size());
    Brain.Screen.newLine();
    Brain.Screen.print("currentcoords: %f, %f", finalPosition.x, finalPosition.y);
    Brain.Screen.newLine();
    Brain.Screen.print("target vel: %f", targetVel);
    Brain.Screen.newLine();
    Brain.Screen.print("signed curvature: %f", signedCurvature);
    Brain.Screen.newLine();
    Brain.Screen.print("TWL & TWR: %f, %f", targetLW, targetRW);
    Brain.Screen.newLine();
    Brain.Screen.print("getinertial: %f", getInertialReading());
    Brain.Screen.newLine();
    Brain.Screen.print("start: %f, %f", closestPoint.x, closestPoint.y);
    Brain.Screen.newLine();
    Brain.Screen.print("lookahead: %f, %f", lookaheadPoint.x, lookaheadPoint.y);
    Brain.Screen.newLine();
    Brain.Screen.print(done);

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("error: %f", getDistance(finalPosition, finalPath.getPoint(finalPath.points.size()-1)));
    Controller1.Screen.newLine();
    Controller1.Screen.print("endVel: %f", finalPath.getPoint(finalPath.points.size()-1).targetVelocity);
    wait(20, msec);
  }
  return 1;
}

void skillsAuto()  { //2
  //Run all threads
  enableOdom = true;

  initOdom();
  vex::task runOdom(RunOdom);

  done = false;
  Path desPath;
  desPath = getPathMotion(0);
  maxPathVelocity = 150;
  maxAcceleration = 50;
  vex::task brainprint(brainPrint);

  driveTo(desPath, 25, 5000, false);

  //desPath = getPathMotion(1);

  //done = true;
  //driveTo(desPath, 0, 4, 4, 1000, true, 1);
  // Brain.Screen.setCursor( 1, 1 );
  // for(int i=0;i<8;i++)
  //   Brain.Screen.print(std::string(&pathBuffer[i]));
  //   Brain.Screen.newLine();

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

void rushAutoRight() {
  
  //INIT PID
  enableDrivePID = true;
  vex::task driveOnlyPID(driverOnlyPID);


  //START:

  resetDriveSensors = true;
  FrontClampOpen();
  desiredMotorVal = 1200;

  turnVals.desiredValue = 0;

  vex::task::sleep(1250);

  resetDriveSensors = true;
  FrontClampClose();
  desiredMotorVal = 0;
  turnVals.desiredValue = 0;

  vex::task::sleep(350);

  resetDriveSensors = true;
  desiredMotorVal = -500;
  turnVals.desiredValue = 0;
  
  vex::task::sleep(1500);

  resetDriveSensors = true;
  desiredMotorVal = 0;
  turnVals.desiredValue = -90;
  
  vex::task::sleep(1500);

  // resetDriveSensors = true;
  // driveVals.desiredValue = 400;
  // turnVals.desiredValue = 0;

  // vex::task::sleep(1050);

  // resetDriveSensors = true;
  // BackTilterExtend();
  // BackClampOpen();
  // driveVals.desiredValue = 500;
  // turnVals.desiredValue = -90;

  // vex::task::sleep(1050);

  // resetDriveSensors = true;
  // driveVals.desiredValue = -600;
  // turnVals.desiredValue = 0;

  // vex::task::sleep(1050);

  // resetDriveSensors = true;
  // BackClampClose();
  // BackTilterRetract();
  // driveVals.desiredValue = 0;
  // turnVals.desiredValue = 0;

  // vex::task::sleep(1050);

}

void testPID(){
  //INIT PID
  enableDrivePID = true;
  vex::task driveOnlyPID(driverOnlyPID);


  //START:

  resetDriveSensors = true;
  desiredMotorVal = 1100;
  turnVals.desiredValue = 0;

  vex::task::sleep(2050);

  resetDriveSensors = true;
  desiredMotorVal = -1100;
  turnVals.desiredValue = 0;

  vex::task::sleep(2050);
}