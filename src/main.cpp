/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftDrive            motor_group   3, 1            
// RightDrive           motor_group   16, 18          
// LeftDriveUp          motor         2               
// RightDriveUp         motor         17              
// TWParallel           encoder       C, D            
// TWHorizontal         encoder       A, B            
// Expander             triport       7               
// FBLift               motor         20              
// BackClamp            digital_out   H               
// BackTilter           digital_out   B               
// FBLiftRotation       rotation      10              
// InertialRight        inertial      19              
// FrontDistance        distance      14              
// FrontClamp           digital_out   A               
// Intake               motor         11              
// InertialLeft         inertial      6               
// Controller2          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "control-funcs.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE
  vexcodeInit();

  PreAutonPurePursuit();

}


//========================================================

//easy swap between autos
void autonomous(void) {
  skillsAuto();
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/






void usercontrol(void) {

  vex::controller Controller1 (vex::controllerType::primary);
  vex::controller Controller2 (vex::controllerType::partner);


  //---------------Settings---------------
  double turnImportance = 0.1;
  
  LeftDrive.setStopping(hold);
  LeftDriveUp.setStopping(hold);
  RightDrive.setStopping(hold);
  RightDriveUp.setStopping(hold);

  // InertialSensor.calibrate();
  // while(InertialSensor.isCalibrating()){
  //   wait(20, msec);
  // }
  // InertialSensor.resetRotation();

  //ringintake thread
  vex::task runIntake(RingIntake);

  while (1) {
    odom();
    //---------------Drivetrain---------------
    double motorForwardVal = Controller1.Axis3.position(percent);
    double motorTurnVal = Controller1.Axis1.position(percent);

    //Volts Range:  -12 --> 12
    double motorTurnVolts = motorTurnVal * 0.12; //convert percentage to volts
    double motorForwardVolts = motorForwardVal * 0.12 * (1 - (std::abs(motorTurnVolts)/12) * turnImportance); 
    
    LeftDrive.spin(forward, motorForwardVolts + motorTurnVolts, voltageUnits::volt);
    LeftDriveUp.spin(forward, motorForwardVolts + motorTurnVolts, voltageUnits::volt);
    RightDrive.spin(forward, (motorForwardVolts) - motorTurnVolts, voltageUnits::volt);
    RightDriveUp.spin(forward, (motorForwardVolts) - motorTurnVolts, voltageUnits::volt);

    //Four Bar Lift
    if (Controller1.ButtonL1.pressing()) {
      FBLift.spin(forward, 10, voltageUnits::volt);
    } else if (Controller1.ButtonL2.pressing()) {
      FBLift.spin(reverse, 10, voltageUnits::volt);
    } else {
      FBLift.stop(brakeType::hold);
    }
    //Front Clamp
    Controller1.ButtonR1.pressed(FrontClampOpen);
    Controller1.ButtonR2.pressed(FrontClampClose);

    //Back Clamp
    Controller1.ButtonUp.pressed(BackClampOpen);
    Controller1.ButtonDown.pressed(BackClampClose);

    
    //_______CONTROLER 2_________j

    //Ring Intake
    Controller2.ButtonA.pressed(swapRingOn);
    Controller2.ButtonB.pressed(swapRingDirection);

    //Back Tilters
    Controller2.ButtonLeft.pressed(BackTilterRetract);
    Controller2.ButtonRight.pressed(BackTilterExtend);


    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("motorPosL&R: %f, %f", LeftDrive.position(deg)*M_PI/180, RightDrive.position(deg)*M_PI/180);
    Brain.Screen.newLine();
    Brain.Screen.print("trackL&RPos: %f, %f", motorLeftPos, motorRightPos);
    Brain.Screen.newLine();
    Brain.Screen.print("trackR: %f   L:, %f", totalDeltaRT, totalDeltaLT);
    Brain.Screen.newLine();
    Brain.Screen.print("final posx: %f, %f", finalPosition.x, finalPosition.y);
    Brain.Screen.newLine();
    Brain.Screen.print("absOrientation: %f", absOrientation);
    Brain.Screen.newLine();
    Brain.Screen.print("inertial: %f", getInertialReading());
    Brain.Screen.newLine();
    Brain.Screen.print("deltaOrientation: %f", deltaOrientation);


    double currentVelocityLeft = (RightDrive.velocity(dps) * M_PI/180 * 4/2 * 7/5);  //wheel diameter/2
    double currentVelocityRight = (LeftDrive.velocity(dps) * M_PI/180 * 4/2 * 7/5);  //wheel diameter/2

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("mVLR: %f, %f", currentVelocityLeft, currentVelocityRight);
    Controller1.Screen.newLine();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
