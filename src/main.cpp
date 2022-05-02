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
#include "main.h"
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
  //skillsAuto();
  rushAutoRight();
 //testPID();
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {
  limiter voltlimit, armlimit;

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

//jjkkjhkj
  while (1) {
    odomTracking();
    
    //---------------Drivetrain---------------
    double motorForwardVal = Controller1.Axis3.position(percent);
    double motorTurnVal = Controller1.Axis1.position(percent);

    //Volts Range:  -12 --> 12
    double motorTurnVolts = motorTurnVal * 0.12; //convert percentage to volts
    double motorForwardVolts = motorForwardVal * 0.12 * (1 - (std::abs(motorTurnVolts)/12) * 0.1); 
    motorForwardVolts = voltlimit.rateLimiter(motorForwardVolts, 45);
    double leftVolts = motorForwardVolts + motorTurnVolts;
    double rightVolts = motorForwardVolts - motorTurnVolts;
    
    // leftVolts = voltlimit.rateLimiter(leftVolts, 100);
    // rightVolts = voltlimit.rateLimiter(rightVolts, 100);
    
    LeftDrive.spin(forward, leftVolts, voltageUnits::volt);
    LeftDriveUp.spin(forward, leftVolts, voltageUnits::volt);
    RightDrive.spin(forward, rightVolts, voltageUnits::volt);
    RightDriveUp.spin(forward, rightVolts, voltageUnits::volt);

    //Four Bar Lift
    if (Controller1.ButtonL1.pressing()) {
      double volts = 10;
      volts = armlimit.rateLimiter(volts, 5);
      FBLift.spin(forward, 10, voltageUnits::volt);
    } else if (Controller1.ButtonL2.pressing()) {
      double volts = 10;
      volts = armlimit.rateLimiter(volts, 5);
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
   // AlexDrive(voltlimit);j
    
    //_______CONTROLER 2_________j

    //Ring Intake
    Controller2.ButtonA.pressed(swapRingOn);
    Controller2.ButtonB.pressed(swapRingDirection);

    //Back Tilters
    Controller2.ButtonR1.pressed(BackTilterRetract);
    Controller2.ButtonR2.pressed(BackTilterExtend);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("trackH&PPos: %f, %f", TWHorizontal.position(deg), TWParallel.position(deg));
    Brain.Screen.newLine();
    Brain.Screen.print("trackFH: %f   Back:, %f", totalDeltaFT, totalDeltaBT);
    Brain.Screen.newLine();
    Brain.Screen.print("final posx: %f, %f", finalPosition.x, finalPosition.y);
    Brain.Screen.newLine();
    Brain.Screen.print("h: %f ,  h2: %f", h, h2);
    Brain.Screen.newLine();
    Brain.Screen.print("absOrientation: %f", absOrientation);
    Brain.Screen.newLine();
    Brain.Screen.print("prevOri: %f", prevOrientation);
    Brain.Screen.newLine();
    Brain.Screen.print("deltaOrientation: %f", deltaOrientation);




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
