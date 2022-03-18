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
// LeftDrive            motor_group   2, 3            
// RightDrive           motor_group   8, 9            
// LeftDriveUp          motor         1               
// RightDriveUp         motor         7               
// TWLeft               encoder       A, B            
// TWRight              encoder       C, D            
// TWHorizontal         encoder       E, F            
// Expander             triport       5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
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
}


//========================================================

//easy swap between autos
void autonomous(void) {

}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  //---------------Settings---------------
  double turnImportance = 0.1;
  
  LeftDrive.setStopping(hold);
  LeftDriveUp.setStopping(hold);
  RightDrive.setStopping(hold);
  RightDriveUp.setStopping(hold);

  while (1) {
    //---------------Drivetrain---------------
    double motorForwardVal = Controller1.Axis3.position(percent);
    double motorTurnVal = Controller1.Axis1.position(percent);

    //Volts Range:  -12 --> 12
    double motorTurnVolts = motorTurnVal * 0.12; //convert percentage to volts
    double motorForwardVolts = motorForwardVal * 0.12 * (1 - (std::abs(motorTurnVolts)/12) * turnImportance); 
    
    LeftDrive.spin(forward, motorForwardVolts + motorTurnVolts, voltageUnits::volt);
    LeftDriveUp.spin(forward, motorForwardVolts + motorTurnVolts, voltageUnits::volt);
    RightDrive.spin(forward, motorForwardVolts - motorTurnVolts, voltageUnits::volt);
    RightDriveUp.spin(forward, motorForwardVolts - motorTurnVolts, voltageUnits::volt);

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
