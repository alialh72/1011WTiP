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
// LeftDrive            motor_group   8, 12           
// RightDrive           motor_group   2, 14           
// LeftDriveUp          motor         4               
// RightDriveUp         motor         1               
// TWLeft               encoder       C, D            
// TWRight              encoder       E, F            
// TWHorizontal         encoder       G, H            
// Expander             triport       11              
// FBLift               motor         21              
// BackClamp            digital_out   E               
// BackTilter1          digital_out   F               
// BackTilter2          digital_out   G               
// FBLiftRotation       rotation      10              
// InertialSensor       inertial      9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include "pure-pursuit.h"

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

    if (Controller1.ButtonR1.pressing()) {
      FBLift.spin(forward, 12, voltageUnits::volt);
    } else if (Controller1.ButtonR2.pressing()) {
      FBLift.spin(reverse, 12, voltageUnits::volt);
    } else {
      FBLift.stop(brakeType::hold);
    }

    

    fullOdomCycle();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(absPos[0]);
    Brain.Screen.newLine();
    Brain.Screen.print(absPos[1]);
    Brain.Screen.newLine();
    Brain.Screen.print("abs orientation: %f", absOrientation * 180 / M_PI);
    Brain.Screen.newLine();
    Brain.Screen.print("BackWheel: %f", deltaBT);
    Brain.Screen.newLine();
    Brain.Screen.print("RightWheel: %f", totalDeltaRT);
    Brain.Screen.newLine();
    Brain.Screen.print("LeftWheel: %f", totalDeltaLT);


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
