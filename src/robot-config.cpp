#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
triport Expander = triport(PORT7);
controller Controller1 = controller(primary);
motor LeftDriveMotorA = motor(PORT3, ratio18_1, true);
motor LeftDriveMotorB = motor(PORT1, ratio18_1, true);
motor_group LeftDrive = motor_group(LeftDriveMotorA, LeftDriveMotorB);
motor RightDriveMotorA = motor(PORT16, ratio18_1, false);
motor RightDriveMotorB = motor(PORT18, ratio18_1, false);
motor_group RightDrive = motor_group(RightDriveMotorA, RightDriveMotorB);
motor LeftDriveUp = motor(PORT2, ratio18_1, false);
motor RightDriveUp = motor(PORT17, ratio18_1, true);
encoder TWParallel = encoder(Expander.C);
encoder TWHorizontal = encoder(Expander.A);
motor FBLift = motor(PORT20, ratio36_1, true);
digital_out BackClamp = digital_out(Brain.ThreeWirePort.H);
digital_out BackTilter = digital_out(Brain.ThreeWirePort.B);
rotation FBLiftRotation = rotation(PORT10, false);
inertial InertialRight = inertial(PORT19);
distance FrontDistance = distance(PORT14);
digital_out FrontClamp = digital_out(Brain.ThreeWirePort.A);
motor Intake = motor(PORT11, ratio6_1, false);
inertial InertialLeft = inertial(PORT6);
controller Controller2 = controller(partner);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}