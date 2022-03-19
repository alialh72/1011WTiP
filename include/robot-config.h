using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group LeftDrive;
extern motor_group RightDrive;
extern motor LeftDriveUp;
extern motor RightDriveUp;
extern encoder TWLeft;
extern encoder TWRight;
extern encoder TWHorizontal;
extern triport Expander;
extern motor FBLift;
extern digital_out BackClamp;
extern digital_out BackTilter1;
extern digital_out BackTilter2;
extern rotation FBLiftRotation;
extern inertial InertialSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );