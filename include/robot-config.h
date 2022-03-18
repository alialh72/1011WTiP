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

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );