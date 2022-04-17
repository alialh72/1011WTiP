#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "vex.h"
#include <cmath>
#include "pid.h"

//-------------------DRIVE BASE---------------------------------

inline double kDrive[3] = {0.08, 0.0, 0.001}; //Starting Vals
inline PID driveLeftVals(kDrive);
inline PID driveRightVals(kDrive);

//Vars modified for use
inline bool enableDrivePID = true;

int drivePID();

#endif