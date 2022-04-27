#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "vex.h"
#include <cmath>
#include "calc-funcs.h"
#include "pid.h"
#include "rate-limiter.h"

//-------------------DRIVE BASE---------------------------------
inline PID driveLeftVals;
inline PID driveRightVals;

limiter lim;

//Vars modified for use
inline bool enableDrivePID = true;

int DrivePID();




#endif