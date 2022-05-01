#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "vex.h"
#include <cmath>
#include "calc-funcs.h"
#include "pid.h"
#include "rate-limiter.h"

//-------------------DRIVE BASE---------------------------------


inline double kDrivePure[3] = {0.021, 0.0, 0.001}; //Starting Vals (No goal being lifted)
inline PID driveRightVals(kDrivePure);
inline PID driveLeftVals(kDrivePure);

inline double kDriveMotor[3] = {0.026, 0.0, 0.001}; //Starting Vals (No goal being lifted)
inline double kDriveInertial[3] = {0.23, 0.0, 0.001}; //Distance sensor vals

inline double desiredMotorVal = 0;
inline PID driveLVals(kDriveMotor);
inline PID driveRVals(kDriveMotor);
inline PID turnVals(kDriveInertial);

//Vars modified for use
inline bool enableDrivePID = true;
inline bool enablePureDrive = true;
inline bool enableDriverPID = true;

inline bool resetDriveSensors = false;

int DrivePID();
int PureDrive();
int driverOnlyPID();



#endif