#ifndef ODOM_H
#define ODOM_H

#include "vex.h"
#include <cmath>
#include <vector>
#include "structs/point-struct.h"
#include "calc-funcs.h"

/*

..Units..

Lengths : Inches
Angles : Radians

*/


//----New Odom-----
inline double arcRadiusL, arcRadiusR, arcRadius;
inline double arcRadiusF, arcRadiusB;
inline double offset, xOffset, yOffset;
inline double offsetB, offsetF;
inline double rPolar, thetaPolar;

inline double motorLeftPos, motorRightPos, backWheelPos, forwardWheelPos;
inline double motorLeftTrack, motorRightTrack, backWheelTrack, forwardWheelTrack;

inline double prevRVal, prevLVal, prevBVal, prevFVal; //inches

inline double prevOrientation = 0;
inline double absOrientation = 0;
inline double deltaOrientation = 0;
inline Point finalPosition({0,0});

inline double deltaLT = 0, deltaRT = 0, deltaBT = 0, deltaFT = 0; //Tracking wheel distance travelled since cycle
inline double totalDeltaLT = 0, totalDeltaRT = 0, totalDeltaBT = 0, totalDeltaFT = 0; //Tracking wheel total change since reset


inline double h; // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
inline double i; // Half on the angle that I've traveled
inline double h2; // The same as h but using the back instead of the side wheels
inline double r, r2; // The radius of the circle the robot travel's around with the right side of the robot
inline double sinI;
//---Funcs----
void getWheelVals();
void odom();
void odomTracking();
void initOdom();


//--------__Parameters__--------//

inline double wheelDiameter = 4; //wheel diameter 
inline double lTCentre = 0; // Left Tracking Wheel distance to centre: Sl
inline double bTCentre = 0; // Back Tracking Wheel distance to centre: Ss

inline double trackWidth = 12.25; // Track Width

//--------__Functions__--------//




#endif