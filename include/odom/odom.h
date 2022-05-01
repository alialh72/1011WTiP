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