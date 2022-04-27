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
inline double offset, xOffset, yOffset;

inline double motorLeftPos, motorRightPos;
inline double motorLeftTrack, motorRightTrack;

inline double prevRVal, prevLVal; //inches

inline double prevOrientation = 0;
inline double absOrientation = 0;
inline double deltaOrientation = 0;
inline Point finalPosition({0,0});

inline double deltaLT = 0, deltaRT = 0; //Tracking wheel distance travelled since cycle
inline double totalDeltaLT = 0, totalDeltaRT = 0; //Tracking wheel total change since reset



//---Funcs----
void getWheelVals();
void odom();
void initOdom();


//--------__Parameters__--------//

inline double wheelDiameter = 4; //wheel diameter 
inline double lTCentre = 0; // Left Tracking Wheel distance to centre: Sl
inline double bTCentre = 0; // Back Tracking Wheel distance to centre: Ss

inline double trackWidth = 12.25; // Track Width

//--------__Functions__--------//




#endif