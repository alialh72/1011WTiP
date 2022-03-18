#include "vex.h"
#include <cmath>
#include <vector>


/*

..Units..

Lengths : Inches
Angles : Radians

*/


//--------__Variables__--------//

double prevLTval, prevRTval, prevBTval; //previous tracking wheel encoder angle  
double prevGblPos[2];  //Prev global position vector [x,y]
double prevGblOrientation; //Prev global orientation
double resetGblOrientation; //Prev Global orientation since last reset

double lTval, rTval, bTval; //Current tracking wheel encoder angle  
double deltaLT, deltaRT, deltaBT; //Tracking wheel distance travelled since cycle
double totalDeltaLT, totalDeltaRT; //Tracking wheel total change since reset
double absOrientation;
double deltaOrientation;
double lclOffset[2];
double avgOrientation;
double lclOffsetPolar[2];
double gblOffsetPolar[2];
double gblOffset[2];
double absPos[2];

//--------__Parameters__--------//

double wheelDiameter = 2.75; //wheel diameter 
double lTCentre; // Left Tracking Wheel distance to centre: Sl
double rTCentre; // Right Tracking Wheel distance to centre: Sr
double bTCentre; // Back Tracking Wheel distance to centre: Ss


//--------__Functions__--------//

void storeVals();
void getChangeCycle();
void updatePrevVals();
void calcChangeReset();
void calcAbsOrientation();
void calcChangeAngle();
void calcLclOffset();
void calcAvgOrientation();
void calcGblOffset();
void calcAbsPos();






