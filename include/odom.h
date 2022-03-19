#include "vex.h"
#include <cmath>
#include <vector>


/*

..Units..

Lengths : Inches
Angles : Radians

*/


//--------__Variables__--------//

inline double prevLTval = 0, prevRTval = 0, prevBTval = 0; //previous tracking wheel encoder angle  
inline double prevGblPos[2] = {0,0};  //Prev global position vector [x,y]
inline double prevGblOrientation = 0; //Prev global orientation
inline double resetGblOrientation = 0; //Prev Global orientation since last reset

inline double lTval, rTval, bTval; //Current tracking wheel encoder angle  
inline double deltaLT, deltaRT, deltaBT; //Tracking wheel distance travelled since cycle
inline double totalDeltaLT, totalDeltaRT; //Tracking wheel total change since reset
inline double absOrientation = 0;
inline double deltaOrientation = 0;
inline double lclOffset[2] = {0,0};
inline double avgOrientation = 0;
inline double lclOffsetPolar[2] = {0,0};
inline double gblOffsetPolar[2] = {0,0};
inline double gblOffset[2] = {0,0};
inline double absPos[2]= {0,0};

//--------__Parameters__--------//

inline double wheelDiameter = 2.75; //wheel diameter 
inline double lTCentre = 1; // Left Tracking Wheel distance to centre: Sl
inline double rTCentre = 1; // Right Tracking Wheel distance to centre: Sr
inline double bTCentre = 1; // Back Tracking Wheel distance to centre: Ss


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

void fullOdomCycle();






