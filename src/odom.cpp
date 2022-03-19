#include "odom.h"


void storeVals(){
  //get encoder vals (degrees) --> convert to radians
  lTval = (TWLeft.position(degrees) * M_PI) / 180;  
  rTval = (TWRight.position(degrees) * M_PI) / 180;
  bTval = (TWHorizontal.position(degrees) * M_PI) / 180;
}

void getChangeCycle(){
  // arc length = theta * radius
  deltaLT = (lTval - prevLTval) * (wheelDiameter/2); 
  deltaRT = (rTval - prevRTval) * (wheelDiameter/2); 
  deltaBT = (bTval - prevBTval) * (wheelDiameter/2); 
}

void updatePrevVals(){
  prevLTval = lTval;
  prevRTval = rTval;
  prevBTval = bTval;

  prevGblOrientation = absOrientation;

  prevGblPos[0] = absPos[0];
  prevGblPos[1] = absPos[1];
}


void calcChangeReset(){
  totalDeltaLT += deltaLT;
  totalDeltaRT += deltaRT;
}

void calcAbsOrientation(){
  absOrientation = resetGblOrientation + (  (totalDeltaLT - totalDeltaRT) / (lTCentre + rTCentre)  );
}

void calcChangeAngle(){
  deltaOrientation = absOrientation - prevGblOrientation;
}

void calcLclOffset(){
  if (deltaOrientation == 0) {
    lclOffset[0] = deltaBT;
    lclOffset[1] = deltaRT;
  } 
  else {
    lclOffset[0] = 2*sin(deltaOrientation/2) * ( (deltaBT/deltaOrientation) + bTCentre); 
    lclOffset[1] = 2*sin(deltaOrientation/2) * ( (deltaRT/deltaOrientation) + rTCentre);
  }
}

void calcAvgOrientation(){
  avgOrientation = prevGblOrientation + (deltaOrientation/2);
}

void calcGblOffset(){
  lclOffsetPolar[0] = hypot(lclOffset[0], lclOffset[1]);
  lclOffsetPolar[1] = atan(lclOffset[1]/lclOffset[0]);

  gblOffsetPolar[0] = lclOffsetPolar[1];
  gblOffsetPolar[1] = lclOffsetPolar[1] - avgOrientation;

  gblOffset[0] = gblOffsetPolar[0] * cos(gblOffsetPolar[1]);
  gblOffset[1] = gblOffsetPolar[0] * sin(gblOffsetPolar[1]);
}

void calcAbsPos(){
  absPos[0] = prevGblPos[0] + gblOffset[0];
  absPos[1] = prevGblPos[1] + gblOffset[1];
}

void fullOdomCycle() {
  storeVals();
  getChangeCycle();
  updatePrevVals();
  calcChangeReset();
  calcAbsOrientation();
  calcChangeAngle();
  calcLclOffset();
  calcAvgOrientation();
  calcGblOffset();
  calcAbsPos();
}