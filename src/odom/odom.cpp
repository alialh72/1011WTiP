#include "odom/odom.h"

void initOdom() {
  prevOrientation = 0;
  absOrientation = 0;
  deltaOrientation = 0;
  Point finalPosition({0,0});

  deltaLT = 0, deltaRT = 0; //Tracking wheel distance travelled since cycle
  totalDeltaLT = 0, totalDeltaRT = 0; //Tracking wheel total change since reset

  LeftDrive.resetPosition();
  RightDrive.resetPosition();

  arcRadiusL = 0, arcRadiusR = 0, arcRadius =0;
  offset = 0, xOffset = 0, yOffset = 0;

  motorLeftPos = 0, motorRightPos = 0;
  motorLeftTrack = 0, motorRightTrack = 0;

  prevRVal = 0, prevLVal = 0; //inches
}


void getWheelVals() {
  //Motor Wheel Vals
  motorLeftPos = (LeftDrive.position(degrees) * M_PI / 180 * 7/5); //in rads
  motorRightPos = (RightDrive.position(degrees) * M_PI / 180 * 7/5); //in rads

  motorLeftTrack = motorLeftPos * wheelDiameter/2; //in rads
  motorRightTrack = motorRightPos * wheelDiameter/2; //in rads

  deltaRT = motorRightTrack - prevRVal;
  deltaLT =  motorLeftTrack - prevLVal;


  prevLVal = motorLeftTrack;
  prevRVal = motorRightTrack;

  totalDeltaRT += deltaRT;
  totalDeltaLT += deltaLT;
}


void odom() {
  getWheelVals();

  absOrientation = getInertialReading() * M_PI / 180;
  deltaOrientation = absOrientation - prevOrientation;
  prevOrientation = absOrientation;

  if (deltaOrientation != 0) {
    arcRadiusL = deltaLT/deltaOrientation;
    arcRadiusR = deltaRT/deltaOrientation;

    arcRadius = (arcRadiusL + arcRadiusR)/2;
    
    offset = 2*arcRadius*sin(deltaOrientation/2);

  } 
  else {
    arcRadius = (deltaLT + deltaRT)/2;
    
    offset = 2*arcRadius*sin(deltaOrientation/2);
  }


  xOffset = offset*sin((deltaOrientation/2) + absOrientation);
  yOffset = offset*cos((deltaOrientation/2) + absOrientation);
    
  finalPosition.x += xOffset;
  finalPosition.y += yOffset;

}