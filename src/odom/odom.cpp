#include "odom/odom.h"

void initOdom() {
  prevOrientation = 0;
  absOrientation = 0;
  deltaOrientation = 0;
  Point finalPosition({0,0});

  deltaLT = 0, deltaRT = 0, deltaBT = 0, deltaFT = 0; //Tracking wheel distance travelled since cycle
  totalDeltaLT = 0, totalDeltaRT = 0, totalDeltaBT = 0, totalDeltaFT = 0;; //Tracking wheel total change since reset

  LeftDrive.resetPosition();
  RightDrive.resetPosition();
  TWParallel.setPosition(0, deg);
  TWHorizontal.setPosition(0, deg);

  arcRadiusL = 0, arcRadiusR = 0, arcRadius = 0;
  offset = 0, xOffset = 0, yOffset = 0;

  motorLeftPos = 0, motorRightPos = 0, backWheelPos = 0, forwardWheelPos = 0;
  motorLeftTrack = 0, motorRightTrack = 0, backWheelTrack = 0, forwardWheelTrack = 0;

  prevRVal = 0, prevLVal = 0, prevBVal = 0, prevFVal = 0; //inches
}


void getWheelVals() {
  //Motor Wheel Vals
  motorLeftPos = (LeftDrive.position(degrees) * M_PI / 180 * 7/5); //in rads
  motorRightPos = (RightDrive.position(degrees) * M_PI / 180 * 7/5); //in rads

  forwardWheelPos = -TWParallel.position(degrees) * M_PI/180;
  backWheelPos = TWHorizontal.position(degrees) * M_PI / 180;

  motorLeftTrack = motorLeftPos * wheelDiameter/2; //in rads
  motorRightTrack = motorRightPos * wheelDiameter/2; //in rads
  backWheelTrack = backWheelPos * (2.75/2);
  forwardWheelTrack = forwardWheelPos * (2.75/2);

  deltaRT = motorRightTrack - prevRVal;
  deltaLT =  motorLeftTrack - prevLVal;
  deltaBT = backWheelTrack - prevBVal;
  deltaFT = forwardWheelTrack - prevFVal;


  prevLVal = motorLeftTrack;
  prevRVal = motorRightTrack;
  prevBVal = backWheelTrack;
  prevFVal = forwardWheelTrack;

  totalDeltaRT += deltaRT;
  totalDeltaLT += deltaLT;
  totalDeltaBT += deltaBT;
  totalDeltaFT += deltaFT;
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

void odomTracking() {
  getWheelVals();

  absOrientation = -(getInertialReading() * M_PI / 180) + (M_PI/2);
  deltaOrientation = absOrientation - prevOrientation;
  prevOrientation = absOrientation;

  if (deltaOrientation != 0) {
    //Forward Movement
    arcRadiusF = deltaFT/deltaOrientation;
    arcRadiusB = deltaBT/deltaOrientation;
    

    offsetF = 2*arcRadiusF*sin(deltaOrientation/2);
    offsetB = 2*arcRadiusB*sin(deltaOrientation/2);

  } 
  else {
    arcRadiusF = deltaFT;
    arcRadiusB = deltaBT;
    
    offsetF = 2*arcRadiusF*sin(deltaOrientation/2);
    offsetB = 2*arcRadiusB*sin(deltaOrientation/2);

  }

  rPolar = sqrt(pow(offsetF, 2) + pow(offsetB, 2));
  thetaPolar = tan(offsetF/offsetB);
  thetaPolar -= ((deltaOrientation/2) + absOrientation);

  xOffset = rPolar * cos(thetaPolar);
  yOffset = rPolar * sin(thetaPolar);

  // xOffset = offsetB*cos((deltaOrientation/2) + absOrientation);
  // yOffset = offsetF*cos((deltaOrientation/2) + absOrientation);
    
  finalPosition.x += xOffset;
  finalPosition.y += yOffset;

}