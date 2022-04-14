#include "pure-pursuit.h"

double rateLimiter(double val, double maxRate) {
  double maxChange = 0.02 * maxRate;
  rateLimiterOutput += clamp(val - prevRateLimiterOutput, -maxChange, maxChange);
  return rateLimiterOutput;
}


//-------------------------------------------
//Follow path functions
//-------------------------------------------
void findClosestPoint() {
  //______Find closest point______
  //Start at prev point index +1

  //by default, set shortest distance to distance from last point
  double shortestDistance = getDistance(Point({absPos[0], absPos[1]}), finalPath.getPoint(finalPath.points.size()-1));
  double shortestIndex = finalPath.points.size()-1;
  
  for (int i = prevClosestPointIndex + 1; i < finalPath.points.size(); i++) {
    double robotDistance = getDistance(Point({absPos[0], absPos[1]}), finalPath.getPoint(i));

    if (robotDistance > shortestDistance) {
      shortestDistance = robotDistance;
      shortestIndex = i;
    }
  }

  closestPoint = finalPath.getPoint(shortestIndex);
}


void findLookaheadPoint() {
  //______Find lookahead point______

  double tVal = calcFractionalT(finalPath, Point({ absPos[0], absPos[1] }), lookaheadDistance);

  //round t val down to get the start tval
  int startT = tVal; //get rids of decimals (converting double to int drops decimals)
  //add 1 to find end point index
  Vector lookaheadSegment(finalPath.getPoint(startT), finalPath.getPoint(startT + 1));

  //similar triangles: 
  //Get only the decimal(the fractional distance along the line)
  double nonIndexedVal = fmod(tVal, 1); // y = x % 1  = 0.45 

  //find new hypotenuse and x y difference
  double similarHyp = nonIndexedVal * lookaheadSegment.magnitude;
  double xDiff = similarHyp * cos(lookaheadSegment.refAngle);
  double yDiff = similarHyp * sin(lookaheadSegment.refAngle);

  //angle stays the same
  //find new x & y distance
  //depending on quadrant, either add/subtract the x and y to the initial point
  switch (lookaheadSegment.quadrant) {
    case 1:
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x + xDiff, lookaheadSegment.startPoint.y + yDiff });
      break;

    case 2:
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x + xDiff, lookaheadSegment.startPoint.y - yDiff });
      break;

    case 3:
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x - xDiff, lookaheadSegment.startPoint.y - yDiff });
      break;

    case 4:
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x - xDiff, lookaheadSegment.startPoint.y + yDiff });
      break;

    case 5:       //straight up
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x, lookaheadSegment.startPoint.y + yDiff });
      break;

    case 6:      //straight right
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x + xDiff, lookaheadSegment.startPoint.y });
      break;

    case 7:      //straight down
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x, lookaheadSegment.startPoint.y - yDiff });
      break;

    case 8:       //straight left
      lookaheadPoint.setCoordinates({ lookaheadSegment.startPoint.x - xDiff, lookaheadSegment.startPoint.y });
      break;
  }

  //____________set vals of lookahead____________

  //______Distance from start______
  double distanceDiff = getDistance(lookaheadPoint, finalPath.getPoint(startT));
  double newDistance = finalPath.getPoint(startT).distanceFromStart + distanceDiff;
  lookaheadPoint.setDistance(newDistance);

  //______Curvature______
  double curvature = 0;
  if (startT + 2 < finalPath.points.size() - 1) {
    curvature = calcCurvature(lookaheadPoint, finalPath.getPoint(startT - 1), finalPath.getPoint(startT + 2));
  }
    
  lookaheadPoint.setCurvature(curvature);

  //______Set max velocity of lookahead____
  //Set max velocity of point i to a minimum of (Max Path Velocity , k/curavture at point i)
  lookaheadPoint.setMaxVelocity(std::min(maxPathVelocity, kMaxVel / lookaheadPoint.curvature));

  //______Set target velocity of lookahead____
  // --> new velocity at point i = min(old target velocity at point i, √(velocity at point (i + 1))2 + 2 * a * distance )
  if (startT + 1 < finalPath.points.size() - 1) {
    double pointDiff = finalPath.getPoint(startT + 1).distanceFromStart - lookaheadPoint.distanceFromStart;

    double newTargetVelocity = std::min(lookaheadPoint.maximumVelocity, sqrt(pow(finalPath.getPoint(startT + 1).targetVelocity, 2) + (2 * maxAcceleration * pointDiff)));

    lookaheadPoint.setTargetVelocity(newTargetVelocity);
  }
  else {
    lookaheadPoint.setTargetVelocity(0);
  }
    
}

void findCurvature() {
  //find curvature of current robot arc
  //curvature = 2x/L^2
  //where x is horizontal distance to the point and L is the lookahead

  aCurvatureSlope = 1 - (tan(absOrientation));
  bCurvatureSlope = 1;
  cCurvatureSlope = (tan(absOrientation) * absPos[0]) - absPos[1];

  relativeX = abs(aCurvatureSlope*lookaheadPoint.x + bCurvatureSlope*lookaheadPoint.y + cCurvatureSlope) / sqrt(pow(aCurvatureSlope, 2) + pow(bCurvatureSlope, 2));

  //get signed curvature
  //side = signum(cross product) = signum((By − Ry) * (Lx − Rx) − (Bx − Rx) * (Ly − Ry))
  //Bx = Rx + cos(robot angle)
  //By = Ry + sin(robot angle)
  
  //sign = signum(cross product)
  crossProduct =  sin(absOrientation) * (lookaheadPoint.x - absPos[0]) - cos(absOrientation) * (lookaheadPoint.y - absPos[1]);

  //signum ternary operator
  side = (crossProduct > 0) ? 1 : ((crossProduct < 0) ? -1 : 0);

  signedCurvature = ((2*relativeX)/pow(lookaheadDistance, 2))*side;

}

void calculateWheelVelocities() {
  //calculate wheel velocities
  /*
  V = target robot velocity
  L = target left wheel’s speed
  R = target right wheel’s speed
  C = curvature of arc
  W = angular velocity of robot
  T = track width

  V = (L + R)/2
  W = (L − R)/T
  V = W/C
  */

  //targetVel = rateLimiter(closestPoint.targetVelocity, maxAcceleration);
  targetVel = closestPoint.targetVelocity;

  targetLW = targetVel * (2 + (signedCurvature * trackWidth) )/2;
  targetRW = targetVel * (2 - (signedCurvature * trackWidth) )/2;


}



//-------------------------------------------
//Threads
//-------------------------------------------

int RunOdom() {
  while (enableOdom) {
    fullOdomCycle();

    vex::task::sleep(20);
  }

  return 1;
}

int UpdateVals() {

  while (enableFollowPath) {
    findClosestPoint();
    findLookaheadPoint();
    findCurvature();


    
    
    vex::task::sleep(20);
  }
  
  return 1;
}


//---------------------------------
//PreAuton
//---------------------------------

void PreAutonPurePursuit() {
  //Define path:
  //  --> Define a set of waypoints for the robot to follow
  desiredPath.points = { Point({1,3}), Point({3,4})};  //replace with actual path values

  //Smooth path
  finalPath = GenerateSmoothPath(desiredPath);

  //Fill point vals
  //--> set the distance from start of each point and the curvature
  finalPath = FillPointVals(finalPath);


}



//--------------------------------
//Controller
//--------------------------------

void PurePursuitController(){

  //Run odometry thread to get updated position of robot
  enableOdom = true;
  vex::task runOdom(RunOdom);
  vex::task updateVals(UpdateVals);

}