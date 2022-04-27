#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "odom/odom.h"
#include "path-generation/path-smoother.h"
#include "path-generation/path-generator.h"
#include "pid/pid-controller.h"
#include "calc-funcs.h"
#include "rate-limiter.h"
#include <vex.h>
#include <cmath>
#include <algorithm>

/*

..Units..

Lengths : Inches
Angles : Radians

*/

//------BOOLEANS-----------
inline bool enableOdom = true;
inline bool enableFollowPath = true;

//------VARS--------
inline Path desiredPath;
inline Path finalPath;
inline Point lookaheadPoint;
inline Point closestPoint;
inline double curvature;
inline double targetVelRight;
inline double targetVelLeft;
inline double targetVelRobot;
inline double angularVel;


//------PARAMETERERS-------
inline double maxPathVelocity = 28; //inches per second
inline double kMaxVel = 3; //[1,5] higher k --> faster around turns 
inline double lookaheadDistance = 23;
inline double maxAcceleration = 4; //incher per second^2


//------PURE PURSUIT LOOP VARIABLES-------
inline double pathSize;
inline double shortestDistance;
inline double prevClosestPointIndex;
inline double closestPointIndex;

inline double aCurvatureSlope;
inline double bCurvatureSlope;
inline double cCurvatureSlope;
inline double relativeX;
inline double side;
inline double crossProduct;
inline double signedCurvature;

inline limiter l;
//-----CALC VELOCITY--------
inline double targetVel;
inline double targetRW, targetLW; //target right/left wheel velocities

//-----Path Numbers---------

inline std::vector<Path> paths;

int RunOdom();
int FollowPath();
void PurePursuitController();
void PreAutonPurePursuit();
void findClosestPoint();
void findLookaheadPoint();
void findCurvature();
void calculateWheelVelocities();
void setPathMotion(int i);
double rateLimiter(double val, double maxRate);


inline Path FillPointVals(Path cpath) {
  Path path = cpath; //make copy of variable

  double runningDistance = 0;
  for (int i = 0; i < path.points.size(); i++) {

    //______Set the point distances from start of path______
    double pointDistance = 0;

    //if point is not the first
    if (i != 0) {
      //find the difference between point i and point i-1
      double pointDiff = getDistance(path.getPoint(i-1), path.getPoint(i)); 

      pointDistance = runningDistance + pointDiff;
      runningDistance += pointDiff;
    } 

    path.points.at(i).setDistance(pointDistance);


    //______Set curvature of points______
    if (i != 0 && i != path.points.size()-1) {
      double curvature = calcCurvature(path.getPoint(i), path.getPoint(i-1), path.getPoint(i+1)); 

      path.points.at(i).setCurvature(curvature);
    } else {
      path.points.at(i).setCurvature(0);
    }

    //______Set max velocity of points______
    //Set max velocity of point i to a minimum of (Max Path Velocity , k/curavture at point i)
    path.points.at(i).setMaxVelocity(std::min(maxPathVelocity, kMaxVel/path.getPoint(i).curvature));
  }

  //----------Calculate Target Velocities-----------
  //1. simulate a robot running the path with max acceleration, starting at the end
  //2. vf = √(vi^2 + 2*a*d).
  // --> new velocity at point i = min(old target velocity at point i, √(velocity at point (i + 1))2 + 2 * a * distance )

  //Set velocity of last point to 0
  path.points.at(path.points.size()-1).setTargetVelocity(0);

  //loop through points back to front
  for (int i = path.points.size()-2; i >= 0 ; i--) {
    double pointDiff = path.getPoint(i+1).distanceFromStart - path.getPoint(i).distanceFromStart;

    double newTargetVelocity = std::min(path.getPoint(i).maximumVelocity , sqrt(pow(path.getPoint(i+1).targetVelocity,2) + (2*maxAcceleration*pointDiff)) );

    path.points.at(i).setTargetVelocity(newTargetVelocity);
  }

  return path;
}

#endif




