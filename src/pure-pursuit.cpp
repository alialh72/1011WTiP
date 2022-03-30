#include "pure-pursuit.h"


int RunOdom() {
  while (enableOdom) {
    fullOdomCycle();

    vex::task::sleep(20);
  }

  return 1;
}


Path FillPointVals(Path cpath) {
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
    } 

    path.points.at(i).setDistance(pointDistance);

    runningDistance += pointDistance;


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
    double pointDiff = path.getPoint(i+1).distanceFromStart - path.getPoint(i+1).distanceFromStart;

    double newTargetVelocity = std::min(path.getPoint(i).maximumVelocity , sqrt(pow(path.getPoint(i+1).targetVelocity,2) + (2*maxAcceleration*pointDiff)) );

    path.points.at(i).setTargetVelocity(newTargetVelocity);
  }



  return path;
}


void PreAutonPurePursuit() {
  //Define path:
  //  --> Define a set of waypoints for the robot to follow
  desiredPath.points = { Point({1,3}), Point({3,4})};

  //Smooth path
  finalPath = GenerateSmoothPath(desiredPath);

  //Fill point vals
  //--> set the distance from start of each point and the curvature
  finalPath = FillPointVals(finalPath);


}

int FollowPath() {

  while (enableFollowPath) {
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

    //______Find lookahead point______

    for (int i = 0; i < finalPath.points.size(); i++) {
      //Line Segment



    }


  }
  
  return 1;
}


void PurePursuitController(){



  //Run odometry thread to get updated position of robot
  enableOdom = true;
  vex::task runOdom(RunOdom);

}