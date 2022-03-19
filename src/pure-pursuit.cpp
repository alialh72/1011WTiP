#include "pure-pursuit.h"

// double GetDistanceAtPoint(int i) {

// }


int RunOdom() {
  while (enableOdom) {
    fullOdomCycle();

    vex::task::sleep(20);
  }

  return 1;
}

void FillPointVals(Path path) {

  double runningDistance = 0;
  for (int i = 0; i < path.points.size(); i++) {

    //__Set the point distances from start of path__
    double pointDistance = 0;

    //if point is not the first
    if (i != 0) {
      //find the difference between point i and point i-1
      double pointDiff = getDistance(path.getPoint(i-1), path.getPoint(i)); 

      pointDistance = runningDistance + pointDiff;
    } 

    path.points.at(i).setDistance(pointDistance);

    runningDistance += pointDistance;


    //__Set curavture of points__
    if (i != 0 && i != path.points.size()-1) {
      double curvature = getCurvature(path.getPoint(i), path.getPoint(i-1), path.getPoint(i+1));

      path.points.at(i).setCurvature(curvature);
    }

  }

}





void PurePursuitController(){
  //Define path:
  //  --> Define a set of waypoints for the robot to follow
  desiredPath.points = { Point({1,3}), Point({3,4})};

  //Smooth path
  finalPath = GenerateSmoothPath(desiredPath);





  //Run odometry thread to get updated position of robot
  enableOdom = true;
  vex::task runOdom(RunOdom);

}