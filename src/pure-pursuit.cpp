#include "pure-pursuit.h"

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

    closestPoint = finalPath.getPoint(shortestIndex);


    //______Find lookahead point______

    double tVal = calcFractionalT(finalPath, Point({absPos[0], absPos[1]}), lookaheadDistance);

    //round t val down to get the start tval
    int startT = tVal; //get rids of decimals
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

  }
  
  return 1;
}


//---------------------------------
//PreAuton
//---------------------------------

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



//--------------------------------
//Controller
//--------------------------------

void PurePursuitController(){

  //Run odometry thread to get updated position of robot
  enableOdom = true;
  vex::task runOdom(RunOdom);

}