
#include <vector>
#include <vex.h>
#include <cmath>
#include <deque>
#include "structs/path-struct.h"

//checks if a line segment intersects with circle returns t value [0,1] 
inline double checkIntersection(Point startSegment, Point endSegment, Point circleCentre, double radius) {
  double t1, t2;
  Vector d(startSegment, endSegment);
  Vector f(circleCentre, startSegment);

  double a = d.Dot(d);
  double b = 2 * f.Dot(d);
  double c = f.Dot(f) - radius * radius;
  double discriminant = (b * b) - (4 * a * c);

  if (discriminant <= 0) {
    return -1; // no intersection
  }
  else {
    discriminant = sqrt(discriminant);
    t1 = (-b - discriminant) / (2 * a);
    t2 = (-b + discriminant) / (2 * a);


    if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
      //return t1 intersection
      return t1;
    }
    if (t2 >= 0 && t2 <= 1 && t1 >= 0 && t1 <= 1) {
      //return t2
      return t2;
    }
    return -1; //return no intersection
  }

}

inline Path getIntersectionPoints(double m, double c, Point circleCentre, double radius) {
  /*
  x = (-(2mc - 2h - 2mk) ± sqrt( (2mc-2h-2mk)^2 - 4(m^2 + 1)(c^2 - 2kc + k^2 + h^2) ) / 2(m^2 +1)
  
  */
  Path intersections;

  double coeffA = (m * m) + 1;
  double coeffB = ( (2 * m * c) - (2 * circleCentre.x) - (2 * m * circleCentre.y) );
  double coeffC = ((c * c) - (2 * circleCentre.y * c) + (circleCentre.y * circleCentre.y) + (circleCentre.x * circleCentre.x) - (radius*radius));

  double discriminant =  (coeffB * coeffB) - (4*coeffA*coeffC);

  if (discriminant < 0) {
    return intersections; //no intersection (point.size() = 0)
  }
  else {
    discriminant = sqrt(discriminant);
    double x1 = (-coeffB - discriminant) / (2 * coeffA);
    double x2 = (-coeffB + discriminant) / (2 * coeffA);

    double y1 = (m * x1) + c;
    double y2 = (m * x2) + c;

    intersections.addPoint(Point({ x1, y1 }));
    intersections.addPoint(Point({ x2, y2 }));

    return intersections;
  }

}


inline Path pointsAroundArc(Point start, Point end, Point circleCentre, double radius, double cutoffPercentage) {

  Path path;

  //Find 2 perpendicular points:
  // 1. find equation of line
  // 2. find perpendicular bisector that passes through circle centre
  // 3. Find intersection point


  //__Equation of line__
  double dy = end.y - start.y;
  double dx = end.x - start.x;
  double m = dy / dx;
  double c = end.y - (m * end.x);


  //__Perpendicular Bisector__
  double mPerp = -(1 / m);
  double cPerp = circleCentre.y - (mPerp * circleCentre.x);

  //__Get Intersection Points__
  Path intersection = getIntersectionPoints(mPerp, cPerp, circleCentre, radius);
  Point p1 = intersection.points.at(0);
  Point p2 = intersection.points.at(1);

  //__Find Bisector Point__
  double bisectorX = (cPerp - c) / (m - mPerp);
  double bisectorY = (m * bisectorX) + c;
  
  Point bisector({ bisectorX, bisectorY });

  double distance1 = getDistance(p1, bisector);
  double distance2 = getDistance(p2, bisector);


  
  if (distance1 >= distance2) {
    //omit p1 if distance1 is larger
    //use p2

    //if distance2 is smaller that cutoff of radius
    if (distance2 <= cutoffPercentage * radius) {
      //add point to path
      path.addPoint(p2);
      return path;
    }
    else {
      //recursion
      Path addedPoints = pointsAroundArc(p2, end, circleCentre, radius, cutoffPercentage);
      for (int i = 0; i < addedPoints.points.size(); i++) {
        path.addPoint(addedPoints.getPoint(i));
      }
      path.addPoint(p2);
      return path;
    }
  }
  else {
    //omit p2 if distance2 is larger
    //use p1

    //if distance1 is smaller that cutoff of radius
    if (distance1 <= cutoffPercentage * radius) {
      //add point to path
      path.addPoint(p1);
      return path;
    }
    else {
      //recursion
      Path addedPoints = pointsAroundArc(p1, end, circleCentre, radius, cutoffPercentage);
      path.addPoint(p1);
      for (int i = 0; i < addedPoints.points.size(); i++) {
        path.addPoint(addedPoints.getPoint(i));
      }
      
      return path;
    }
  }
  
}


inline Path CreatePath(Point startingPoint, Point endingPoint, double approachHeading, double avoidanceCircleRadius) {
  Path path;
  path.addPoint(startingPoint);

  double referenceAngle;

  //find reference angle
  double x_diff;
  double y_diff;
  Point targetPoint, furtherTargetPoint;

  //0-pi/2
  if (approachHeading < (M_PI/2) && approachHeading > 0) {
    referenceAngle = (M_PI/2) - approachHeading;
    x_diff = avoidanceCircleRadius * cos(referenceAngle);
    y_diff = avoidanceCircleRadius * sin(referenceAngle);

    targetPoint.setCoordinates({ endingPoint.x - x_diff, endingPoint.y - y_diff });
    furtherTargetPoint.setCoordinates({ endingPoint.x - 0.65 *x_diff, endingPoint.y - 0.65 *y_diff });
  }

  //pi/2 - pi
  else if (approachHeading < (M_PI) && approachHeading > (M_PI/2)) {
    referenceAngle = approachHeading - (M_PI / 2);
    x_diff = avoidanceCircleRadius * cos(referenceAngle);
    y_diff = avoidanceCircleRadius * sin(referenceAngle);

    targetPoint.setCoordinates({ endingPoint.x - x_diff, endingPoint.y + y_diff });
    furtherTargetPoint.setCoordinates({ endingPoint.x - 0.65 * x_diff, endingPoint.y + 0.65 * y_diff });
  }

  //pi - 3pi/2
  else if (approachHeading < (3*M_PI/2) && approachHeading > M_PI) {
    referenceAngle = (3*M_PI / 2) - approachHeading;
    x_diff = avoidanceCircleRadius * cos(referenceAngle);
    y_diff = avoidanceCircleRadius * sin(referenceAngle);

    targetPoint.setCoordinates({ endingPoint.x + x_diff, endingPoint.y + y_diff });
    furtherTargetPoint.setCoordinates({ endingPoint.x + 0.65 * x_diff, endingPoint.y + 0.65 * y_diff });
  }

  //3pi/2 - 2pi
  else if (approachHeading < (2*M_PI) && approachHeading > (3*M_PI/2)) {
    referenceAngle = (M_PI / 2) - approachHeading;
    x_diff = avoidanceCircleRadius * cos(referenceAngle);
    y_diff = avoidanceCircleRadius * sin(referenceAngle);

    targetPoint.setCoordinates({ endingPoint.x + x_diff, endingPoint.y - y_diff });
    furtherTargetPoint.setCoordinates({ endingPoint.x + 0.65 * x_diff, endingPoint.y - 0.65 * y_diff });
  }

  //approach heading 0 or 2pi
  else if (approachHeading == 0 || approachHeading == 2 * M_PI) {
    targetPoint.setCoordinates({ endingPoint.x, endingPoint.y - avoidanceCircleRadius });
    furtherTargetPoint.setCoordinates({ endingPoint.x , endingPoint.y - 0.65 * avoidanceCircleRadius });
  }


  //approach heading pi/2
  else if (approachHeading == M_PI/2) {
    targetPoint.setCoordinates({ endingPoint.x - avoidanceCircleRadius, endingPoint.y });
    furtherTargetPoint.setCoordinates({ endingPoint.x - 0.65 * avoidanceCircleRadius , endingPoint.y  });
  }


  //approach heading pi
  else if (approachHeading == M_PI) {
    targetPoint.setCoordinates({ endingPoint.x , endingPoint.y + avoidanceCircleRadius });
    furtherTargetPoint.setCoordinates({ endingPoint.x  , endingPoint.y + 0.65 * avoidanceCircleRadius });
  }

  //approach heading 3pi/2
  else if (approachHeading == M_PI / 2) {
    targetPoint.setCoordinates({ endingPoint.x + avoidanceCircleRadius, endingPoint.y });
    furtherTargetPoint.setCoordinates({ endingPoint.x + 0.65 * avoidanceCircleRadius , endingPoint.y  });
  }

  //check if a straight line to the point intersects with circle
  if (checkIntersection(startingPoint, targetPoint, endingPoint, avoidanceCircleRadius) == -1) {
    path.addPoint(furtherTargetPoint);
    path.addPoint(endingPoint);
    return path;
  } 

  else {
    //loop

    //Find 2 perpendicular points:
    // 1. find equation of line
    // 2. find perpendicular bisector that passes through circle centre
    // 3. Find intersection point

    //Find 2 segment lengths
    //ommit the larger length
    //check if arc length is less than 10% of avoidance radius
    //if true: 
    //  add point to path
    //  break loop
    //else:
    //  add point to path
    //  repeat loop with new point & target point
    Path addedPoints = pointsAroundArc(startingPoint, targetPoint, endingPoint, avoidanceCircleRadius, 0.1);

    for (int i = 0; i < addedPoints.points.size(); i++) {
      path.addPoint(addedPoints.getPoint(i));
    }

    //add a point further inside the circle so theres a less sharp curve
    path.addPoint(furtherTargetPoint);
    path.addPoint(endingPoint);

  }



  return path;
}
