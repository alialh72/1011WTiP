#ifndef PATHGENERATOR_H
#define PATHGENERATOR_H

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

inline Path getIntersectionPoints(double m, double c, Point circleCentre, double radius);
inline Path pointsAroundArc(Point start, Point end, Point circleCentre, double radius, double cutoffPercentage);
inline Path createPath(Point startingPoint, Point endingPoint, double approachHeading, double avoidanceCircleRadius);

#endif