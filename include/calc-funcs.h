#ifndef CALC-FUNCS_H
#define CALC-FUNCS_H

#include "structs/path-struct.h"
#include "vex.h"
#include <vector>
#include <cmath>
#include <deque>

//gets the curvature of a point (p1)
inline double calcCurvature(Point p1, Point p2, Point p3) {

  //find radius
  double k1 = 0.5*( pow(p1.x,2) + pow(p1.y, 2) - pow(p2.x,2) - pow(p2.y,2)   ) /(p1.x - p2.x +0.0001);
  double k2=(p1.y - p2.y)/(p1.x - p2.x);

  double b= 0.5*( pow(p2.x,2) - 2*p2.x *k1 + pow(p2.y,2) - pow(p3.x,2) +2* p3.x *k1 - pow(p3.y,2))/(p3.x *k2 -p3.y +p2.y -p2.x *k2);
  double a=k1 - k2 *b;

  double r = sqrt( pow((p1.x - a),2) +pow((p1.y - b),2) );

  //curvature = 1/radius
  double curvature = 1/r;

  return curvature;
}

//checks if a line segment intersects with circle returns t value [0,1] 
inline double calcIntersection(Point startSegment, Point endSegment, Point robotCentre, double radius) {
  double t1, t2;
  Vector d(startSegment, endSegment);
  Vector f(robotCentre, startSegment);

  double a = d.Dot(d);
  double b = 2 * f.Dot(d);
  double c = f.Dot(f) - radius * radius;
  double discriminant = (b * b) - (4 * a * c);

  if (discriminant < 0) {
    return -1; // no intersection
  }
  else {
    discriminant = sqrt(discriminant);
    t1 = (-b - discriminant) / (2 * a);
    t2 = (-b + discriminant) / (2 * a);

    if (t1 >= 0 && t1 <= 1 && t1 >= t2) {
        //return t1 intersection
        return t1;
    }
    if (t2 >= 0 && t2 <= 1 && t2 > t1) {
        //return t2
        return t2;
    }
    return -1; //return no intersection
  }

}

//Calculate tVal + index
inline double calcFractionalT(Path path, Point robotPos, double lookahead, double startingLargest) {

  double largestTVal = startingLargest;
  //loop through path up until last point
  for (int i = 0; i < path.points.size()-1; i++) {
    //start and end points of vector
    Point startPoint = path.getPoint(i);
    Point endPoint = path.getPoint(i + 1);


    double currentTVal = calcIntersection(startPoint, endPoint, robotPos, lookahead);

    if (currentTVal >= 0 && currentTVal <= 1)  {
      if (currentTVal + i >= largestTVal) {
        largestTVal = currentTVal + i;
      }
    }
      
  }

  return largestTVal;
}

//clamp function
inline double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}


inline double getInertialReading() {
  return ( (InertialRight.rotation() + InertialLeft.rotation())/2 );
}
#endif