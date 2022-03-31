#ifndef POINT-STRUCT_H
#define POINT-STRUCT_H

#include <vector>
#include <cmath>
#include <deque>

/*

..Units..

Lengths : Inches
Angles : Radians

*/

struct Point {
  double x, y;

  double distanceFromStart;
  double curvature;

  double globalHeading;

  double maximumVelocity;
  double targetVelocity;

  void setCoordinates (std::vector<double> points) {
    x = points.at(0);
    y = points.at(1);
  }

  void setDistance (double distanceFromStart){
    this -> distanceFromStart = distanceFromStart;
  }

  void setCurvature (double curvature){
    this -> curvature = curvature;
  }

  void setMaxVelocity (double maximumVelocity){
    this -> maximumVelocity = maximumVelocity;
  }

  void setTargetVelocity (double targetVelocity){
    this -> targetVelocity = targetVelocity;
  }

  Point(std::vector<double> point){
    x = point.at(0);
    y = point.at(1);
  }

  Point() {
    //empty constructor
  }

};

//return the magnitude of a vector: c^2 = a^2 + b^2
inline double getMagnitude(Point point) {
  return sqrt( pow(point.x, 2) + pow(point.y, 2) );
}


//calculates the distance between 2 points
inline double getDistance(Point p0, Point p1) {
  return getMagnitude(Point({p1.x - p0.x, p1.y - p0.y}));
}

#endif