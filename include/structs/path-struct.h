#ifndef PATH-STRUCT_H
#define PATH-STRUCT_H

#include <vector>
#include <cmath>
#include <deque>
#include "point-struct.h"

struct Vector {
  double magnitude;
  double thetaHeading;
  double refAngle;
  int quadrant;
  Point startPoint, endPoint;

  double Dot(Vector b) {
    return magnitude * b.magnitude * cos(std::abs(b.thetaHeading - thetaHeading));
  }

  Vector(Point p1, Point p2) {
    //__Start & End__
    startPoint = p1;
    endPoint = p2;

    //__Vector magnitude
    magnitude = getDistance(p1, p2);

    //__Determine quadrant__

    //quadrant 1:  x2 > 1 & y2 > y1    
    if (p2.x > p1.x && p2.y > p1.y) {
      double insideAngle = asin(std::abs(p2.x - p1.x) / magnitude);
      quadrant = 1;
      refAngle = M_PI/2 - insideAngle;
      thetaHeading = insideAngle;
    }

    //quadrant 2:  x2 > x1 & y1 > y2    
    else if (p2.x > p1.x && p1.y > p2.y) {
      double insideAngle = asin(std::abs(p1.y - p2.y) / magnitude);
      quadrant = 2;
      refAngle = insideAngle;
      thetaHeading = insideAngle + (M_PI / 2);
    }

    //quadrant 3:  x1 > x2 & y1 > y2    
    else if (p1.x > p2.x && p1.y > p2.y) {
      double insideAngle = asin(std::abs(p2.y - p1.y) / magnitude);
      quadrant = 3;
      refAngle = insideAngle;
      thetaHeading = (3 * M_PI / 2) - insideAngle;
    }

    //quadrant 4:  x1 > x2 & y2 > y1    
    else if (p1.x > p2.x && p2.y > p1.y) {
      double insideAngle = asin(std::abs(p2.x - p1.x) / magnitude);
      quadrant = 4;
      refAngle = M_PI/2 - insideAngle;
      thetaHeading = (2 * M_PI) - insideAngle;
    }

    //quadrant 5: straight up y2 > y1 && x2 == x1
    else if (p1.x == p2.x && p2.y > p1.y) {
      double insideAngle = 0;
      quadrant = 5;
      refAngle = M_PI / 2;
      thetaHeading = insideAngle;
    }
    
    //quadrant 6: straight right y2 == y1 && x2 > x1
    else if (p2.x > p1.x && p2.y == p1.y) {
      double insideAngle = M_PI / 2;
      quadrant = 6;
      refAngle = 0;
      thetaHeading = insideAngle;
    }

    //quadrant 7: straight down y1 > y2 && x2 = x1
    else if (p1.x == p2.x && p1.y > p2.y) {
      double insideAngle = M_PI;
      quadrant = 7;
      refAngle = M_PI/2;
      thetaHeading = insideAngle;
    }

    //quadrant 8: straight left x1 > x2 && y2 = y1
    else if (p1.x > p2.x && p2.y == p1.y) {
      double insideAngle = 3 * M_PI / 2;
      quadrant = 8;
      refAngle = 0;
      thetaHeading = insideAngle;
    }

    else {
      thetaHeading = 0;
    }


  }

};


struct Path {
  //deque is used here instead of vector as it makes it easy to push points to the front
  //point --> a vector of [x,y]
  std::deque<Point> points;

  //returns a point
  Point getPoint(double t) {
    return points.at(t);
  }
  
  //adds a point
  void addPoint(Point point){
    points.push_back(point);
  }

  void addPointVector(std::vector<double> vPoint){
    Point point(vPoint);
    points.push_back(point);
  }
};


struct Segment {
  std::vector<double> a, b, c, d;
};

#endif