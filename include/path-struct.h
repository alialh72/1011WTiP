#include <vector>
#include <cmath>
#include <deque>

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



//return the magnitude of a vector: c^2 = a^2 + b^2
inline double getMagnitude(Point point) {
  return sqrt( pow(point.x, 2) + pow(point.y, 2) );
}



//calculates the distance between 2 vectors
inline double getDistance(Point p0, Point p1) {
  return getMagnitude(Point({p1.x - p0.x, p1.y - p1.y}));
}

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