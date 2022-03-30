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

//return the magnitude of a vector: c^2 = a^2 + b^2
inline double getMagnitude(Point point) {
  return sqrt( pow(point.x, 2) + pow(point.y, 2) );
}


//calculates the distance between 2 vectors
inline double getDistance(Point p0, Point p1) {
  return getMagnitude(Point({p1.x - p0.x, p1.y - p1.y}));
}

struct Vector {
  double magnitude;
  double thetaHeading;
  int quadrant;

  double Dot(Vector b) {
    return magnitude * b.magnitude * cos(std::abs(b.thetaHeading - thetaHeading));
  }

  Vector(Point p1, Point p2) {
    //__Vector magnitude
    magnitude = getDistance(p1, p2);

    //__Determine quadrant__

    //quadrant 1:  x2 > 1 & y2 > y1    
    if (p2.x > p1.x && p2.y > p1.y) {
      double insideAngle = asin( std::abs(p2.x - p1.x) /  magnitude ); 
      quadrant = 1;
      thetaHeading = insideAngle;
    }

    //quadrant 2:  x2 > x1 & y1 > y2    
    else if (p2.x > p1.x && p1.y > p2.y) {
      double insideAngle = asin( std::abs(p1.y - p2.y) /  magnitude ); 
      quadrant = 2;
      thetaHeading = insideAngle + (M_PI/2);
    }

    //quadrant 3:  x1 > x2 & y1 > y2    
    else if (p1.x > p2.x && p1.y > p2.y) {
      double insideAngle = asin( std::abs(p2.y - p1.y) /  magnitude ); 
      quadrant = 3;
      thetaHeading = (3*M_PI/2) - insideAngle ;
    }

    //quadrant 4:  x1 > x2 & y2 > y1    
    else if (p1.x > p2.x && p2.y > p1.y) {
      double insideAngle = asin( std::abs(p2.x - p1.x) /  magnitude ); 
      quadrant = 4;
      thetaHeading = (2*M_PI) - insideAngle;
    }

    //quadrant 5: straight up y2 > y1 && x2 == x1
    else if (p1.x == p2.x && p2.y > p1.y) {
      double insideAngle = 0; 
      quadrant = 5;
      thetaHeading = insideAngle;
    }

    //quadrant 6: straight right y2 == y1 && x2 > x1
    else if (p2.x > p1.x && p2.y == p1.y) {
      double insideAngle = M_PI/2; 
      quadrant = 6;
      thetaHeading = insideAngle;
    }

    //quadrant 7: straight up y2 > y1 && x2 = x1
    else if (p1.x == p2.x && p1.y > p2.y) {
      double insideAngle = M_PI; 
      quadrant = 7;
      thetaHeading = insideAngle;
    }

    //quadrant 8: straight up y2 > y1 && x2 = x1
    else if (p1.x > p2.x && p2.y == p1.y) {
      double insideAngle = 3*M_PI/2; 
      quadrant = 8;
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

inline double calcIntersection(Point E, Point L, Point C, double radius) {
  double t1, t2;
  Vector d(E, L);
  Vector f(C, E);
 
  double a =  d.Dot(d);
  double b = 2*f.Dot(d);
  double c = f.Dot(f) - radius*radius; 
  double discriminant = (b*b)- (4*a*c);

  if (discriminant < 0) {
    return 0; // no intersection
  } 
  else{
    discriminant = sqrt(discriminant);
    t1 = (-b - discriminant)/(2*a);
    t2 = (-b + discriminant)/(2*a);

    if (t1 >= 0 && t1 <=1){ 
      //return t1 intersection
      return t1;
    }
    if (t2 >= 0 && t2 <=1){
      //return t2
      return t2;
    }
    return 0; //return no intersection
  }

}