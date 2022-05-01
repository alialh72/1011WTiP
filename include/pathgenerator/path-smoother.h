//Credits to Ali Al Hamadani
//Algo taken from github: https://github.com/alialh72/SmoothPathGenerator/



#ifndef PATHSMOOTHER_H
#define PATHSMOOTHER_H

#include <vector>
#include <vex.h>
#include <cmath>
#include <deque>
#include "structs/path-struct.h"

inline Segment CalcCoefficients(double alpha, double tension, Point p0, Point p1, Point p2, Point p3){
  
    
  //Catmull-Rom Spline calculations
  double t01 = pow(getDistance(p0, p1), alpha);
  double t12 = pow(getDistance(p1, p2), alpha);
  double t23 = pow(getDistance(p2, p3), alpha);
  
  std::vector<double> m1 = {};
  std::vector<double> m2 = {};
  
  //m1:
  //x:
  double m1x = (1 - tension) *
      (p2.x - p1.x + t12 * ((p1.x - p0.x) / t01 - (p2.x - p0.x) / (t01 + t12)));
  //y:
  double m1y = (1 - tension) *
      (p2.y - p1.y + t12 * ((p1.y - p0.y) / t01 - (p2.y - p0.y) / (t01 + t12)));
  
  m1.push_back(m1x);
  m1.push_back(m1y);
  
  
  //m2:
  //x:
  double m2x = (1. - tension) *
  (p2.x - p1.x + t12 * ((p3.x - p2.x) / t23 - (p3.x - p1.x) / (t12 + t23)));
  
  //y:
  double m2y = (1. - tension) *
  (p2.y - p1.y + t12 * ((p3.y - p2.y) / t23 - (p3.y - p1.y) / (t12 + t23)));
  
  m2.push_back(m2x);
  m2.push_back(m2y);
  
  Segment segment;
  
  //Calculate segment coeffiecents using precalculated values
  segment.a = {2 * (p1.x - p2.x) + m1.at(0) + m2.at(0)  ,   2 * (p1.y - p2.y) + m1.at(1) + m2.at(1)};
  segment.b = {(-3) * (p1.x - p2.x) - m1.at(0) - m1.at(0) - m2.at(0)  ,  (-3) * (p1.y - p2.y) - m1.at(1) - m1.at(1) - m2.at(1) };
  segment.c = {m1.at(0) , m1.at(1)};
  segment.d = {p1.x, p1.y};
  
  //point in spline = at^3 + bt^2 + ct + d
  //t is a point in the spline segment [0,1]
  
  return segment;
}



inline Path GenerateSmoothPath(Path path){
    
  //Inject a starting control point and ending control point
  //--> This is because splines need 2 control points to be generated
  //These control points are colinear with the 2 closest points
  
  //inject first control point
  Point firstPoint = path.getPoint(0);
  
  double diff_x1 = path.getPoint(1).x - firstPoint.x;
  double diff_y1 = path.getPoint(1).y - firstPoint.y;
  
  Point startingControl({firstPoint.x - diff_x1, firstPoint.y - diff_y1});
  
  path.points.push_front(startingControl);
  
  
  //inject last control point
  Point lastPoint = path.getPoint(path.points.size()-1);
  
  double diff_x2 = lastPoint.x - path.getPoint(path.points.size()-2).x;
  double diff_y2 = lastPoint.y - path.getPoint(path.points.size()-2).y;
  Point finalControl({lastPoint.x + diff_x2, lastPoint.y + diff_y2});
  
  path.points.push_back(finalControl);
  
  
  //Smooth Path
  std::vector<Segment> segments;
  
  for (int i = 0; i < path.points.size()-3; i++){
    //Generate segment coefficients
    Segment segment = CalcCoefficients(0.75, 0, path.getPoint(i), path.getPoint(i+1), path.getPoint(i+2), path.getPoint(i+3));
    
    //add it to vector containing all segment coefficients
    segments.push_back(segment);
  }
  
  
  Path smoothedPath;

  //Loop through different segments
  for (int z = 0; z < segments.size(); z++) {
    
    Segment currentSegment = segments.at(z);
    
    //point in spline = at^3 + bt^2 + ct + d
    //t: [0,1], where 0 is the first point of the segment and 1 is the last
    //a, b, c, d were calculated above for each spline segment
    
    //add the first point of the first segment to the smoothedPath
    if(z == 0) {
      double xVal = segments.at(0).d.at(0);
      double yVal =  segments.at(0).d.at(1);
      smoothedPath.addPointVector({xVal, yVal});
    }
    

    //generate an even number of points in each segment
    //t: [0, 1]
    //t increment: 0.1 => 10 points will be generated for each spine
    
    for (double t = 0.25; t <= 1; t += 0.25) {
        
      //point in spline = at^3 + bt^2 + ct + d
      double xVal = (currentSegment.a.at(0) * pow(t,3)) + (currentSegment.b.at(0) * pow(t,2)) + (currentSegment.c.at(0) * t) + currentSegment.d.at(0);
      
      double yVal = (currentSegment.a.at(1) * pow(t,3)) + (currentSegment.b.at(1) * pow(t,2)) + (currentSegment.c.at(1) * t) + currentSegment.d.at(1);
      
      //add generated point to smoothedPath
      smoothedPath.addPointVector({xVal, yVal});
    }
    

  }
  
  return smoothedPath;
    
}

#endif