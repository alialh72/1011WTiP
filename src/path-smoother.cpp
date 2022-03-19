#include "path-smoother.h"

Segment CalcCoefficients(double alpha, double tension, std::vector<double> p0, std::vector<double> p1, std::vector<double> p2, std::vector<double> p3){
    
    //Catmull-Rom Spline calculations
    double t01 = pow(getDistance(p0, p1), alpha);
    double t12 = pow(getDistance(p1, p2), alpha);
    double t23 = pow(getDistance(p2, p3), alpha);
    
    std::vector<double> m1 = {};
    std::vector<double> m2 = {};
    
    //m1:
    //x:
    double m1x = (1 - tension) *
        (p2.at(0) - p1.at(0) + t12 * ((p1.at(0) - p0.at(0)) / t01 - (p2.at(0) - p0.at(0)) / (t01 + t12)));
    //y:
    double m1y = (1 - tension) *
        (p2.at(1) - p1.at(1) + t12 * ((p1.at(1) - p0.at(1)) / t01 - (p2.at(1) - p0.at(1)) / (t01 + t12)));
    
    m1.push_back(m1x);
    m1.push_back(m1y);
    
    
    //m2:
    //x:
    double m2x = (1. - tension) *
    (p2.at(0) - p1.at(0) + t12 * ((p3.at(0) - p2.at(0)) / t23 - (p3.at(0) - p1.at(0)) / (t12 + t23)));
    
    //y:
    double m2y = (1. - tension) *
    (p2.at(1) - p1.at(1) + t12 * ((p3.at(1) - p2.at(1)) / t23 - (p3.at(1) - p1.at(1)) / (t12 + t23)));
    
    m2.push_back(m2x);
    m2.push_back(m2y);
    
    Segment segment;
    
    //Calculate segment coeffiecents using precalculated values
    segment.a = {2 * (p1.at(0) - p2.at(0)) + m1.at(0) + m2.at(0)  ,   2 * (p1.at(1) - p2.at(1)) + m1.at(1) + m2.at(1)};
    segment.b = {(-3) * (p1.at(0) - p2.at(0)) - m1.at(0) - m1.at(0) - m2.at(0)  ,  (-3) * (p1.at(1) - p2.at(1)) - m1.at(1) - m1.at(1) - m2.at(1) };
    segment.c = {m1.at(0) , m1.at(1)};
    segment.d = {p1.at(0), p1.at(1)};
    
    //point in spline = at^3 + bt^2 + ct + d
    //t is a point in the spline segment [0,1]
    
    return segment;
}




Path GenerateSmoothPath(Path path){
    
    //Inject a starting control point and ending control point
    //--> This is because splines need 2 control points to be generated
    //These control points are colinear with the 2 closest points
    
    //inject first control point
    std::vector<double> firstPoint = path.getPoint(0);
    
    double diff_x1 = path.getPoint(1).at(0) - firstPoint.at(0);
    double diff_y1 = path.getPoint(1).at(1) - firstPoint.at(1);
    
    std::vector<double> startingControl = {firstPoint.at(0) - diff_x1, firstPoint.at(1) - diff_y1};
    
    path.points.push_front(startingControl);
    
    
    //inject last control point
    std::vector<double> lastPoint = path.getPoint(path.points.size()-1);
    
    double diff_x2 = lastPoint.at(0) - path.getPoint(path.points.size()-2).at(0);
    double diff_y2 = lastPoint.at(1) - path.getPoint(path.points.size()-2).at(1);
    std::vector<double> finalControl = {lastPoint.at(0) + diff_x2, lastPoint.at(1) + diff_y2};
    
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
            smoothedPath.addPoint({xVal, yVal});
        }
        

        //generate an even number of points in each segment
        //t: [0, 1]
        //t increment: 0.1 => 10 points will be generated for each spine
        
        for (double t = 0.1; t <= 1; t += 0.1) {
            
            //point in spline = at^3 + bt^2 + ct + d
            double xVal = (currentSegment.a.at(0) * pow(t,3)) + (currentSegment.b.at(0) * pow(t,2)) + (currentSegment.c.at(0) * t) + currentSegment.d.at(0);
            
            double yVal = (currentSegment.a.at(1) * pow(t,3)) + (currentSegment.b.at(1) * pow(t,2)) + (currentSegment.c.at(1) * t) + currentSegment.d.at(1);
            
            //add generated point to smoothedPath
            smoothedPath.addPoint({xVal, yVal});
        }
        

    }
    
    return smoothedPath;
    
}

