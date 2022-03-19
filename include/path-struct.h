#include <vector>
#include <cmath>
#include <deque>

struct Path {
  //deque is used here instead of vector as it makes it easy to push points to the front
  //point --> a vector of [x,y]
  std::deque<std::vector<double>> points;

  //returns a point
  std::vector<double> getPoint(double t) {
      return points.at(t);
  }
  
  //adds a point
  void addPoint(std::vector<double> point){
      points.push_back(point);
  }
};


struct Segment {
    std::vector<double> a, b, c, d;
};



//return the magnitude of a vector: c^2 = a^2 + b^2
inline double getMagnitude(std::vector<double> point) {
    return sqrt( pow(point.at(0), 2) + pow(point.at(1), 2) );
}



//calculates the distance between 2 vectors
inline double getDistance(std::vector<double> p0, std::vector<double> p1) {
    return getMagnitude({p1.at(0) - p0.at(0), p1.at(1) - p1.at(1)});
}