#include "vex.h"
#include <cmath>

using namespace vex;

class PID {       
  public:     

    //Settings    
    double kP;  
    double kI;
    double kD;

    //Calculations
    int error; //SensorValue - DesiredValue : Position
    int prevError = 0; //Position 20ms ago
    int derivative; //error - prevError : Speed
    int totalError = 0;

    //Auto Control
    int desiredValue = 0;

    PID(double k[3]) {
      kP = k[0];
      kI = k[1];
      kD = k[2];
    }

    void changePID(double k[3]) {
      kP = k[0];
      kI = k[1];
      kD = k[2];
    }

    void updatePIDVals(double pos, bool flip) {

      //flip the subtraction --> used for distance sensor
      if (flip) {
        //Proportional
        error = pos - desiredValue;
      } else {
        //Proportional
        error = desiredValue - pos; 
      }
      

      //Derivative
      derivative = error - prevError;

      //Integral
      totalError += error;

      //Derivative --> set prevError
      prevError = error;
    }

    double calculatePower(){
      return (error * kP) + (totalError * kI) + (derivative * kD);
    }

};