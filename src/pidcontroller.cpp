#include "vex.h"
#include <cmath>
#include "pidcontroller.h"

//========================ARM PID========================

int drivePID(){

  while(enableDrivePID) {

    //===============================================================
    //--------------Drive PID--------------//

    double currentVelocity = ((TWLeft.velocity(dps) + TWRight.velocity(dps) + TWHorizontal.velocity(dps)) / 3 ) * ;

    driveVals.updatePIDVals(FBLift.position(degrees), false); //pass the fb lift motor posiiton

    double drivePower = driveVals.calculatePower(); 
    //===============================================================
        
    Clamp.spin(forward, drivePower, voltageUnits::volt);

    vex::task::sleep(20);
  }

  return 1;
}