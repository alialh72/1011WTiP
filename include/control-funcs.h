#include "vex.h"
#include "main.h"

//==============================
//Pistons
//==============================

void FrontClampOpen() { 
  //open clamp
  FrontClamp.set(true); 
} 

void FrontClampClose() { 
  //close clamp
  FrontClamp.set(false); 
} 

void BackTilterRetract() { 
  //set tilters to extended state
  BackTilter.set(false);
} 

void BackTilterExtend() {
  //set tilters to closed state
  BackTilter.set(true);
}

void BackClampClose() { 
  //set clamp to closed state
  BackClamp.set(false); 
} 

void BackClampOpen() { 
  //set clamp to open state
  BackClamp.set(true);
}

//==============================
//Booleans
//==============================

void swapRingOn() {
  ringOn = !ringOn;
}

void swapRingDirection() {
  ringDirection = !ringDirection;
}


//==============================
//Threads
//==============================

//Ring intake thread
int RingIntake() {
  while (1) {
    if (ringOn) {
      if (ringDirection) {
        Intake.spin(forward, 12, voltageUnits::volt);
      } else {
        Intake.spin(reverse, 12, voltageUnits::volt);
      }

    } else {
      Intake.stop(brakeType::hold);
    }

    vex::task::sleep(20);
  }
  return 1;
}