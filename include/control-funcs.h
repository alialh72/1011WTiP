#ifndef CONTROLFUNCS_H
#define CONTROLFUNCS_H

#include "vex.h"

inline bool ringOn = false;
inline bool ringDirection = true; //true --> forward, false --> reverse
inline bool brakeTypeBool = true; //true --> hold, false --> coast

//==============================
//Pistons
//==============================

inline void FrontClampOpen() { 
  //open clamp
  FrontClamp.set(true); 
  wait(10, msec);
} 

inline void FrontClampClose() { 
  //close clamp
  FrontClamp.set(false); 
  wait(10, msec);

} 

inline void BackTilterRetract() { 
  //set tilters to extended state
  BackTilter.set(false);
  wait(10, msec);

} 

inline void BackTilterExtend() {
  //set tilters to closed state
  BackTilter.set(true);
  wait(10, msec);

}

inline void BackClampClose() { 
  //set clamp to closed state
  BackClamp.set(false); 
  wait(10, msec);

} 

inline void BackClampOpen() { 
  //set clamp to open state
  BackClamp.set(true);
  wait(10, msec);

}

//==============================
//Booleans
//==============================

inline void swapRingOn() {
  ringOn = !ringOn;
}

inline void swapRingDirection() {
  ringDirection = !ringDirection;
}


//==============================
//Threads
//==============================

//Ring intake thread
inline int RingIntake() {
  while (1) {
    if (ringOn) {
      if (ringDirection) {
        Intake.spin(forward, 250, velocityUnits::rpm);
      } else {
        Intake.spin(reverse, 250, velocityUnits::rpm);
      }

    } else {
      Intake.stop(brakeType::hold);
    }

    vex::task::sleep(20);
  }
  return 1;
}

#endif