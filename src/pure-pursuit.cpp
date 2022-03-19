#include "pure-pursuit.h"

int RunOdom() {
  while (enableOdom) {
    fullOdomCycle();

    vex::task::sleep(20);
  }

  return 1;
}






void PurePursuitController(){
  enableOdom = true;
  vex::task runOdom(RunOdom);

}