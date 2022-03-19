#include "odom.h"
#include "path-smoother.h"
#include <vex.h>
#include <cmath>

inline bool enableOdom = true;

int RunOdom();
void PurePursuitController();