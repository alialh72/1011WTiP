#include "odom.h"
#include "path-smoother.h"
#include <vex.h>
#include <cmath>

inline bool enableOdom = true;
inline Path desiredPath;
inline Path finalPath;

int RunOdom();
void FillPointVals(Path path);
void PurePursuitController();