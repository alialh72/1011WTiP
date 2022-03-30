#include "odom.h"
#include "path-smoother.h"
#include <vex.h>
#include <cmath>
#include <algorithm>

inline bool enableOdom = true;
inline bool enableFollowPath = true;
inline Path desiredPath;
inline Path finalPath;

//------PARAMETERERS-------
inline double maxPathVelocity = 20; //inches per second
inline double kMaxVel = 3; //[1,5] higher k --> faster around turns 
inline double lookaheadDistance = 10;

inline double maxAcceleration = 5; //incher per second^2


//------PURE PURSUIT LOOP VARIABLES-------
inline double prevClosestPointIndex = 0;
inline double closestPointIndex;


int RunOdom();
Path FillPointVals(Path path);
int FollowPath();
void PurePursuitController();