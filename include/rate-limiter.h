#ifndef RATELIMITER_H
#define RATELIMITER_H
#include "calc-funcs.h"

class limiter{
  public: 
    double rateLimiterOutput = 0, prevRateLimiterOutput = 0;
    
    double rateLimiter(double val, double maxRate) {
      double maxChange = 0.02 * maxRate;
      rateLimiterOutput += clamp(val - prevRateLimiterOutput, -maxChange, maxChange);
      prevRateLimiterOutput = rateLimiterOutput;
      return rateLimiterOutput;
    }

    void reset() {
      rateLimiterOutput = 0;
      prevRateLimiterOutput = 0;
    }
};

#endif