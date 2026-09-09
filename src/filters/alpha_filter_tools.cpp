#include "ihmc/yovariables/filters/alpha_filter_tools.h"

#include <cmath>

#include "ihmc/yovariables/filters/filter_math.h"

namespace ihmc::yovariables::filters
{
double computeAlphaGivenBreakFrequencyProperly(double breakFrequencyInHertz, double dt)
{
   if (std::isinf(breakFrequencyInHertz))
      return 0.0;

   double omega = kTwoPi * breakFrequencyInHertz;
   double alpha = (1.0 - omega * dt / 2.0) / (1.0 + omega * dt / 2.0);
   return clamp(alpha, 0.0, 1.0);
}

double computeBreakFrequencyGivenAlpha(double alpha, double dt)
{
   return (1.0 - alpha) / (M_PI * dt * (1.0 + alpha));
}
}
