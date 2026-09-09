#include "ihmc/yovariables/filters/alpha_based_on_break_frequency_provider.h"

#include <limits>

#include "ihmc/yovariables/filters/alpha_filter_tools.h"

namespace ihmc::yovariables::filters
{
AlphaBasedOnBreakFrequencyProvider::AlphaBasedOnBreakFrequencyProvider(providers::DoubleProvider& breakFrequencyProvider, double dt)
   : breakFrequencyProvider_(breakFrequencyProvider), dt_(dt), previousBreakFrequency_(std::numeric_limits<double>::quiet_NaN())
{
}

double AlphaBasedOnBreakFrequencyProvider::getValue() const
{
   double currentBreakFrequency = breakFrequencyProvider_.getValue();
   if (currentBreakFrequency != previousBreakFrequency_)
   {
      alpha_ = computeAlphaGivenBreakFrequencyProperly(currentBreakFrequency, dt_);
      previousBreakFrequency_ = currentBreakFrequency;
   }
   return alpha_;
}
}
