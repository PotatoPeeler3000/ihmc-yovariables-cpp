#pragma once

#include "ihmc/yovariables/providers/double_provider.h"

namespace ihmc::yovariables::filters
{
/** Computes the alpha value for a given break-frequency provider, caching to avoid recomputing unless the break frequency changed. */
class AlphaBasedOnBreakFrequencyProvider : public providers::DoubleProvider
{
public:
   AlphaBasedOnBreakFrequencyProvider(providers::DoubleProvider& breakFrequencyProvider, double dt);

   double getValue() const override;

private:
   providers::DoubleProvider& breakFrequencyProvider_;
   double dt_;
   mutable double previousBreakFrequency_;
   mutable double alpha_ = 0.0;
};
}
