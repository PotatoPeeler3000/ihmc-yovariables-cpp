#pragma once

#include <optional>

#include "ihmc/yovariables/filters/filter_math.h"
#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
/**
 * Tracks a reference value, but limits the maximum rate of change to maxRateVariable * dt per
 * update.
 */
class RateLimitedYoVariable : public variable::YoDouble
{
public:
   RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, double maxRate, double dt);
   RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& maxRateVariable, double dt);
   RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, double maxRate, providers::DoubleProvider* positionVariable, double dt);
   RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& maxRateVariable,
                          providers::DoubleProvider* unlimitedPosition, double dt);
   RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& maxRateVariable,
                          providers::DoubleProvider* unlimitedPosition, providers::DoubleProvider& dt);

   void reset();

   /** @throws std::logic_error if constructed without a position variable to track. */
   void update();
   void update(double currentPosition);

private:
   providers::DoubleProvider* maxRateVariable_ = nullptr;

   providers::DoubleProvider* unlimitedPosition_ = nullptr;
   variable::YoBoolean limited_;

   providers::DoubleProvider* dt_ = nullptr;

   variable::YoBoolean hasBeenCalled_;

   std::optional<variable::YoDouble> internalMaxRate_;
   std::optional<ConstantDoubleProvider> internalDt_;
};
}
