#pragma once

#include <memory>
#include <vector>

#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
/** Introduces a fixed delay, in ticks, between variableToDelay and this variable's value. */
class DelayedYoDouble : public variable::YoDouble
{
public:
   DelayedYoDouble(const std::string& name, const std::string& description, providers::DoubleProvider& variableToDelay, int ticksToDelay,
                    registry::YoRegistry* registry);

   void update();
   void reset();

private:
   providers::DoubleProvider& variableToDelay_;
   std::vector<std::unique_ptr<variable::YoDouble>> previousYoDouble_;
};
}
