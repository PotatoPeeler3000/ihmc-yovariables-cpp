#pragma once

#include <memory>
#include <vector>

#include "ihmc/yovariables/variable/yo_boolean.h"

namespace ihmc::yovariables::filters
{
/** Introduces a fixed delay, in ticks, between variableToDelay and this variable's value. */
class DelayedYoBoolean : public variable::YoBoolean
{
public:
   DelayedYoBoolean(const std::string& name, const std::string& description, variable::YoBoolean& variableToDelay, int ticksToDelay,
                     registry::YoRegistry* registry);

   void update();
   void reset();

private:
   variable::YoBoolean& variableToDelay_;
   std::vector<std::unique_ptr<variable::YoBoolean>> previousYoVariables_;
};
}
