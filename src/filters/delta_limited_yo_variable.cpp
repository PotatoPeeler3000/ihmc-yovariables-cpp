#include "ihmc/yovariables/filters/delta_limited_yo_variable.h"

#include <cmath>

namespace ihmc::yovariables::filters
{
DeltaLimitedYoVariable::DeltaLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, double maxDelta)
   : YoDouble(name, registry), maxDelta_(name + "MaxAllowedDelta", registry), actual_(name + "Actual", registry), desired_(name + "Desired", registry),
     isLimitingActive_(name + "IsLimitingActive", registry)
{
   maxDelta_.set(std::abs(maxDelta));
   isLimitingActive_.set(false);
}

void DeltaLimitedYoVariable::setMaxDelta(double maxDelta)
{
   maxDelta_.set(std::abs(maxDelta));
}

void DeltaLimitedYoVariable::updateOutput(double actual, double desired)
{
   desired_.set(desired);
   actual_.set(actual);
   updateOutput();
}

bool DeltaLimitedYoVariable::isLimitingActive() const
{
   return isLimitingActive_.getBooleanValue();
}

void DeltaLimitedYoVariable::updateOutput()
{
   double actualDoubleValue = actual_.getDoubleValue();
   double desiredDoubleValue = desired_.getDoubleValue();
   double maxDeltaDoubleValue = std::abs(maxDelta_.getDoubleValue());
   double rawDelta = actualDoubleValue - desiredDoubleValue;
   double sign = rawDelta > 0.0 ? 1.0 : (rawDelta < 0.0 ? -1.0 : 0.0);
   double requestedDelta = std::abs(rawDelta);
   double overshoot = maxDeltaDoubleValue - requestedDelta;

   if (overshoot < 0)
   {
      desiredDoubleValue = actualDoubleValue - maxDeltaDoubleValue * sign;
      isLimitingActive_.set(true);
   }
   else
   {
      isLimitingActive_.set(false);
   }

   set(desiredDoubleValue);
}
}
