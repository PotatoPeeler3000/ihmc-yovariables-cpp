#include "ihmc/yovariables/filters/deadbanded_yo_variable.h"

#include <stdexcept>

#include "ihmc/yovariables/filters/filter_math.h"

namespace ihmc::yovariables::filters
{
DeadbandedYoVariable::DeadbandedYoVariable(const std::string& name, providers::DoubleProvider& deadzoneSize, registry::YoRegistry* registry)
   : YoDouble(name, registry), deadzoneSize_(deadzoneSize)
{
}

DeadbandedYoVariable::DeadbandedYoVariable(const std::string& name, providers::DoubleProvider& inputVariable, providers::DoubleProvider& deadzoneSize,
                                            registry::YoRegistry* registry)
   : YoDouble(name, registry), deadzoneSize_(deadzoneSize), inputVariable_(&inputVariable)
{
}

void DeadbandedYoVariable::update()
{
   if (inputVariable_ == nullptr)
      throw std::logic_error("DeadbandedYoVariable must be constructed with a non-null input variable to call update().");
   update(inputVariable_->getValue());
}

void DeadbandedYoVariable::update(double valueToBeCorrected)
{
   YoDouble::set(applyDeadband(deadzoneSize_.getValue(), valueToBeCorrected));
}
}
