#include "ihmc/yovariables/filters/alpha_filtered_yo_variable.h"

#include <stdexcept>

#include "ihmc/yovariables/filters/filter_math.h"
#include "ihmc/yovariables/filters/variable_tools.h"

namespace ihmc::yovariables::filters
{
AlphaFilteredYoVariable::AlphaFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, double alpha,
                                                  variable::YoDouble* positionVariable)
   : YoDouble(name, registry), hasBeenCalled(hasBeenCalledName(name, ""), registry), position_(positionVariable)
{
   internalAlpha_.emplace(alphaVariableName(name, ""), registry);
   internalAlpha_->set(alpha);
   alphaVariable_ = &*internalAlpha_;
   reset();
}

AlphaFilteredYoVariable::AlphaFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& alphaVariable,
                                                  variable::YoDouble* positionVariable)
   : AlphaFilteredYoVariable(name, "", registry, alphaVariable, positionVariable)
{
}

AlphaFilteredYoVariable::AlphaFilteredYoVariable(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                                                  providers::DoubleProvider& alphaVariable, variable::YoDouble* positionVariable)
   : YoDouble(name, description, registry), hasBeenCalled(hasBeenCalledName(name, ""), registry), alphaVariable_(&alphaVariable),
     position_(positionVariable)
{
   reset();
}

void AlphaFilteredYoVariable::reset()
{
   hasBeenCalled.set(false);
}

void AlphaFilteredYoVariable::update()
{
   if (position_ == nullptr)
      throw std::logic_error("AlphaFilteredYoVariable must be constructed with a non-null position variable to call update(); use update(double) instead.");
   update(position_->getDoubleValue());
}

void AlphaFilteredYoVariable::update(double currentPosition)
{
   if (!hasBeenCalled.getBooleanValue())
   {
      hasBeenCalled.set(true);
      set(currentPosition);
   }
   else
   {
      double alpha = alphaVariable_->getValue();
      set(interpolate(currentPosition, getDoubleValue(), alpha));
   }
}

bool AlphaFilteredYoVariable::getHasBeenCalled() const
{
   return hasBeenCalled.getBooleanValue();
}
}
