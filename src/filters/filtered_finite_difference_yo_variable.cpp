#include "ihmc/yovariables/filters/filtered_finite_difference_yo_variable.h"

#include <stdexcept>

#include "ihmc/yovariables/filters/variable_tools.h"

namespace ihmc::yovariables::filters
{
FilteredFiniteDifferenceYoVariable::FilteredFiniteDifferenceYoVariable(const std::string& name, const std::string& description, double alpha, double dt,
                                                                        registry::YoRegistry* registry, variable::YoDouble* positionVariable)
   : YoDouble(name, description, registry), position_(positionVariable), lastPosition_(name + "_lastPosition", registry),
     hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   internalAlpha_.emplace(alphaVariableName(name, ""), registry);
   internalAlpha_->set(alpha);
   alphaVariable_ = &*internalAlpha_;

   internalDt_.emplace(dt);
   dt_ = &*internalDt_;

   reset();
}

FilteredFiniteDifferenceYoVariable::FilteredFiniteDifferenceYoVariable(const std::string& name, const std::string& description,
                                                                        providers::DoubleProvider& alphaVariable, double dt, registry::YoRegistry* registry,
                                                                        variable::YoDouble* positionVariable)
   : YoDouble(name, description, registry), alphaVariable_(&alphaVariable), position_(positionVariable), lastPosition_(name + "_lastPosition", registry),
     hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   internalDt_.emplace(dt);
   dt_ = &*internalDt_;

   reset();
}

FilteredFiniteDifferenceYoVariable::FilteredFiniteDifferenceYoVariable(const std::string& name, const std::string& description,
                                                                        providers::DoubleProvider& alphaVariable, providers::DoubleProvider& dt,
                                                                        registry::YoRegistry* registry, variable::YoDouble* positionVariable)
   : YoDouble(name, description, registry), alphaVariable_(&alphaVariable), dt_(&dt), position_(positionVariable),
     lastPosition_(name + "_lastPosition", registry), hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   reset();
}

void FilteredFiniteDifferenceYoVariable::reset()
{
   hasBeenCalled_.set(false);
}

void FilteredFiniteDifferenceYoVariable::update()
{
   if (position_ == nullptr)
      throw std::logic_error("FilteredFiniteDifferenceYoVariable must be constructed with a non-null position variable to call update().");
   update(position_->getValue());
}

void FilteredFiniteDifferenceYoVariable::updateForAngles()
{
   if (position_ == nullptr)
      throw std::logic_error("FilteredFiniteDifferenceYoVariable must be constructed with a non-null position variable to call updateForAngles().");
   updateForAngles(position_->getValue());
}

void FilteredFiniteDifferenceYoVariable::update(double currentPosition)
{
   if (!hasBeenCalled_.getBooleanValue())
   {
      hasBeenCalled_.set(true);
      lastPosition_.set(currentPosition);
      set(0.0);
   }

   double difference = currentPosition - lastPosition_.getDoubleValue();
   updateUsingDifference(difference);
   lastPosition_.set(currentPosition);
}

void FilteredFiniteDifferenceYoVariable::updateForAngles(double currentPosition)
{
   if (!hasBeenCalled_.getBooleanValue())
   {
      hasBeenCalled_.set(true);
      lastPosition_.set(currentPosition);
      set(0.0);
   }

   double difference = angleDifferenceMinusPiToPi(currentPosition, lastPosition_.getDoubleValue());
   updateUsingDifference(difference);
   lastPosition_.set(currentPosition);
}

void FilteredFiniteDifferenceYoVariable::updateUsingDifference(double difference)
{
   double previousFilteredDerivative = getDoubleValue();
   double currentRawDerivative = difference / dt_->getValue();

   double alpha = alphaVariable_->getValue();
   set(alpha * previousFilteredDerivative + (1.0 - alpha) * currentRawDerivative);
}
}
