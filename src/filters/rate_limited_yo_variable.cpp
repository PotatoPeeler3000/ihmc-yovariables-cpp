#include "ihmc/yovariables/filters/rate_limited_yo_variable.h"

#include <cmath>
#include <stdexcept>

#include "ihmc/yovariables/filters/variable_tools.h"

namespace ihmc::yovariables::filters
{
namespace
{
std::string limitedName(const std::string& name)
{
   return limitedCalledName(name, "");
}
} // namespace

RateLimitedYoVariable::RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, double maxRate, double dt)
   : YoDouble(name, registry), limited_(limitedName(name), registry), hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   internalMaxRate_.emplace(maxRateName(name, ""), registry);
   internalMaxRate_->set(maxRate);
   maxRateVariable_ = &*internalMaxRate_;

   internalDt_.emplace(dt);
   dt_ = &*internalDt_;

   reset();
}

RateLimitedYoVariable::RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& maxRateVariable, double dt)
   : YoDouble(name, registry), maxRateVariable_(&maxRateVariable), limited_(limitedName(name), registry), hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   internalDt_.emplace(dt);
   dt_ = &*internalDt_;

   reset();
}

RateLimitedYoVariable::RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, double maxRate,
                                              providers::DoubleProvider* positionVariable, double dt)
   : YoDouble(name, registry), unlimitedPosition_(positionVariable), limited_(limitedName(name), registry),
     hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   internalMaxRate_.emplace(maxRateName(name, ""), registry);
   internalMaxRate_->set(maxRate);
   maxRateVariable_ = &*internalMaxRate_;

   internalDt_.emplace(dt);
   dt_ = &*internalDt_;

   reset();
}

RateLimitedYoVariable::RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& maxRateVariable,
                                              providers::DoubleProvider* unlimitedPosition, double dt)
   : YoDouble(name, registry), maxRateVariable_(&maxRateVariable), unlimitedPosition_(unlimitedPosition), limited_(limitedName(name), registry),
     hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   internalDt_.emplace(dt);
   dt_ = &*internalDt_;

   reset();
}

RateLimitedYoVariable::RateLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& maxRateVariable,
                                              providers::DoubleProvider* unlimitedPosition, providers::DoubleProvider& dt)
   : YoDouble(name, registry), maxRateVariable_(&maxRateVariable), unlimitedPosition_(unlimitedPosition), limited_(limitedName(name), registry), dt_(&dt),
     hasBeenCalled_(hasBeenCalledName(name, ""), registry)
{
   reset();
}

void RateLimitedYoVariable::reset()
{
   hasBeenCalled_.set(false);
}

void RateLimitedYoVariable::update()
{
   if (unlimitedPosition_ == nullptr)
      throw std::logic_error("RateLimitedYoVariable must be constructed with a non-null position variable to call update().");
   update(unlimitedPosition_->getValue());
}

void RateLimitedYoVariable::update(double currentPosition)
{
   if (!hasBeenCalled_.getBooleanValue())
   {
      hasBeenCalled_.set(true);
      set(currentPosition);
   }

   if (maxRateVariable_->getValue() < 0)
      throw std::runtime_error("The maxRate parameter in RateLimitedYoVariable cannot be negative.");

   double difference = currentPosition - getDoubleValue();
   if (std::abs(difference) > maxRateVariable_->getValue() * dt_->getValue())
   {
      double sign = difference > 0.0 ? 1.0 : (difference < 0.0 ? -1.0 : 0.0);
      difference = sign * maxRateVariable_->getValue() * dt_->getValue();
      limited_.set(true);
   }
   else
   {
      limited_.set(false);
   }

   set(getDoubleValue() + difference);
}
}
