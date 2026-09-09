#include "ihmc/yovariables/filters/acceleration_limited_yo_variable.h"

#include <cmath>
#include <stdexcept>

#include "ihmc/yovariables/filters/filter_math.h"

namespace ihmc::yovariables::filters
{
AccelerationLimitedYoVariable::AccelerationLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider* maxRate,
                                                               providers::DoubleProvider* maxAcceleration, double dt)
   : AccelerationLimitedYoVariable(name, registry, maxRate, maxAcceleration, nullptr, dt)
{
}

AccelerationLimitedYoVariable::AccelerationLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider* maxRate,
                                                               providers::DoubleProvider* maxAcceleration, providers::DoubleProvider* inputVariable,
                                                               double dt)
   : YoDouble(name, registry), dt_(dt), hasBeenInitialized_(name + "HasBeenInitialized", registry), smoothedRate_(name + "SmoothedRate", registry),
     smoothedAcceleration_(name + "SmoothedAcceleration", registry), positionGain_(name + "PositionGain", registry),
     velocityGain_(name + "VelocityGain", registry), inputVariable_(inputVariable)
{
   if (maxRate != nullptr && maxAcceleration != nullptr)
   {
      maximumRate_ = maxRate;
      maximumAcceleration_ = maxAcceleration;
   }

   double w0 = 2.0 * M_PI * 16.0;
   double zeta = 1.0;

   setGainsByPolePlacement(w0, zeta);
   hasBeenInitialized_.set(false);
}

void AccelerationLimitedYoVariable::setGainsByPolePlacement(double w0, double zeta)
{
   positionGain_.set(w0 * w0);
   velocityGain_.set(2.0 * zeta * w0);
}

variable::YoDouble& AccelerationLimitedYoVariable::getPositionGain()
{
   return positionGain_;
}

variable::YoDouble& AccelerationLimitedYoVariable::getVelocityGain()
{
   return velocityGain_;
}

void AccelerationLimitedYoVariable::update()
{
   if (inputVariable_ == nullptr)
      throw std::logic_error("AccelerationLimitedYoVariable must be constructed with a non-null input variable to call update().");
   update(inputVariable_->getValue());
}

void AccelerationLimitedYoVariable::update(double input)
{
   if (!hasBeenInitialized_.getBooleanValue())
      initialize(input);

   double positionError = input - getDoubleValue();
   double acceleration = -velocityGain_.getDoubleValue() * smoothedRate_.getDoubleValue() + positionGain_.getDoubleValue() * positionError;
   acceleration = clampSymmetric(acceleration, maximumAcceleration_->getValue());

   smoothedAcceleration_.set(acceleration);
   smoothedRate_.add(smoothedAcceleration_.getDoubleValue() * dt_);
   smoothedRate_.set(clampSymmetric(smoothedRate_.getDoubleValue(), maximumRate_->getValue()));
   add(smoothedRate_.getDoubleValue() * dt_);
}

void AccelerationLimitedYoVariable::initialize(double input)
{
   set(input);
   smoothedRate_.set(0.0);
   smoothedAcceleration_.set(0.0);
   hasBeenInitialized_.set(true);
}

void AccelerationLimitedYoVariable::reset()
{
   hasBeenInitialized_.set(false);
   smoothedRate_.set(0.0);
   smoothedAcceleration_.set(0.0);
}

variable::YoDouble& AccelerationLimitedYoVariable::getSmoothedRate()
{
   return smoothedRate_;
}

variable::YoDouble& AccelerationLimitedYoVariable::getSmoothedAcceleration()
{
   return smoothedAcceleration_;
}

bool AccelerationLimitedYoVariable::hasBeenInitialized() const
{
   return hasBeenInitialized_.getBooleanValue();
}

double AccelerationLimitedYoVariable::getMaximumRate() const
{
   return maximumRate_->getValue();
}

double AccelerationLimitedYoVariable::getMaximumAcceleration() const
{
   return maximumAcceleration_->getValue();
}
}
