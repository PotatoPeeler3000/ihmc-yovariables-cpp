#include "ihmc/yovariables/filters/jerk_limited_yo_double.h"

#include <cmath>

#include "ihmc/yovariables/filters/filter_math.h"

namespace ihmc::yovariables::filters
{
JerkLimitedYoDouble::JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration,
                                          variable::YoDouble& maxJerk, double dt)
   : JerkLimitedYoDouble(name, registry, maxAcceleration, maxJerk, nullptr, nullptr, nullptr, dt)
{
}

JerkLimitedYoDouble::JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration,
                                          variable::YoDouble& maxJerk, variable::YoDouble* inputPosition, double dt)
   : JerkLimitedYoDouble(name, registry, maxAcceleration, maxJerk, inputPosition, nullptr, nullptr, dt)
{
}

JerkLimitedYoDouble::JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration,
                                          variable::YoDouble& maxJerk, variable::YoDouble* inputPosition, variable::YoDouble* inputVelocity, double dt)
   : JerkLimitedYoDouble(name, registry, maxAcceleration, maxJerk, inputPosition, inputVelocity, nullptr, dt)
{
}

JerkLimitedYoDouble::JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration,
                                          variable::YoDouble& maxJerk, variable::YoDouble* inputPosition, variable::YoDouble* inputVelocity,
                                          variable::YoDouble* inputAcceleration, double dt)
   : YoDouble(name, registry), dt_(dt), hasBeenInitialized_(name + "HasBeenInitialized", registry), smoothedRate_(name + "SmoothedRate", registry),
     smoothedAcceleration_(name + "SmoothedAcceleration", registry), smoothedJerk_(name + "SmoothedJerk", registry),
     positionGain_(name + "PositionGain", registry), velocityGain_(name + "VelocityGain", registry),
     accelerationGain_(name + "AccelerationGain", registry), maximumJerk_(maxJerk), maximumAcceleration_(maxAcceleration), inputPosition_(inputPosition),
     inputVelocity_(inputVelocity), inputAcceleration_(inputAcceleration)
{
   double w0 = 2.0 * M_PI * 16.0;
   double w1 = 2.0 * M_PI * 16.0;
   double zeta = 1.0;

   setGainsByPolePlacement(w0, w1, zeta);
   hasBeenInitialized_.set(false);
}

void JerkLimitedYoDouble::setMaximumAcceleration(double maximumAcceleration)
{
   maximumAcceleration_.set(maximumAcceleration);
}

void JerkLimitedYoDouble::setMaximumJerk(double maximumJerk)
{
   maximumJerk_.set(maximumJerk);
}

void JerkLimitedYoDouble::setGainsByPolePlacement(double w0, double w1, double zeta)
{
   positionGain_.set(w0 * w1 * w1);
   velocityGain_.set(w1 * w1 + 2.0 * zeta * w1 * w0);
   accelerationGain_.set(w0 + 2.0 * zeta * w1);
}

void JerkLimitedYoDouble::update()
{
   double inputPositionValue = inputPosition_ == nullptr ? 0.0 : inputPosition_->getDoubleValue();
   double inputVelocityValue = inputVelocity_ == nullptr ? 0.0 : inputVelocity_->getDoubleValue();
   double inputAccelerationValue = inputAcceleration_ == nullptr ? 0.0 : inputAcceleration_->getDoubleValue();

   update(inputPositionValue, inputVelocityValue, inputAccelerationValue);
}

void JerkLimitedYoDouble::update(double inputPosition)
{
   update(inputPosition, 0.0, 0.0);
}

void JerkLimitedYoDouble::update(double inputPosition, double inputVelocity)
{
   update(inputPosition, inputVelocity, 0.0);
}

void JerkLimitedYoDouble::update(double inputPosition, double inputVelocity, double inputAcceleration)
{
   if (!hasBeenInitialized_.getBooleanValue())
      initialize(inputPosition, inputVelocity, inputAcceleration);

   double positionError = inputPosition - getDoubleValue();
   double velocityError = inputVelocity - smoothedRate_.getDoubleValue();
   double accelerationError = inputAcceleration - smoothedAcceleration_.getDoubleValue();
   double jerk = accelerationGain_.getDoubleValue() * accelerationError + velocityGain_.getDoubleValue() * velocityError
                 + positionGain_.getDoubleValue() * positionError;
   jerk = clampSymmetric(jerk, maximumJerk_.getDoubleValue());

   smoothedJerk_.set(jerk);
   smoothedAcceleration_.add(smoothedJerk_.getDoubleValue() * dt_);
   smoothedAcceleration_.set(clampSymmetric(smoothedAcceleration_.getDoubleValue(), maximumJerk_.getDoubleValue()));
   smoothedRate_.add(smoothedAcceleration_.getDoubleValue() * dt_);
   add(smoothedRate_.getDoubleValue() * dt_);
}

void JerkLimitedYoDouble::initialize(double inputPosition, double inputVelocity, double inputAcceleration)
{
   set(inputPosition);
   smoothedRate_.set(inputVelocity);
   smoothedAcceleration_.set(inputAcceleration);
   smoothedJerk_.set(0.0);

   hasBeenInitialized_.set(true);
}

void JerkLimitedYoDouble::reset()
{
   hasBeenInitialized_.set(false);
   smoothedRate_.set(0.0);
   smoothedAcceleration_.set(0.0);
   smoothedJerk_.set(0.0);
}
}
