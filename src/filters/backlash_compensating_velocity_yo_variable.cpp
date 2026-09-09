#include "ihmc/yovariables/filters/backlash_compensating_velocity_yo_variable.h"

#include <cmath>
#include <stdexcept>

#include "ihmc/yovariables/filters/filter_math.h"
#include "ihmc/yovariables/filters/variable_tools.h"

namespace ihmc::yovariables::filters
{
BacklashCompensatingVelocityYoVariable::BacklashCompensatingVelocityYoVariable(const std::string& name, const std::string& description,
                                                                                 providers::DoubleProvider& alphaVariable, double dt,
                                                                                 providers::DoubleProvider& slopTime, registry::YoRegistry* registry,
                                                                                 providers::DoubleProvider* positionVariable)
   : YoDouble(name, description, registry), dt_(dt), finiteDifferenceVelocity_(name + "finiteDifferenceVelocity", "", alphaVariable, dt, registry),
     alphaVariable_(alphaVariable), position_(positionVariable), lastPosition_(name + "_lastPosition", registry),
     hasBeenCalled_(hasBeenCalledName(name, ""), registry), backlashState_(name + "BacklashState", registry, true), slopTime_(slopTime),
     timeSinceSloppy_(name + "TimeSinceSloppy", registry)
{
   backlashState_.set(variable::YoEnum<BacklashState>::NULL_VALUE);
   reset();
}

void BacklashCompensatingVelocityYoVariable::reset()
{
   hasBeenCalled_.set(false);
   backlashState_.set(variable::YoEnum<BacklashState>::NULL_VALUE);
}

void BacklashCompensatingVelocityYoVariable::update()
{
   if (position_ == nullptr)
      throw std::logic_error("BacklashCompensatingVelocityYoVariable must be constructed with a non-null position variable to call update().");
   update(position_->getValue());
}

void BacklashCompensatingVelocityYoVariable::update(double currentPosition)
{
   if (!hasBeenCalled_.getBooleanValue())
   {
      hasBeenCalled_.set(true);
      lastPosition_.set(currentPosition);
      set(0.0);
   }

   finiteDifferenceVelocity_.update(currentPosition);
   double velocityFromFiniteDifferences = finiteDifferenceVelocity_.getDoubleValue();

   timeSinceSloppy_.add(dt_);

   std::optional<BacklashState> currentState = backlashState_.getEnumValueOrNull();

   if (!currentState.has_value())
   {
      if (velocityFromFiniteDifferences < 0.0)
         backlashState_.set(BacklashState::BACKWARD_OK);
      else if (velocityFromFiniteDifferences > 0.0)
         backlashState_.set(BacklashState::FORWARD_OK);
   }
   else
   {
      switch (*currentState)
      {
         case BacklashState::BACKWARD_OK:
            if (velocityFromFiniteDifferences > 0.0)
            {
               timeSinceSloppy_.set(0.0);
               backlashState_.set(BacklashState::FORWARD_SLOP);
            }
            break;

         case BacklashState::FORWARD_OK:
            if (velocityFromFiniteDifferences < 0.0)
            {
               timeSinceSloppy_.set(0.0);
               backlashState_.set(BacklashState::BACKWARD_SLOP);
            }
            break;

         case BacklashState::BACKWARD_SLOP:
            if (velocityFromFiniteDifferences > 0.0)
            {
               timeSinceSloppy_.set(0.0);
               backlashState_.set(BacklashState::FORWARD_SLOP);
            }
            else if (timeSinceSloppy_.getDoubleValue() > slopTime_.getValue())
            {
               backlashState_.set(BacklashState::BACKWARD_OK);
               timeSinceSloppy_.set(0.0);
            }
            break;

         case BacklashState::FORWARD_SLOP:
            if (velocityFromFiniteDifferences < 0.0)
            {
               timeSinceSloppy_.set(0.0);
               backlashState_.set(BacklashState::BACKWARD_SLOP);
            }
            else if (timeSinceSloppy_.getDoubleValue() > slopTime_.getValue())
            {
               backlashState_.set(BacklashState::FORWARD_OK);
               timeSinceSloppy_.set(0.0);
            }
            break;
      }
   }

   double difference = currentPosition - lastPosition_.getDoubleValue();

   std::optional<BacklashState> stateAfterUpdate = backlashState_.getEnumValueOrNull();
   if (stateAfterUpdate.has_value() && isInBacklash(*stateAfterUpdate))
   {
      double alpha = timeSinceSloppy_.getDoubleValue() / slopTime_.getValue();
      alpha = clamp(alpha, 0.0, 1.0);
      if (std::isnan(alpha))
         alpha = 1.0;

      difference = alpha * difference;
   }

   updateUsingDifference(difference);
   lastPosition_.set(currentPosition);
}

void BacklashCompensatingVelocityYoVariable::updateUsingDifference(double difference)
{
   double previousFilteredDerivative = getDoubleValue();
   double currentRawDerivative = difference / dt_;

   set(interpolate(currentRawDerivative, previousFilteredDerivative, alphaVariable_.getValue()));
}
}
