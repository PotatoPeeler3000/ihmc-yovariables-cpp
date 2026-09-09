#include "ihmc/yovariables/filters/backlash_processing_yo_variable.h"

#include <cmath>
#include <stdexcept>

#include "ihmc/yovariables/filters/filter_math.h"
#include "ihmc/yovariables/filters/variable_tools.h"

namespace ihmc::yovariables::filters
{
BacklashProcessingYoVariable::BacklashProcessingYoVariable(const std::string& name, const std::string& description, double dt,
                                                             providers::DoubleProvider& slopTime, registry::YoRegistry* registry,
                                                             variable::YoDouble* velocityVariable)
   : YoDouble(name, description, registry), velocity_(velocityVariable), hasBeenCalled_(hasBeenCalledName(name, ""), registry),
     backlashState_(name + "BacklashState", registry, true), slopTime_(slopTime), timeSinceSloppy_(name + "TimeSinceSloppy", registry), dt_(dt)
{
   backlashState_.set(variable::YoEnum<BacklashState>::NULL_VALUE);
   reset();
}

void BacklashProcessingYoVariable::reset()
{
   hasBeenCalled_.set(false);
   backlashState_.set(variable::YoEnum<BacklashState>::NULL_VALUE);
}

void BacklashProcessingYoVariable::update()
{
   if (velocity_ == nullptr)
      throw std::logic_error("BacklashProcessingYoVariable must be constructed with a non-null velocity variable to call update().");
   update(velocity_->getDoubleValue());
}

void BacklashProcessingYoVariable::update(double currentVelocity)
{
   if (!backlashState_.getEnumValueOrNull().has_value())
      backlashState_.set(BacklashState::FORWARD_OK);

   if (!hasBeenCalled_.getBooleanValue())
   {
      hasBeenCalled_.set(true);
      set(currentVelocity);
   }

   timeSinceSloppy_.add(dt_);

   switch (*backlashState_.getEnumValueOrNull())
   {
      case BacklashState::BACKWARD_OK:
         if (currentVelocity > 0.0)
         {
            timeSinceSloppy_.set(0.0);
            backlashState_.set(BacklashState::FORWARD_SLOP);
         }
         break;

      case BacklashState::FORWARD_OK:
         if (currentVelocity < 0.0)
         {
            timeSinceSloppy_.set(0.0);
            backlashState_.set(BacklashState::BACKWARD_SLOP);
         }
         break;

      case BacklashState::BACKWARD_SLOP:
         if (currentVelocity > 0.0)
         {
            timeSinceSloppy_.set(0.0);
            backlashState_.set(BacklashState::FORWARD_SLOP);
         }
         else if (timeSinceSloppy_.getDoubleValue() > slopTime_.getValue())
         {
            backlashState_.set(BacklashState::BACKWARD_OK);
         }
         break;

      case BacklashState::FORWARD_SLOP:
         if (currentVelocity < 0.0)
         {
            timeSinceSloppy_.set(0.0);
            backlashState_.set(BacklashState::BACKWARD_SLOP);
         }
         else if (timeSinceSloppy_.getDoubleValue() > slopTime_.getValue())
         {
            backlashState_.set(BacklashState::FORWARD_OK);
         }
         break;
   }

   double percent = timeSinceSloppy_.getDoubleValue() / slopTime_.getValue();
   percent = clamp(percent, 0.0, 1.0);
   if (std::isnan(percent) || slopTime_.getValue() < dt_)
      percent = 1.0;

   set(percent * currentVelocity);
}
}
