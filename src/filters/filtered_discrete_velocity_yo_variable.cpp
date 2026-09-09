#include "ihmc/yovariables/filters/filtered_discrete_velocity_yo_variable.h"

#include "ihmc/yovariables/filters/filter_math.h"

namespace ihmc::yovariables::filters
{
FilteredDiscreteVelocityYoVariable::FilteredDiscreteVelocityYoVariable(const std::string& name, const std::string& description,
                                                                         providers::DoubleProvider& alphaVariable, variable::YoDouble& positionVariable,
                                                                         variable::YoDouble& time, registry::YoRegistry* registry)
   : YoDouble(name, description, registry), time_(time), alphaVariable_(alphaVariable), position_(positionVariable),
     lastUpdateTime_(name + "_lastUpdateTime", registry), lastUpdateDirection_(name + "_lastUpdateDirection", registry, false),
     unfilteredVelocity_(name + "_unfilteredVelocity", registry), lastPosition_(name + "_lastPosition", registry)
{
   reset();
}

void FilteredDiscreteVelocityYoVariable::reset()
{
   hasBeenCalled_ = false;
}

void FilteredDiscreteVelocityYoVariable::update()
{
   update(position_.getDoubleValue());
}

void FilteredDiscreteVelocityYoVariable::update(double currentPosition)
{
   if (!hasBeenCalled_)
   {
      hasBeenCalled_ = true;
      lastPosition_.set(currentPosition);
      lastUpdateTime_.set(time_.getDoubleValue());
      lastUpdateDirection_.set(FilteredDiscreteVelocityDirection::NONE);
   }

   bool countChanged = currentPosition != lastPosition_.getDoubleValue();

   bool directionChanged = false;
   if (countChanged)
   {
      if (currentPosition > lastPosition_.getDoubleValue())
      {
         if (lastUpdateDirection_.getEnumValue() != FilteredDiscreteVelocityDirection::FORWARD)
            directionChanged = true;
         lastUpdateDirection_.set(FilteredDiscreteVelocityDirection::FORWARD);
      }
      else if (currentPosition < lastPosition_.getDoubleValue())
      {
         if (lastUpdateDirection_.getEnumValue() != FilteredDiscreteVelocityDirection::BACKWARD)
            directionChanged = true;
         lastUpdateDirection_.set(FilteredDiscreteVelocityDirection::BACKWARD);
      }
   }

   if (directionChanged)
   {
      unfilteredVelocity_.set(0.0);
   }
   else if (countChanged)
   {
      double diffTime = time_.getDoubleValue() - lastUpdateTime_.getDoubleValue();
      if (diffTime < 1e-7)
         unfilteredVelocity_.set(0.0);
      else
         unfilteredVelocity_.set((currentPosition - lastPosition_.getDoubleValue()) / diffTime);
   }
   else
   {
      // Count hasn't changed: not clear what the velocity actually is. Multiply by a largish
      // fraction so it trails to zero if the velocity stops quickly, rather than holding forever.
      unfilteredVelocity_.set(0.99 * unfilteredVelocity_.getDoubleValue());
   }

   set(interpolate(unfilteredVelocity_.getDoubleValue(), getDoubleValue(), alphaVariable_.getValue()));

   if (countChanged)
   {
      lastPosition_.set(currentPosition);
      lastUpdateTime_.set(time_.getDoubleValue());
   }
}

double FilteredDiscreteVelocityYoVariable::getUnfilteredVelocity() const
{
   return unfilteredVelocity_.getDoubleValue();
}
}
