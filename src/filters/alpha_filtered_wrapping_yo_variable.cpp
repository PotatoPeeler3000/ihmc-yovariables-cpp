#include "ihmc/yovariables/filters/alpha_filtered_wrapping_yo_variable.h"

#include <cmath>

#include "ihmc/yovariables/filters/filter_math.h"

namespace ihmc::yovariables::filters
{
AlphaFilteredWrappingYoVariable::AlphaFilteredWrappingYoVariable(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                                                                   variable::YoDouble& unfilteredVariable, providers::DoubleProvider& alphaVariable,
                                                                   double lowerLimit, double upperLimit)
   : AlphaFilteredYoVariable(name, description, registry, alphaVariable), unfilteredVariable_(unfilteredVariable),
     unfilteredInRangeVariable_(name + "UnfilteredInRangeVariable", registry), alphaVariable_(alphaVariable), upperLimit_(upperLimit),
     lowerLimit_(lowerLimit), range_(upperLimit - lowerLimit), temporaryOutputVariable_(name + "TemporaryOutputVariable", registry),
     error_(name + "Error", registry)
{
}

void AlphaFilteredWrappingYoVariable::update()
{
   update(unfilteredVariable_.getDoubleValue());
}

void AlphaFilteredWrappingYoVariable::update(double currentPosition)
{
   if (!hasBeenCalled.getBooleanValue())
   {
      hasBeenCalled.set(true);
      previousUnfilteredVariable_ = unfilteredVariable_.getDoubleValue();

      unfilteredVariableModulo(currentPosition);

      temporaryOutputVariable_.set(unfilteredInRangeVariable_.getDoubleValue());
      set(unfilteredInRangeVariable_.getDoubleValue());
   }
   else
   {
      if (!epsilonEquals(currentPosition, previousUnfilteredVariable_, EPSILON))
      {
         previousUnfilteredVariable_ = currentPosition;

         unfilteredVariableModulo(currentPosition);

         double standardError = unfilteredInRangeVariable_.getDoubleValue() - getDoubleValue();
         double wrappingError;
         if (unfilteredInRangeVariable_.getDoubleValue() > getDoubleValue())
            wrappingError = lowerLimit_ - getDoubleValue() + unfilteredInRangeVariable_.getDoubleValue() - upperLimit_;
         else
            wrappingError = upperLimit_ - getDoubleValue() + unfilteredInRangeVariable_.getDoubleValue() - lowerLimit_;

         if (std::abs(standardError) < std::abs(wrappingError))
            error_.set(standardError);
         else
            error_.set(wrappingError);

         temporaryOutputVariable_.set(getDoubleValue());
         if (getDoubleValue() + error_.getDoubleValue() >= upperLimit_)
            temporaryOutputVariable_.set(getDoubleValue() - range_);
         if (getDoubleValue() + error_.getDoubleValue() < lowerLimit_)
            temporaryOutputVariable_.set(getDoubleValue() + range_);
      }

      temporaryOutputVariable_.set(alphaVariable_.getValue() * temporaryOutputVariable_.getDoubleValue()
                                    + (1.0 - alphaVariable_.getValue()) * unfilteredInRangeVariable_.getDoubleValue());

      if (temporaryOutputVariable_.getDoubleValue() > upperLimit_ + EPSILON)
         set(temporaryOutputVariable_.getDoubleValue() - range_);
      else if (temporaryOutputVariable_.getDoubleValue() <= lowerLimit_ - EPSILON)
         set(temporaryOutputVariable_.getDoubleValue() + range_);
      else
         set(temporaryOutputVariable_.getDoubleValue());
   }
}

void AlphaFilteredWrappingYoVariable::unfilteredVariableModulo(double currentPosition)
{
   bool rangeNeedsToBeChecked = true;
   unfilteredInRangeVariable_.set(currentPosition);

   while (rangeNeedsToBeChecked)
   {
      rangeNeedsToBeChecked = false;
      if (unfilteredInRangeVariable_.getDoubleValue() >= upperLimit_)
      {
         unfilteredInRangeVariable_.set(unfilteredInRangeVariable_.getDoubleValue() - range_);
         rangeNeedsToBeChecked = true;
      }
      if (unfilteredInRangeVariable_.getDoubleValue() < lowerLimit_)
      {
         unfilteredInRangeVariable_.set(unfilteredInRangeVariable_.getDoubleValue() + range_);
         rangeNeedsToBeChecked = true;
      }
   }
}
}
