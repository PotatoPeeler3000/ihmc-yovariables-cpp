#include "ihmc/yovariables/filters/butterworth_filtered_yo_variable.h"

#include <cmath>
#include <stdexcept>

#include "ihmc/yovariables/filters/filter_math.h"

namespace ihmc::yovariables::filters
{
ButterworthFilteredYoVariable::ButterworthFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, double alpha,
                                                               ButterworthFilterType butterworthFilterType, variable::YoDouble* positionVariable)
   : YoDouble(name, registry), butterworthFilterType_(butterworthFilterType), position_(positionVariable), previousInput_(name + "_prevIn", registry),
     hasBeenCalled_(name + "HasBeenCalled", registry)
{
   internalAlpha_.emplace(name + "AlphaVariable", registry);
   internalAlpha_->set(alpha);
   alphaVariable_ = &*internalAlpha_;
   reset();
}

ButterworthFilteredYoVariable::ButterworthFilteredYoVariable(const std::string& name, registry::YoRegistry* registry,
                                                               providers::DoubleProvider& alphaVariable, ButterworthFilterType butterworthFilterType,
                                                               variable::YoDouble* positionVariable)
   : YoDouble(name, registry), alphaVariable_(&alphaVariable), butterworthFilterType_(butterworthFilterType), position_(positionVariable),
     previousInput_(name + "_prevIn", registry), hasBeenCalled_(name + "HasBeenCalled", registry)
{
   reset();
}

void ButterworthFilteredYoVariable::reset()
{
   hasBeenCalled_.set(false);
}

void ButterworthFilteredYoVariable::update()
{
   if (position_ == nullptr)
      throw std::logic_error("ButterworthFilteredYoVariable must be constructed with a non-null position variable to call update().");
   update(position_->getValue());
}

void ButterworthFilteredYoVariable::update(double currentInput)
{
   if (!hasBeenCalled_.getValue())
   {
      hasBeenCalled_.set(true);

      if (butterworthFilterType_ == ButterworthFilterType::HIGH_PASS)
         set(0.0);
      else
         set(currentInput);
   }
   else
   {
      double alpha = alphaVariable_->getValue();

      switch (butterworthFilterType_)
      {
         case ButterworthFilterType::LOW_PASS:
            set(alpha * getValue() + 0.5 * (1.0 - alpha) * (currentInput + previousInput_.getValue()));
            break;
         case ButterworthFilterType::HIGH_PASS:
            set(alpha * getValue() + 0.5 * (1.0 + alpha) * (currentInput - previousInput_.getValue()));
            break;
      }
   }

   previousInput_.set(currentInput);
}

double ButterworthFilteredYoVariable::computeAlphaGivenBreakFrequency(double breakFrequencyInHertz, double dt)
{
   if (std::isinf(breakFrequencyInHertz))
      return 0.0;

   double samplingFrequency = 1.0 / dt;
   if (breakFrequencyInHertz > 0.25 * samplingFrequency)
      return 0.0;

   double tanOmegaBreak = std::tan(M_PI * breakFrequencyInHertz * dt);
   return clamp((1.0 - tanOmegaBreak) / (1.0 + tanOmegaBreak), 0.0, 1.0);
}

double ButterworthFilteredYoVariable::computeBreakFrequencyGivenAlpha(double alpha, double dt)
{
   double beta = (1.0 - alpha) / (1.0 + alpha);
   return std::max(std::atan(beta) / (dt * M_PI), 0.0);
}
}
