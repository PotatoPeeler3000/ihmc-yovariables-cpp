#pragma once

#include <optional>

#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
class ButterworthFilteredYoVariable : public variable::YoDouble
{
public:
   enum class ButterworthFilterType
   {
      LOW_PASS,
      HIGH_PASS
   };

   ButterworthFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, double alpha, ButterworthFilterType butterworthFilterType,
                                  variable::YoDouble* positionVariable = nullptr);
   ButterworthFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& alphaVariable,
                                  ButterworthFilterType butterworthFilterType, variable::YoDouble* positionVariable = nullptr);

   void reset();

   /** @throws std::logic_error if constructed without a position variable. */
   void update();
   void update(double currentInput);

   static double computeAlphaGivenBreakFrequency(double breakFrequencyInHertz, double dt);
   static double computeBreakFrequencyGivenAlpha(double alpha, double dt);

private:
   providers::DoubleProvider* alphaVariable_ = nullptr;
   ButterworthFilterType butterworthFilterType_;

   variable::YoDouble* position_ = nullptr;
   variable::YoDouble previousInput_;

   variable::YoBoolean hasBeenCalled_;

   std::optional<variable::YoDouble> internalAlpha_;
};
}
