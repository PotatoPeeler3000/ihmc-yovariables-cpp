#pragma once

#include "ihmc/yovariables/filters/alpha_filtered_yo_variable.h"

namespace ihmc::yovariables::filters
{
/** AlphaFilteredYoVariable that wraps its value within [lowerLimit, upperLimit). */
class AlphaFilteredWrappingYoVariable : public AlphaFilteredYoVariable
{
public:
   static constexpr double EPSILON = 1e-10;

   AlphaFilteredWrappingYoVariable(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                                    variable::YoDouble& unfilteredVariable, providers::DoubleProvider& alphaVariable, double lowerLimit, double upperLimit);

   void update() override;
   void update(double currentPosition) override;

private:
   void unfilteredVariableModulo(double currentPosition);

   double previousUnfilteredVariable_ = 0.0;
   variable::YoDouble& unfilteredVariable_;
   variable::YoDouble unfilteredInRangeVariable_;
   providers::DoubleProvider& alphaVariable_;

   variable::YoDouble temporaryOutputVariable_;
   variable::YoDouble error_;
   double upperLimit_;
   double lowerLimit_;
   double range_;
};
}
