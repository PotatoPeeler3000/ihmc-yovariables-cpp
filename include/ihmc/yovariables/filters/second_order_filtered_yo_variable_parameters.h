#pragma once

#include "ihmc/yovariables/filters/second_order_filter_type.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
class SecondOrderFilteredYoVariableParameters
{
public:
   SecondOrderFilteredYoVariableParameters(const std::string& name, registry::YoRegistry* registry, double naturalFrequencyInHz, double dampingRatio,
                                            SecondOrderFilterType filterType);

   variable::YoDouble& getNaturalFrequencyInHz();
   variable::YoDouble& getDampingRatio();
   SecondOrderFilterType getFilterType() const;

private:
   variable::YoDouble naturalFrequencyInHz_;
   variable::YoDouble dampingRatio_;
   SecondOrderFilterType filterType_;
};
}
