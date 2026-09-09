#include "ihmc/yovariables/filters/second_order_filtered_yo_variable_parameters.h"

namespace ihmc::yovariables::filters
{
SecondOrderFilteredYoVariableParameters::SecondOrderFilteredYoVariableParameters(const std::string& name, registry::YoRegistry* registry,
                                                                                   double naturalFrequencyInHz, double dampingRatio,
                                                                                   SecondOrderFilterType filterType)
   : naturalFrequencyInHz_(name + "NaturalFrequency", registry), dampingRatio_(name + "DampingRatio", registry), filterType_(filterType)
{
   naturalFrequencyInHz_.set(naturalFrequencyInHz);
   dampingRatio_.set(dampingRatio);
}

variable::YoDouble& SecondOrderFilteredYoVariableParameters::getNaturalFrequencyInHz()
{
   return naturalFrequencyInHz_;
}

variable::YoDouble& SecondOrderFilteredYoVariableParameters::getDampingRatio()
{
   return dampingRatio_;
}

SecondOrderFilterType SecondOrderFilteredYoVariableParameters::getFilterType() const
{
   return filterType_;
}
}
