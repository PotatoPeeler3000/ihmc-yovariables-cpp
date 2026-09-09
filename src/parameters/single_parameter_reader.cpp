#include "ihmc/yovariables/parameters/single_parameter_reader.h"

#include <stdexcept>

#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::parameters::SingleParameterReader
{
void readParameter(YoParameter& parameter, double doubleValue, ParameterLoadStatus loadStatus)
{
   if (loadStatus == ParameterLoadStatus::UNLOADED)
      throw std::invalid_argument("Can not load parameter and set the status to unloaded.");

   parameter.getVariable().setValueFromDouble(doubleValue);
   parameter.setLoadStatus(loadStatus);
}
}
