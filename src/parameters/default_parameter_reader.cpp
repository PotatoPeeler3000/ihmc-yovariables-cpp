#include "ihmc/yovariables/parameters/default_parameter_reader.h"

namespace ihmc::yovariables::parameters
{
const std::unordered_map<std::string, ParameterData>& DefaultParameterReader::getValues() const
{
   static const std::unordered_map<std::string, ParameterData> empty;
   return empty;
}
}
