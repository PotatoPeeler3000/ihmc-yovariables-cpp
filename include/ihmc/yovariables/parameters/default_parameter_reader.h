#pragma once

#include "ihmc/yovariables/parameters/abstract_parameter_reader.h"

namespace ihmc::yovariables::parameters
{
/** Parameter reader that always initializes parameters to their constructor-provided default. */
class DefaultParameterReader : public AbstractParameterReader
{
protected:
   const std::unordered_map<std::string, ParameterData>& getValues() const override;
};
}
