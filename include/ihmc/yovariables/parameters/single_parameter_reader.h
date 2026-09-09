#pragma once

#include "ihmc/yovariables/parameters/parameter_load_status.h"

namespace ihmc::yovariables::parameters
{
class YoParameter;

/**
 * Tools for initializing a single parameter. Main use case: deserialization of parameters.
 */
namespace SingleParameterReader
{
/**
 * Initializes parameter from doubleValue.
 *
 * @throws std::invalid_argument if loadStatus == ParameterLoadStatus::UNLOADED.
 */
void readParameter(YoParameter& parameter, double doubleValue, ParameterLoadStatus loadStatus);
}
}
