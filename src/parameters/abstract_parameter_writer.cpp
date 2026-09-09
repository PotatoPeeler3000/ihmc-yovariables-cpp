#include "ihmc/yovariables/parameters/abstract_parameter_writer.h"

#include <stdexcept>

#include "ihmc/yovariables/parameters/abstract_parameter_reader.h"
#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_variable.h"
#include "ihmc/yovariables/variable/yo_variable_type.h"

namespace ihmc::yovariables::parameters
{
namespace
{
/**
 * Java derives this from parameter.getClass().getSimpleName() reflectively; C++ has no equivalent,
 * so it is derived from the backing variable's YoVariableType instead - XmlParameterReader never
 * actually reads this field back (only value/min/max/name), so this is purely informational, and
 * the strings produced here match Java's class-name-derived ones exactly.
 */
std::string parameterTypeName(YoParameter& parameter)
{
   switch (parameter.getVariable().getType())
   {
      case variable::YoVariableType::DOUBLE:
         return "DoubleParameter";
      case variable::YoVariableType::BOOLEAN:
         return "BooleanParameter";
      case variable::YoVariableType::ENUM:
         return "EnumParameter";
      case variable::YoVariableType::INTEGER:
         return "IntegerParameter";
      case variable::YoVariableType::LONG:
         return "LongParameter";
      default:
         throw std::logic_error("Unhandled YoVariableType");
   }
}
} // namespace

void AbstractParameterWriter::addParameters(registry::YoRegistry& registry)
{
   std::vector<YoParameter*> parameters = registry.collectSubtreeParameters();

   for (YoParameter* parameter : parameters)
   {
      registry::YoNamespace relativeNamespace = AbstractParameterReader::getRelativeNamespace(*parameter->getNamespace(), registry);

      std::string value = parameter->getValueAsString();
      std::string min = std::to_string(parameter->getVariable().getLowerBound());
      std::string max = std::to_string(parameter->getVariable().getUpperBound());
      setValue(relativeNamespace, parameter->getName(), parameter->getDescription(), parameterTypeName(*parameter), value, min, max);
   }
}
}
