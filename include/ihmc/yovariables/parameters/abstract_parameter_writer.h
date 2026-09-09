#pragma once

#include <string>

namespace ihmc::yovariables::registry
{
class YoNamespace;
class YoRegistry;
}

namespace ihmc::yovariables::parameters
{
/**
 * Base class for parameter writers, used to export parameters to an external destination (e.g. an
 * XML file when using XmlParameterWriter).
 */
class AbstractParameterWriter
{
public:
   virtual ~AbstractParameterWriter() = default;

   /** Recurses from registry, finds all parameters, and calls setValue() for each. */
   void addParameters(registry::YoRegistry& registry);

protected:
   /** Exports the data for a single parameter. */
   virtual void setValue(const registry::YoNamespace& namespaceValue, const std::string& name, const std::string& description, const std::string& type,
                          const std::string& value, const std::string& min, const std::string& max) = 0;
};
}
