#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ihmc/yovariables/parameters/parameter_data.h"

namespace ihmc::yovariables::registry
{
class YoNamespace;
class YoRegistry;
}

namespace ihmc::yovariables::parameters
{
/**
 * Base class for parameter readers, used to initialize parameters from an external source (e.g.
 * an XML file when using XmlParameterReader).
 */
class AbstractParameterReader
{
public:
   virtual ~AbstractParameterReader() = default;

   /**
    * Recurses from registry, finds all parameters, and initializes them from getValues(). Parameters
    * getValues() does not cover are initialized to default (see YoParameter::loadDefault()).
    */
   void readParametersInRegistry(registry::YoRegistry& registry);

   /**
    * Same as readParametersInRegistry(YoRegistry&), additionally reporting which parameters used
    * their default value and which entries in getValues() matched no parameter in the registry.
    */
   void readParametersInRegistry(registry::YoRegistry& registry, std::unordered_set<std::string>& defaultParametersToPack,
                                  std::unordered_set<std::string>& unmatchedParametersToPack);

   static registry::YoNamespace getRelativeNamespace(const registry::YoNamespace& parameterNamespace, registry::YoRegistry& registry);

protected:
   /** Map from parameter full-name to its initial value, used to initialize parameters. */
   virtual const std::unordered_map<std::string, ParameterData>& getValues() const = 0;
};
}
