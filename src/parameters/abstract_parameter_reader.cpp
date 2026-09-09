#include "ihmc/yovariables/parameters/abstract_parameter_reader.h"

#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry.h"

namespace ihmc::yovariables::parameters
{
void AbstractParameterReader::readParametersInRegistry(registry::YoRegistry& registry)
{
   std::unordered_set<std::string> defaultParameters;
   std::unordered_set<std::string> unmatchedParameters;
   readParametersInRegistry(registry, defaultParameters, unmatchedParameters);
}

void AbstractParameterReader::readParametersInRegistry(registry::YoRegistry& registry, std::unordered_set<std::string>& defaultParametersToPack,
                                                         std::unordered_set<std::string>& unmatchedParametersToPack)
{
   defaultParametersToPack.clear();
   unmatchedParametersToPack.clear();

   std::vector<YoParameter*> parameters = registry.collectSubtreeParameters();
   std::unordered_map<std::string, ParameterData> localMap = getValues();

   for (YoParameter* parameter : parameters)
   {
      registry::YoNamespace relativeNamespace = getRelativeNamespace(*parameter->getNamespace(), registry);
      std::string fullName = relativeNamespace.getName() + "." + parameter->getName();

      auto it = localMap.find(fullName);
      if (it != localMap.end())
      {
         it->second.setParameterFromThis(*parameter);
         localMap.erase(it);
      }
      else
      {
         parameter->loadDefault();
         defaultParametersToPack.insert(fullName);
      }
   }

   for (const auto& [name, data] : localMap)
      unmatchedParametersToPack.insert(name);
}

registry::YoNamespace AbstractParameterReader::getRelativeNamespace(const registry::YoNamespace& parameterNamespace, registry::YoRegistry& registry)
{
   const registry::YoNamespace& registryNamespace = registry.getNamespace();
   if (registryNamespace.isRoot())
      return parameterNamespace;
   return parameterNamespace.removeStart(*registry.getNamespace().removeEnd(1)).value();
}
}
