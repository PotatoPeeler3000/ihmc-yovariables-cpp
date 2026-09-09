#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::parameters
{
class YoParameter;
}

namespace ihmc::yovariables::registry
{
class YoRegistry;
class YoVariableHolder;
}

namespace ihmc::yovariables::tools
{
using VariablePredicate = std::function<bool(variable::YoVariable*)>;
using RegistryPredicate = std::function<bool(registry::YoRegistry*)>;
using ParameterPredicate = std::function<bool(parameters::YoParameter*)>;

/** Recurses the registry subtree to find the first parameter matching the search criteria. */
parameters::YoParameter* findFirstParameter(const std::optional<std::string>& namespaceEnding,
                                             const std::string& name,
                                             const ParameterPredicate& predicate,
                                             registry::YoRegistry& registry);

/** Recurses the registry subtree to find the first variable matching the search criteria. */
variable::YoVariable* findFirstVariable(const std::optional<std::string>& namespaceEnding,
                                         const std::string& name,
                                         const VariablePredicate& predicate,
                                         registry::YoRegistry& registry);

/** Recurses the registry subtree to find the first registry matching the search criteria. */
registry::YoRegistry* findFirstRegistry(const std::optional<std::string>& namespaceEnding,
                                         const std::string& name,
                                         const RegistryPredicate& predicate,
                                         registry::YoRegistry& root);

/** Recurses the registry subtree to find all variables matching the search criteria. */
std::vector<variable::YoVariable*> findVariables(const std::optional<std::string>& namespaceEnding,
                                                  const std::string& name,
                                                  const VariablePredicate& predicate,
                                                  registry::YoRegistry& registry);

/** Recurses the yoVariableHolder subtree to find the first variable for which filter returns true. */
variable::YoVariable* findVariable(const VariablePredicate& filter, registry::YoVariableHolder& yoVariableHolder);

/** Recurses the yoVariableHolder subtree to find all variables for which filter returns true. */
std::vector<variable::YoVariable*> filterVariables(const VariablePredicate& filter, registry::YoVariableHolder& yoVariableHolder);

/** Recurses the registry subtree to find all registries for which filter returns true. */
std::vector<registry::YoRegistry*> filterRegistries(const RegistryPredicate& filter, registry::YoRegistry& registry);

/** Creates a filter that matches a variable's name against any of the given regular expressions. */
VariablePredicate regularExpressionFilter(const std::vector<std::string>& regularExpressions);
}
