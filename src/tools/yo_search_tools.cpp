#include "ihmc/yovariables/tools/yo_search_tools.h"

#include <regex>

#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/registry/yo_variable_holder.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::tools
{
parameters::YoParameter* findFirstParameter(const std::optional<std::string>& namespaceEnding,
                                             const std::string& name,
                                             const ParameterPredicate& predicate,
                                             registry::YoRegistry& registry)
{
   VariablePredicate variablePredicate = [&](variable::YoVariable* candidate) {
      if (!candidate->isParameter())
         return false;
      return !predicate || predicate(candidate->getParameter());
   };

   variable::YoVariable* result = findFirstVariable(namespaceEnding, name, variablePredicate, registry);
   return result == nullptr ? nullptr : result->getParameter();
}

variable::YoVariable* findFirstVariable(const std::optional<std::string>& namespaceEnding,
                                         const std::string& name,
                                         const VariablePredicate& predicate,
                                         registry::YoRegistry& registry)
{
   if (!namespaceEnding.has_value() || registry.getNamespace().endsWith(*namespaceEnding))
   {
      variable::YoVariable* candidate = registry.getVariable(name);
      if (candidate != nullptr && (!predicate || predicate(candidate)))
         return candidate;
   }

   for (registry::YoRegistry* child : registry.getChildRegistries())
   {
      variable::YoVariable* result = findFirstVariable(namespaceEnding, name, predicate, *child);
      if (result != nullptr)
         return result;
   }

   return nullptr;
}

registry::YoRegistry* findFirstRegistry(const std::optional<std::string>& namespaceEnding,
                                         const std::string& name,
                                         const RegistryPredicate& predicate,
                                         registry::YoRegistry& root)
{
   if (!namespaceEnding.has_value() || root.getNamespace().endsWith(*namespaceEnding))
   {
      registry::YoRegistry* candidate = root.getChild(name);
      if (candidate != nullptr && (!predicate || predicate(candidate)))
         return candidate;
   }

   for (registry::YoRegistry* child : root.getChildRegistries())
   {
      registry::YoRegistry* result = findFirstRegistry(namespaceEnding, name, predicate, *child);
      if (result != nullptr)
         return result;
   }

   return nullptr;
}

std::vector<variable::YoVariable*> findVariables(const std::optional<std::string>& namespaceEnding,
                                                  const std::string& name,
                                                  const VariablePredicate& predicate,
                                                  registry::YoRegistry& registry)
{
   std::vector<variable::YoVariable*> result;

   std::function<void(registry::YoRegistry&)> recurse = [&](registry::YoRegistry& current) {
      if (!namespaceEnding.has_value() || current.getNamespace().endsWith(*namespaceEnding))
      {
         variable::YoVariable* candidate = current.getVariable(name);
         if (candidate != nullptr && (!predicate || predicate(candidate)))
            result.push_back(candidate);
      }

      for (registry::YoRegistry* child : current.getChildRegistries())
         recurse(*child);
   };
   recurse(registry);

   return result;
}

variable::YoVariable* findVariable(const VariablePredicate& filter, registry::YoVariableHolder& yoVariableHolder)
{
   for (variable::YoVariable* candidate : yoVariableHolder.getVariables())
   {
      if (filter(candidate))
         return candidate;
   }

   for (registry::YoVariableHolder* child : yoVariableHolder.getChildren())
   {
      variable::YoVariable* result = findVariable(filter, *child);
      if (result != nullptr)
         return result;
   }

   return nullptr;
}

std::vector<variable::YoVariable*> filterVariables(const VariablePredicate& filter, registry::YoVariableHolder& yoVariableHolder)
{
   std::vector<variable::YoVariable*> result;

   std::function<void(registry::YoVariableHolder&)> recurse = [&](registry::YoVariableHolder& holder) {
      for (variable::YoVariable* candidate : holder.getVariables())
      {
         if (filter(candidate))
            result.push_back(candidate);
      }

      for (registry::YoVariableHolder* child : holder.getChildren())
         recurse(*child);
   };
   recurse(yoVariableHolder);

   return result;
}

std::vector<registry::YoRegistry*> filterRegistries(const RegistryPredicate& filter, registry::YoRegistry& registry)
{
   std::vector<registry::YoRegistry*> result;

   std::function<void(registry::YoRegistry&)> recurse = [&](registry::YoRegistry& current) {
      if (filter(&current))
         result.push_back(&current);

      for (registry::YoRegistry* child : current.getChildRegistries())
         recurse(*child);
   };
   recurse(registry);

   return result;
}

VariablePredicate regularExpressionFilter(const std::vector<std::string>& regularExpressions)
{
   std::vector<std::regex> patterns;
   patterns.reserve(regularExpressions.size());
   for (const std::string& expression : regularExpressions)
      patterns.emplace_back(expression);

   return [patterns](variable::YoVariable* candidate) {
      for (const std::regex& pattern : patterns)
      {
         if (std::regex_match(candidate->getName(), pattern))
            return true;
      }
      return false;
   };
}
}
