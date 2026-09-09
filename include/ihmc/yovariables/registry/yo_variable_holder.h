#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ihmc/yovariables/tools/yo_search_tools.h"
#include "ihmc/yovariables/tools/yo_tools.h"

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::registry
{
class YoNamespace;

/** Base interface for a class that manages a collection of YoVariables. */
class YoVariableHolder
{
public:
   virtual ~YoVariableHolder() = default;

   /**
    * All the variables directly in this holder. A YoRegistry only returns the variables it
    * directly holds, without recursing into its subtree.
    */
   virtual std::vector<variable::YoVariable*> getVariables() = 0;

   /** The children directly in this holder. */
   virtual std::vector<YoVariableHolder*> getChildren()
   {
      return {};
   }

   /** Finds the first variable matching name, splitting a trailing namespace off name if present. */
   virtual variable::YoVariable* findVariable(const std::string& name)
   {
      auto [namespaceEnding, shortName] = splitTrailingName(name);
      return findVariable(namespaceEnding, shortName);
   }

   /**
    * Finds the first variable matching name and namespaceEnding (optional; unset searches by name
    * only).
    */
   virtual variable::YoVariable* findVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) = 0;

   /** Finds all variables matching name, splitting a trailing namespace off name if present. */
   virtual std::vector<variable::YoVariable*> findVariables(const std::string& name)
   {
      auto [namespaceEnding, shortName] = splitTrailingName(name);
      return findVariables(namespaceEnding, shortName);
   }

   virtual std::vector<variable::YoVariable*> findVariables(const std::optional<std::string>& namespaceEnding, const std::string& name) = 0;

   /** Finds all variables registered under the registry matching namespaceValue exactly. */
   virtual std::vector<variable::YoVariable*> findVariables(const YoNamespace& namespaceValue) = 0;

   /** All variables in this holder's subtree for which filter returns true. */
   virtual std::vector<variable::YoVariable*> filterVariables(const tools::VariablePredicate& filter)
   {
      return tools::filterVariables(filter, *this);
   }

   /** True if there is exactly one variable matching name (see findVariable(name)). */
   virtual bool hasUniqueVariable(const std::string& name)
   {
      auto [namespaceEnding, shortName] = splitTrailingName(name);
      return hasUniqueVariable(namespaceEnding, shortName);
   }

   virtual bool hasUniqueVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) = 0;

   virtual bool hasVariable(const std::string& name)
   {
      return findVariable(name) != nullptr;
   }

   virtual bool hasVariable(const std::optional<std::string>& namespaceEnding, const std::string& name)
   {
      return findVariable(namespaceEnding, name) != nullptr;
   }

private:
   /** Splits name at its last namespace separator into (namespaceEnding, shortName). */
   static std::pair<std::optional<std::string>, std::string> splitTrailingName(const std::string& name)
   {
      std::size_t separatorIndex = name.rfind(tools::kNamespaceSeparator);
      if (separatorIndex == std::string::npos)
         return {std::nullopt, name};
      return {name.substr(0, separatorIndex), name.substr(separatorIndex + 1)};
   }
};
}
