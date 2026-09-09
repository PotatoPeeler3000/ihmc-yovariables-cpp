#include "ihmc/yovariables/tools/yo_tools.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>

#include "ihmc/yovariables/exceptions/illegal_name_exception.h"
#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/tools/yo_search_tools.h"

namespace ihmc::yovariables::tools
{
void checkForIllegalCharacters(const std::string& name)
{
   if (name.find_first_of(kIllegalCharacters) != std::string::npos)
   {
      throw exceptions::IllegalNameException(name + " contains at least one illegal character.");
   }
}

void checkNameDoesNotContainSeparator(const std::string& name)
{
   if (name.find(kNamespaceSeparator) != std::string::npos)
      throw exceptions::IllegalNameException("The name cannot contain '" + std::string(kNamespaceSeparatorString) + "'. Was: " + name);
}

std::vector<std::string> splitName(const std::string& name)
{
   std::vector<std::string> result;
   std::size_t start = 0;

   while (true)
   {
      std::size_t separatorIndex = name.find(kNamespaceSeparator, start);
      if (separatorIndex == std::string::npos)
      {
         result.push_back(name.substr(start));
         break;
      }
      result.push_back(name.substr(start, separatorIndex - start));
      start = separatorIndex + 1;
   }

   return result;
}

std::string joinNames(const std::vector<std::string>& subNames)
{
   std::string result;
   for (std::size_t i = 0; i < subNames.size(); i++)
   {
      if (i > 0)
         result += kNamespaceSeparator;
      result += subNames[i];
   }
   return result;
}

void checkNamespaceSanity(const registry::YoNamespace& namespaceToCheck)
{
   const std::vector<std::string>& subNames = namespaceToCheck.getSubNames();

   for (const std::string& subName : subNames)
   {
      if (subName.empty())
         throw exceptions::IllegalNameException("The namespace has 1+ empty subname.\nNamespace: " + namespaceToCheck.getName());
   }

   for (const std::string& subName : subNames)
   {
      if (subName.find(kNamespaceSeparator) != std::string::npos)
         throw exceptions::IllegalNameException("A sub-name can not contain the separator character '" + std::string(kNamespaceSeparatorString) + "'.");
   }

   if (joinNames(subNames) != namespaceToCheck.getName())
   {
      throw exceptions::IllegalNameException("The namespace has inconsistent sub-names.\nNamespace: " + namespaceToCheck.getName());
   }

   std::unordered_set<std::string> uniqueSubNames(subNames.begin(), subNames.end());
   if (uniqueSubNames.size() != subNames.size())
   {
      throw exceptions::IllegalNameException("The namespace has duplicate sub-names.\nNamespace: " + namespaceToCheck.getName());
   }
}

registry::YoNamespace concatenate(const registry::YoNamespace& namespaceA, const registry::YoNamespace& namespaceB)
{
   std::vector<std::string> subNames;
   subNames.reserve(namespaceA.size() + namespaceB.size());
   subNames.insert(subNames.end(), namespaceA.getSubNames().begin(), namespaceA.getSubNames().end());
   subNames.insert(subNames.end(), namespaceB.getSubNames().begin(), namespaceB.getSubNames().end());
   return registry::YoNamespace(std::move(subNames));
}

registry::YoNamespace concatenate(const registry::YoNamespace& namespaceValue, const std::string& name)
{
   std::vector<std::string> nameParts = splitName(name);
   std::vector<std::string> subNames;
   subNames.reserve(namespaceValue.size() + nameParts.size());
   subNames.insert(subNames.end(), namespaceValue.getSubNames().begin(), namespaceValue.getSubNames().end());
   subNames.insert(subNames.end(), nameParts.begin(), nameParts.end());
   return registry::YoNamespace(std::move(subNames));
}

registry::YoNamespace concatenate(const std::string& name, const registry::YoNamespace& namespaceValue)
{
   std::vector<std::string> nameParts = splitName(name);
   std::vector<std::string> subNames;
   subNames.reserve(nameParts.size() + namespaceValue.size());
   subNames.insert(subNames.end(), nameParts.begin(), nameParts.end());
   subNames.insert(subNames.end(), namespaceValue.getSubNames().begin(), namespaceValue.getSubNames().end());
   return registry::YoNamespace(std::move(subNames));
}

registry::YoNamespace concatenate(const std::string& nameA, const std::string& nameB)
{
   std::vector<std::string> subNamesA = splitName(nameA);
   std::vector<std::string> subNamesB = splitName(nameB);
   std::vector<std::string> subNames;
   subNames.reserve(subNamesA.size() + subNamesB.size());
   subNames.insert(subNames.end(), subNamesA.begin(), subNamesA.end());
   subNames.insert(subNames.end(), subNamesB.begin(), subNamesB.end());
   return registry::YoNamespace(std::move(subNames));
}

std::string toShortName(const std::string& name)
{
   std::size_t separatorIndex = name.rfind(kNamespaceSeparator);
   if (separatorIndex == std::string::npos)
      return name;
   return name.substr(separatorIndex + 1);
}

namespace
{
/** Truncates original to length (appending placeholder) if too long, else right-pads with spaces. */
std::string trimOrPadToLength(const std::string& original, int length, const std::string& placeholder)
{
   if (static_cast<int>(original.size()) > length)
      return original.substr(0, length - static_cast<int>(placeholder.size())) + placeholder;

   std::string padded = original;
   padded.resize(static_cast<std::size_t>(length), ' ');
   return padded;
}
} // namespace

std::string getRegistryInfo(const registry::YoRegistry& registry, int maxNameLength)
{
   int variables = static_cast<int>(registry.getNumberOfVariables());
   int children = static_cast<int>(registry.getChildRegistries().size());
   constexpr int maxPropertyLength = 17; // "Variables: " is 11 chars, leaving 6 for the integer.

   std::string variableString = trimOrPadToLength("Variables: " + std::to_string(variables), maxPropertyLength, "...");
   std::string childrenString = trimOrPadToLength("Children: " + std::to_string(children), maxPropertyLength, "...");

   // "YoRegistry", not a reflective class name: nothing in this port subclasses YoRegistry.
   std::string name = trimOrPadToLength("YoRegistry " + registry.getNamespace().getName(), maxNameLength, "...");

   return name + "\t" + variableString + "\t" + childrenString;
}

void printStatistics(const std::function<bool(const registry::YoRegistry&)>& filter, registry::YoRegistry& root,
                      const std::function<std::string(const registry::YoRegistry&)>& registryInfoFunction, std::ostream& printStream)
{
   std::vector<registry::YoRegistry*> registriesOfInterest =
      filterRegistries([&filter](registry::YoRegistry* candidate) { return filter(*candidate); }, root);

   std::sort(registriesOfInterest.begin(), registriesOfInterest.end(), [](registry::YoRegistry* a, registry::YoRegistry* b) {
      return a->getNumberOfVariables() > b->getNumberOfVariables();
   });

   printStream << "YoTools: Printing descendants of " << root.getName() << " registry.\n";
   printStream << "Total number of variables: " << root.getNumberOfVariablesDeep() << "\n";
   printStream << "Sorting by number of variables.\n";
   for (registry::YoRegistry* registryOfInterest : registriesOfInterest)
      printStream << registryInfoFunction(*registryOfInterest) << "\n";
}

void printStatistics(int minVariablesToPrint, int minChildrenToPrint, registry::YoRegistry& root,
                      const std::function<std::string(const registry::YoRegistry&)>& registryInfoFunction, std::ostream& printStream)
{
   printStatistics(
      [minVariablesToPrint, minChildrenToPrint](const registry::YoRegistry& candidate) {
         return static_cast<int>(candidate.getNumberOfVariables()) >= minVariablesToPrint
                || static_cast<int>(candidate.getChildRegistries().size()) >= minChildrenToPrint;
      },
      root,
      registryInfoFunction,
      printStream);
}

void printStatistics(int minVariablesToPrint, int minChildrenToPrint, registry::YoRegistry& root)
{
   printStatistics(minVariablesToPrint, minChildrenToPrint, root, [](const registry::YoRegistry& r) { return getRegistryInfo(r); }, std::cout);
}
}
