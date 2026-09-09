#include "ihmc/yovariables/tools/yo_tools.h"

#include <unordered_set>

#include "ihmc/yovariables/exceptions/illegal_name_exception.h"
#include "ihmc/yovariables/registry/yo_namespace.h"

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
}
