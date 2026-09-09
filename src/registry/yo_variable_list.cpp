#include "ihmc/yovariables/registry/yo_variable_list.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include "ihmc/yovariables/exceptions/name_collision_exception.h"
#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/tools/yo_tools.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::registry
{
namespace
{
std::string toLower(const std::string& value)
{
   std::string result = value;
   std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
   return result;
}
} // namespace

YoVariableList::YoVariableList(std::string name) : name_(std::move(name))
{
}

YoVariableList::YoVariableList(std::string name, const std::vector<variable::YoVariable*>& variables) : name_(std::move(name))
{
   addAll(variables);
}

const std::string& YoVariableList::getName() const
{
   return name_;
}

bool YoVariableList::isEmpty() const
{
   return variableList_.empty();
}

std::size_t YoVariableList::size() const
{
   return variableList_.size();
}

void YoVariableList::clear()
{
   variableList_.clear();
   simpleNameToVariablesMap_.clear();
}

variable::YoVariable* YoVariableList::get(int index) const
{
   if (index < 0 || static_cast<std::size_t>(index) >= variableList_.size())
      throw std::out_of_range("Index: " + std::to_string(index) + ", Size: " + std::to_string(variableList_.size()));
   return variableList_[static_cast<std::size_t>(index)];
}

bool YoVariableList::registerVariableInMap(variable::YoVariable* variable)
{
   std::string key = toLower(variable->getName());
   auto it = simpleNameToVariablesMap_.find(key);

   if (it == simpleNameToVariablesMap_.end())
   {
      simpleNameToVariablesMap_[key].push_back(variable);
      return true;
   }

   std::vector<variable::YoVariable*>& homonyms = it->second;
   if (std::find(homonyms.begin(), homonyms.end(), variable) != homonyms.end())
      return false;

   for (variable::YoVariable* homonym : homonyms)
   {
      const YoNamespace* homonymNamespace = homonym->getNamespace();
      const YoNamespace* variableNamespace = variable->getNamespace();
      bool sameNamespace = homonymNamespace == variableNamespace || (homonymNamespace != nullptr && variableNamespace != nullptr && *homonymNamespace == *variableNamespace);
      if (sameNamespace)
         throw exceptions::NameCollisionException("Name collision with " + variable->getFullNameString());
   }

   homonyms.push_back(variable);
   return true;
}

bool YoVariableList::add(variable::YoVariable* variable)
{
   if (!registerVariableInMap(variable))
      return false;
   variableList_.push_back(variable);
   return true;
}

void YoVariableList::add(int index, variable::YoVariable* variable)
{
   if (index > static_cast<int>(size()) || index < 0)
      throw std::out_of_range("Index: " + std::to_string(index) + ", Size: " + std::to_string(size()));

   if (!registerVariableInMap(variable))
      return;
   variableList_.insert(variableList_.begin() + index, variable);
}

variable::YoVariable* YoVariableList::set(int index, variable::YoVariable* variable)
{
   variable::YoVariable* previousVariable = get(index);

   if (variable == previousVariable)
      return previousVariable;

   if (!registerVariableInMap(variable))
      return nullptr;

   auto it = simpleNameToVariablesMap_.find(toLower(previousVariable->getName()));
   if (it != simpleNameToVariablesMap_.end())
   {
      std::vector<variable::YoVariable*>& homonyms = it->second;
      homonyms.erase(std::remove(homonyms.begin(), homonyms.end(), previousVariable), homonyms.end());
   }

   variableList_[static_cast<std::size_t>(index)] = variable;
   return previousVariable;
}

variable::YoVariable* YoVariableList::remove(int index)
{
   variable::YoVariable* variable = get(index);

   auto it = simpleNameToVariablesMap_.find(toLower(variable->getName()));
   if (it != simpleNameToVariablesMap_.end())
   {
      std::vector<variable::YoVariable*>& homonyms = it->second;
      homonyms.erase(std::remove(homonyms.begin(), homonyms.end(), variable), homonyms.end());
   }
   variableList_.erase(std::remove(variableList_.begin(), variableList_.end(), variable), variableList_.end());
   return variable;
}

bool YoVariableList::remove(variable::YoVariable* variable)
{
   if (variable == nullptr)
      return false;

   auto it = simpleNameToVariablesMap_.find(toLower(variable->getName()));
   if (it == simpleNameToVariablesMap_.end())
      return false;

   std::vector<variable::YoVariable*>& homonyms = it->second;
   bool modified = false;
   auto homonymIt = std::find(homonyms.begin(), homonyms.end(), variable);
   if (homonymIt != homonyms.end())
   {
      homonyms.erase(homonymIt);
      modified = true;
   }

   auto listIt = std::find(variableList_.begin(), variableList_.end(), variable);
   if (listIt != variableList_.end())
   {
      variableList_.erase(listIt);
      modified = true;
   }

   return modified;
}

void YoVariableList::addAll(const std::vector<variable::YoVariable*>& variables)
{
   for (variable::YoVariable* variable : variables)
      add(variable);
}

void YoVariableList::addAll(const YoVariableList& other)
{
   addAll(other.variableList_);
}

int YoVariableList::indexOf(const variable::YoVariable* variable) const
{
   auto it = std::find(variableList_.begin(), variableList_.end(), variable);
   if (it == variableList_.end())
      return -1;
   return static_cast<int>(std::distance(variableList_.begin(), it));
}

bool YoVariableList::contains(const variable::YoVariable* variable) const
{
   return indexOf(variable) != -1;
}

std::string YoVariableList::toString() const
{
   if (variableList_.size() > 10)
      return name_ + ", contains: " + std::to_string(variableList_.size()) + " variables.";

   std::string result = name_ + ", variables:";
   for (variable::YoVariable* variable : variableList_)
      result += "\n" + variable->toString();
   return result;
}

std::vector<variable::YoVariable*> YoVariableList::getVariables()
{
   return variableList_;
}

variable::YoVariable* YoVariableList::findVariable(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   auto it = simpleNameToVariablesMap_.find(toLower(name));
   if (it == simpleNameToVariablesMap_.end() || it->second.empty())
      return nullptr;

   if (!namespaceEnding.has_value())
      return it->second.front();

   for (variable::YoVariable* candidate : it->second)
   {
      const YoNamespace* candidateNamespace = candidate->getNamespace();
      if (candidateNamespace != nullptr && candidateNamespace->endsWith(*namespaceEnding))
         return candidate;
   }

   return nullptr;
}

int YoVariableList::countNumberOfVariables(const std::optional<std::string>& parentNamespace, const std::string& name) const
{
   auto it = simpleNameToVariablesMap_.find(toLower(name));
   if (it == simpleNameToVariablesMap_.end() || it->second.empty())
      return 0;

   if (!parentNamespace.has_value())
      return static_cast<int>(it->second.size());

   int count = 0;
   for (variable::YoVariable* candidate : it->second)
   {
      const YoNamespace* candidateNamespace = candidate->getNamespace();
      if (candidateNamespace != nullptr && candidateNamespace->endsWith(*parentNamespace))
         count++;
   }
   return count;
}

bool YoVariableList::hasUniqueVariable(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   return countNumberOfVariables(namespaceEnding, name) == 1;
}

std::vector<variable::YoVariable*> YoVariableList::findVariables(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   auto it = simpleNameToVariablesMap_.find(toLower(name));
   if (it == simpleNameToVariablesMap_.end() || it->second.empty())
      return {};

   if (!namespaceEnding.has_value())
      return it->second;

   std::vector<variable::YoVariable*> result;
   for (variable::YoVariable* candidate : it->second)
   {
      const YoNamespace* candidateNamespace = candidate->getNamespace();
      if (candidateNamespace != nullptr && candidateNamespace->endsWith(*namespaceEnding))
         result.push_back(candidate);
   }
   return result;
}

std::vector<variable::YoVariable*> YoVariableList::findVariables(const YoNamespace& namespaceValue)
{
   std::vector<variable::YoVariable*> result;
   for (variable::YoVariable* variable : variableList_)
   {
      const YoNamespace* variableNamespace = variable->getNamespace();
      if (variableNamespace != nullptr && *variableNamespace == namespaceValue)
         result.push_back(variable);
   }
   return result;
}
}
