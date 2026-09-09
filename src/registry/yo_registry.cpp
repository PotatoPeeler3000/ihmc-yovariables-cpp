#include "ihmc/yovariables/registry/yo_registry.h"

#include <algorithm>
#include <stdexcept>

#include "ihmc/yovariables/exceptions/illegal_operation_exception.h"
#include "ihmc/yovariables/exceptions/name_collision_exception.h"
#include "ihmc/yovariables/tools/yo_search_tools.h"
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

/** Concrete Change reported to YoRegistryChangedListeners; see YoRegistryChangedListener::Change. */
class YoRegistry::RegistryChange : public listener::YoRegistryChangedListener::Change
{
public:
   RegistryChange(YoRegistry* source, YoRegistry* targetParentRegistry, YoRegistry* targetRegistry, variable::YoVariable* targetVariable, ChangeType type)
      : source_(source), targetParentRegistry_(targetParentRegistry), targetRegistry_(targetRegistry), targetVariable_(targetVariable), type_(type)
   {
   }

   bool wasRegistryAdded() const override { return type_ == ChangeType::REGISTRY_ADDED; }
   bool wasRegistryRemoved() const override { return type_ == ChangeType::REGISTRY_REMOVED; }
   bool wasVariableAdded() const override { return type_ == ChangeType::VARIABLE_ADDED; }
   bool wasVariableRemoved() const override { return type_ == ChangeType::VARIABLE_REMOVED; }
   bool wasCleared() const override { return type_ == ChangeType::CLEARED; }
   YoRegistry* getSource() const override { return source_; }
   YoRegistry* getTargetParentRegistry() const override { return targetParentRegistry_; }
   YoRegistry* getTargetRegistry() const override { return targetRegistry_; }
   variable::YoVariable* getTargetVariable() const override { return targetVariable_; }

private:
   YoRegistry* source_;
   YoRegistry* targetParentRegistry_;
   YoRegistry* targetRegistry_;
   variable::YoVariable* targetVariable_;
   ChangeType type_;
};

YoRegistry::YoRegistry(const std::string& name) : name_(name), namespace_(name.empty() ? "?" : name)
{
   if (name.empty())
      throw std::invalid_argument("Cannot create a registry without a name.");

   tools::checkForIllegalCharacters(name);
}

const std::string& YoRegistry::getName() const
{
   return name_;
}

const YoNamespace& YoRegistry::getNamespace() const
{
   return namespace_;
}

void YoRegistry::setRestrictionLevel(YoRegistryRestrictionLevel newLevel)
{
   if (static_cast<int>(restrictionLevel_) > static_cast<int>(newLevel))
      throw std::invalid_argument("Cannot reduce restriction level.");

   restrictionLevel_ = newLevel;

   for (YoRegistry* child : children_)
   {
      if (static_cast<int>(child->restrictionLevel_) < static_cast<int>(newLevel))
         child->setRestrictionLevel(newLevel);
   }
}

YoRegistryRestrictionLevel YoRegistry::getRestrictionLevel() const
{
   return restrictionLevel_;
}

void YoRegistry::destroy()
{
   if (!isRoot() && restrictionLevel_ != YoRegistryRestrictionLevel::FULLY_MUTABLE)
      throw exceptions::IllegalOperationException("Cannot clear a registry that is not the root and that does not have appropriate restriction level.");
   detachFromParent();
   destroyInternal(false);
   notifyListeners(nullptr, nullptr, nullptr, ChangeType::CLEARED);
   changedListeners_.clear();
}

void YoRegistry::destroyInternal(bool clearListeners)
{
   for (variable::YoVariable* variable : variables_)
      variable->destroy();
   variables_.clear();
   nameToVariableMap_.clear();
   parameters_.clear();
   for (YoRegistry* child : children_)
      child->destroyInternal(true);
   children_.clear();
   nameToChildMap_.clear();
   restrictionLevel_ = YoRegistryRestrictionLevel::FULLY_MUTABLE;
   if (clearListeners)
      changedListeners_.clear();
}

void YoRegistry::addListener(listener::YoRegistryChangedListener* listenerToAdd)
{
   changedListeners_.push_back(listenerToAdd);
}

void YoRegistry::removeListeners()
{
   changedListeners_.clear();
}

bool YoRegistry::removeListener(listener::YoRegistryChangedListener* listenerToRemove)
{
   auto it = std::find(changedListeners_.begin(), changedListeners_.end(), listenerToRemove);
   if (it == changedListeners_.end())
      return false;
   changedListeners_.erase(it);
   return true;
}

void YoRegistry::addVariable(variable::YoVariable* variable)
{
   if (!isAdditionAllowed(restrictionLevel_))
      throw exceptions::IllegalOperationException("Cannot add variables to this registry: " + namespace_.getName());

   std::string variableName = toLower(variable->getName());

   auto existing = nameToVariableMap_.find(variableName);
   if (existing != nameToVariableMap_.end())
   {
      if (existing->second == variable)
         return;
      throw exceptions::NameCollisionException("Name collision for new variable: " + variableName + ". Parent name space = " + namespace_.getName());
   }

   if (variable->getRegistry() != nullptr)
      variable->getRegistry()->removeVariable(variable);

   variables_.push_back(variable);
   nameToVariableMap_[variableName] = variable;
   variable->setRegistry(this);

   if (variable->isParameter())
      parameters_.push_back(variable->getParameter());

   notifyListeners(this, nullptr, variable, ChangeType::VARIABLE_ADDED);
}

void YoRegistry::removeVariable(variable::YoVariable* variable)
{
   std::string variableName = toLower(variable->getName());
   auto it = nameToVariableMap_.find(variableName);
   if (it == nameToVariableMap_.end() || it->second != variable)
      return;

   if (!isRemovalAllowed(restrictionLevel_))
      throw exceptions::IllegalOperationException("Cannot remove variables from this registry: " + namespace_.getName());

   variable->setRegistry(nullptr);
   variables_.erase(std::remove(variables_.begin(), variables_.end(), variable), variables_.end());
   nameToVariableMap_.erase(it);

   if (variable->isParameter())
   {
      auto parameterIt = std::find(parameters_.begin(), parameters_.end(), variable->getParameter());
      if (parameterIt != parameters_.end())
         parameters_.erase(parameterIt);
   }

   notifyListeners(this, nullptr, variable, ChangeType::VARIABLE_REMOVED);
}

void YoRegistry::addChild(YoRegistry* child, bool notify)
{
   if (child == nullptr)
      return;
   if (child == this)
      throw exceptions::IllegalOperationException("Cannot register a registry as a child of itself, registry: " + namespace_.getName());

   if (!isAdditionAllowed(restrictionLevel_))
      throw exceptions::IllegalOperationException("Cannot add children to this registry: " + namespace_.getName());

   std::string childName = toLower(child->getName());

   auto existing = nameToChildMap_.find(childName);
   if (existing != nameToChildMap_.end())
   {
      if (existing->second == child)
         return;
      throw exceptions::NameCollisionException("Name collision for new child: " + childName + ". Parent name space = " + namespace_.getName());
   }

   child->detachFromParent();
   child->parent_ = this;
   child->setParentNamespace(&namespace_);
   if (static_cast<int>(child->getRestrictionLevel()) < static_cast<int>(restrictionLevel_))
      child->setRestrictionLevel(restrictionLevel_);

   children_.push_back(child);
   nameToChildMap_[childName] = child;

   if (notify)
      notifyListeners(this, child, nullptr, ChangeType::REGISTRY_ADDED);
}

void YoRegistry::removeChild(YoRegistry* child)
{
   if (child == nullptr || child->getParent() != this)
      return;

   if (!isRemovalAllowed(restrictionLevel_))
      throw exceptions::IllegalOperationException("Cannot remove children from this registry: " + namespace_.getName());

   std::string childName = toLower(child->getName());

   child->parent_ = nullptr;
   child->setParentNamespace(nullptr);

   children_.erase(std::remove(children_.begin(), children_.end(), child), children_.end());
   nameToChildMap_.erase(childName);

   notifyListeners(this, child, nullptr, ChangeType::REGISTRY_REMOVED);
}

void YoRegistry::detachFromParent()
{
   if (parent_ == nullptr)
      return;
   parent_->removeChild(this);
}

void YoRegistry::setParentNamespace(const YoNamespace* parentNamespace)
{
   if (parentNamespace == nullptr)
      namespace_ = YoNamespace(name_);
   else
      namespace_ = parentNamespace->append(name_);
   namespace_.checkSanity();
   for (YoRegistry* child : children_)
      child->setParentNamespace(&namespace_);
   for (variable::YoVariable* variable : variables_)
      variable->resetFullName();
}

bool YoRegistry::hasParameters() const
{
   return !parameters_.empty();
}

bool YoRegistry::hasParametersDeep() const
{
   if (hasParameters())
      return true;
   for (YoRegistry* child : children_)
   {
      if (child->hasParametersDeep())
         return true;
   }
   return false;
}

bool YoRegistry::isRoot() const
{
   return parent_ == nullptr;
}

YoRegistry* YoRegistry::getRoot()
{
   return isRoot() ? this : parent_->getRoot();
}

YoRegistry* YoRegistry::getParent()
{
   return parent_;
}

YoRegistry* YoRegistry::getChild(const std::string& name) const
{
   auto it = nameToChildMap_.find(toLower(name));
   return it == nameToChildMap_.end() ? nullptr : it->second;
}

const std::vector<YoRegistry*>& YoRegistry::getChildRegistries() const
{
   return children_;
}

variable::YoVariable* YoRegistry::getVariable(const std::string& name) const
{
   auto it = nameToVariableMap_.find(toLower(name));
   return it == nameToVariableMap_.end() ? nullptr : it->second;
}

variable::YoVariable* YoRegistry::getVariable(std::size_t index) const
{
   return variables_.at(index);
}

parameters::YoParameter* YoRegistry::getParameter(const std::string& name) const
{
   variable::YoVariable* variable = getVariable(name);
   if (variable != nullptr && variable->isParameter())
      return variable->getParameter();
   return nullptr;
}

const std::vector<parameters::YoParameter*>& YoRegistry::getParameters() const
{
   return parameters_;
}

std::vector<variable::YoVariable*> YoRegistry::collectSubtreeVariables() const
{
   std::vector<variable::YoVariable*> result;
   result.insert(result.end(), variables_.begin(), variables_.end());
   for (YoRegistry* child : children_)
   {
      std::vector<variable::YoVariable*> childVariables = child->collectSubtreeVariables();
      result.insert(result.end(), childVariables.begin(), childVariables.end());
   }
   return result;
}

std::vector<parameters::YoParameter*> YoRegistry::collectSubtreeParameters() const
{
   std::vector<parameters::YoParameter*> result;
   result.insert(result.end(), parameters_.begin(), parameters_.end());
   for (YoRegistry* child : children_)
   {
      std::vector<parameters::YoParameter*> childParameters = child->collectSubtreeParameters();
      result.insert(result.end(), childParameters.begin(), childParameters.end());
   }
   return result;
}

std::vector<YoRegistry*> YoRegistry::collectSubtreeRegistries()
{
   std::vector<YoRegistry*> result;
   result.push_back(this);
   for (YoRegistry* child : children_)
   {
      std::vector<YoRegistry*> childRegistries = child->collectSubtreeRegistries();
      result.insert(result.end(), childRegistries.begin(), childRegistries.end());
   }
   return result;
}

YoRegistry* YoRegistry::findRegistry(const std::string& name)
{
   std::size_t separatorIndex = name.rfind(tools::kNamespaceSeparator);
   if (separatorIndex == std::string::npos)
      return findRegistry(std::nullopt, name);
   return findRegistry(name.substr(0, separatorIndex), name.substr(separatorIndex + 1));
}

YoRegistry* YoRegistry::findRegistry(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   return tools::findFirstRegistry(namespaceEnding, name, nullptr, *this);
}

YoRegistry* YoRegistry::findRegistry(const YoNamespace& namespaceEnding)
{
   if (namespace_.endsWith(namespaceEnding))
      return this;

   for (YoRegistry* child : children_)
   {
      YoRegistry* result = child->findRegistry(namespaceEnding);
      if (result != nullptr)
         return result;
   }

   return nullptr;
}

std::size_t YoRegistry::countNumberOfVariables(const std::optional<std::string>& parentNamespace, const std::string& name) const
{
   std::size_t count = 0;

   if (!parentNamespace.has_value() || namespace_.endsWith(*parentNamespace))
   {
      if (nameToVariableMap_.find(toLower(name)) != nameToVariableMap_.end())
         count++;
   }

   for (YoRegistry* child : children_)
      count += child->countNumberOfVariables(parentNamespace, name);

   return count;
}

std::size_t YoRegistry::getNumberOfVariables() const
{
   return variables_.size();
}

std::size_t YoRegistry::getNumberOfVariablesDeep() const
{
   std::size_t count = variables_.size();
   for (YoRegistry* child : children_)
      count += child->getNumberOfVariablesDeep();
   return count;
}

std::vector<variable::YoVariable*> YoRegistry::getVariables()
{
   return variables_;
}

std::vector<YoVariableHolder*> YoRegistry::getChildren()
{
   std::vector<YoVariableHolder*> result;
   result.reserve(children_.size());
   for (YoRegistry* child : children_)
      result.push_back(child);
   return result;
}

variable::YoVariable* YoRegistry::findVariable(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   return tools::findFirstVariable(namespaceEnding, name, nullptr, *this);
}

std::vector<variable::YoVariable*> YoRegistry::findVariables(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   return tools::findVariables(namespaceEnding, name, nullptr, *this);
}

std::vector<variable::YoVariable*> YoRegistry::findVariables(const YoNamespace& namespaceValue)
{
   YoRegistry* target = findRegistry(namespaceValue);
   return target == nullptr ? std::vector<variable::YoVariable*>{} : target->getVariables();
}

bool YoRegistry::hasUniqueVariable(const std::optional<std::string>& namespaceEnding, const std::string& name)
{
   tools::checkNameDoesNotContainSeparator(name);
   return countNumberOfVariables(namespaceEnding, name) == 1;
}

void YoRegistry::notifyListeners(YoRegistry* targetParentRegistry, YoRegistry* targetRegistry, variable::YoVariable* targetVariable, ChangeType type)
{
   if (!changedListeners_.empty())
   {
      RegistryChange change(this, targetParentRegistry, targetRegistry, targetVariable, type);
      for (listener::YoRegistryChangedListener* listenerToNotify : changedListeners_)
         listenerToNotify->changed(change);
   }

   if (parent_ != nullptr)
      parent_->notifyListeners(targetParentRegistry, targetRegistry, targetVariable, type);
}

bool YoRegistry::operator==(const YoRegistry& other) const
{
   if (this == &other)
      return true;

   if (namespace_ != other.namespace_)
      return false;

   if (variables_.size() != other.variables_.size())
      return false;

   for (variable::YoVariable* variable : variables_)
   {
      if (other.nameToVariableMap_.find(toLower(variable->getName())) == other.nameToVariableMap_.end())
         return false;
   }

   if (children_.size() != other.children_.size())
      return false;

   for (YoRegistry* child : children_)
   {
      if (other.nameToChildMap_.find(toLower(child->getName())) == other.nameToChildMap_.end())
         return false;
   }

   return true;
}

std::string YoRegistry::toString() const
{
   return namespace_.getName();
}
}
