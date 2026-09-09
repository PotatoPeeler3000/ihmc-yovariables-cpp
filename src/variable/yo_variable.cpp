#include "ihmc/yovariables/variable/yo_variable.h"

#include <algorithm>

#include "ihmc/yovariables/exceptions/name_collision_exception.h"
#include "ihmc/yovariables/listener/yo_variable_changed_listener.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/tools/yo_tools.h"

namespace ihmc::yovariables::variable
{
YoVariable::YoVariable(YoVariableType type, const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : name_(name), description_(description), type_(type)
{
   tools::checkForIllegalCharacters(name);
   setRegistry(registry);
}

void YoVariable::setRegistry(registry::YoRegistry* registry)
{
   if (registry == registry_)
      return;

   registry::YoRegistry* oldRegistry = registry_;
   registry_ = nullptr;

   if (oldRegistry != nullptr && oldRegistry->hasVariable(name_))
      oldRegistry->removeVariable(this);

   if (registry != nullptr)
   {
      YoVariable* existingVariable = registry->getVariable(name_);
      if (existingVariable != nullptr && existingVariable != this)
         throw exceptions::NameCollisionException("A variable named " + name_ + " already exists in the registry " + registry->getNamespace().getName()
                                                    + ".");
      registry->addVariable(this);
   }

   registry_ = registry;
   resetFullName();
}

void YoVariable::resetFullName()
{
   fullName_.reset();
}

registry::YoRegistry* YoVariable::getRegistry() const
{
   return registry_;
}

const std::string& YoVariable::getName() const
{
   return name_;
}

const std::string& YoVariable::getDescription() const
{
   return description_;
}

const registry::YoNamespace& YoVariable::getFullName()
{
   if (!fullName_.has_value())
   {
      if (registry_ == nullptr)
         fullName_ = registry::YoNamespace(name_);
      else
         fullName_ = registry_->getNamespace().append(name_);
   }

   return *fullName_;
}

std::string YoVariable::getFullNameString()
{
   return getFullName().getName();
}

const registry::YoNamespace* YoVariable::getNamespace() const
{
   return registry_ == nullptr ? nullptr : &registry_->getNamespace();
}

void YoVariable::setVariableBounds(double lowerBound, double upperBound)
{
   lowerBound_ = lowerBound;
   upperBound_ = upperBound;
}

double YoVariable::getLowerBound() const
{
   return lowerBound_;
}

double YoVariable::getUpperBound() const
{
   return upperBound_;
}

YoVariableType YoVariable::getType() const
{
   return type_;
}

void YoVariable::addListener(listener::YoVariableChangedListener* listenerToAdd)
{
   changedListeners_.push_back(listenerToAdd);
}

void YoVariable::removeListeners()
{
   changedListeners_.clear();
}

const std::vector<listener::YoVariableChangedListener*>& YoVariable::getListeners() const
{
   return changedListeners_;
}

bool YoVariable::removeListener(listener::YoVariableChangedListener* listenerToRemove)
{
   auto it = std::find(changedListeners_.begin(), changedListeners_.end(), listenerToRemove);
   if (it == changedListeners_.end())
      return false;
   changedListeners_.erase(it);
   return true;
}

void YoVariable::notifyListeners()
{
   for (listener::YoVariableChangedListener* listenerToNotify : changedListeners_)
      listenerToNotify->changed(*this);
}

bool YoVariable::setValueFromDouble(double value)
{
   return setValueFromDouble(value, true);
}

bool YoVariable::setValueFromLongBits(std::int64_t value)
{
   return setValueFromLongBits(value, true);
}

std::string YoVariable::getValueAsString()
{
   return getValueAsString(std::nullopt);
}

bool YoVariable::parseValue(const std::string& valueAsString)
{
   return parseValue(valueAsString, true);
}

void YoVariable::destroy()
{
   setRegistry(nullptr);
   changedListeners_.clear();
}
}
