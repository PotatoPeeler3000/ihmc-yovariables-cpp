#include "ihmc/yovariables/parameters/yo_parameter.h"

#include <algorithm>

#include "ihmc/yovariables/exceptions/illegal_operation_exception.h"
#include "ihmc/yovariables/listener/yo_parameter_changed_listener.h"
#include "ihmc/yovariables/listener/yo_variable_changed_listener.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::parameters
{
/** Delegates YoVariableChangedListener notifications to this parameter's YoParameterChangedListeners. */
class YoParameter::ChangedListenerHolder : public listener::YoVariableChangedListener
{
public:
   void changed(variable::YoVariable& source) override
   {
      parameters::YoParameter* parameter = source.getParameter();
      for (listener::YoParameterChangedListener* listenerToNotify : changedListeners_)
         listenerToNotify->changed(*parameter);
   }

   void addListener(listener::YoParameterChangedListener* listenerToAdd)
   {
      changedListeners_.push_back(listenerToAdd);
   }

   void removeListeners()
   {
      changedListeners_.clear();
   }

   bool removeListener(listener::YoParameterChangedListener* listenerToRemove)
   {
      auto it = std::find(changedListeners_.begin(), changedListeners_.end(), listenerToRemove);
      if (it == changedListeners_.end())
         return false;
      changedListeners_.erase(it);
      return true;
   }

   const std::vector<listener::YoParameterChangedListener*>& getListeners() const
   {
      return changedListeners_;
   }

private:
   std::vector<listener::YoParameterChangedListener*> changedListeners_;
};

YoParameter::~YoParameter()
{
   delete changedListenerHolder_;
}

std::string YoParameter::getName() const
{
   return getVariable().getName();
}

std::string YoParameter::getDescription() const
{
   return getVariable().getDescription();
}

const registry::YoNamespace& YoParameter::getFullName() const
{
   return getVariable().getFullName();
}

std::string YoParameter::getFullNameString() const
{
   return getVariable().getFullNameString();
}

const registry::YoNamespace* YoParameter::getNamespace() const
{
   return getVariable().getNamespace();
}

void YoParameter::setParameterBounds(double lowerBound, double upperBound)
{
   getVariable().setVariableBounds(lowerBound, upperBound);
}

double YoParameter::getLowerBound() const
{
   return getVariable().getLowerBound();
}

double YoParameter::getUpperBound() const
{
   return getVariable().getUpperBound();
}

void YoParameter::addListener(listener::YoParameterChangedListener* listenerToAdd)
{
   if (!changedListenerHolder_)
   {
      changedListenerHolder_ = new ChangedListenerHolder();
      getVariable().addListener(changedListenerHolder_);
   }

   changedListenerHolder_->addListener(listenerToAdd);
}

void YoParameter::removeListeners()
{
   if (changedListenerHolder_)
      changedListenerHolder_->removeListeners();
}

const std::vector<listener::YoParameterChangedListener*>& YoParameter::getListeners() const
{
   static const std::vector<listener::YoParameterChangedListener*> empty;
   return changedListenerHolder_ ? changedListenerHolder_->getListeners() : empty;
}

bool YoParameter::removeListener(listener::YoParameterChangedListener* listenerToRemove)
{
   if (!changedListenerHolder_)
      return false;
   return changedListenerHolder_->removeListener(listenerToRemove);
}

std::string YoParameter::getValueAsString()
{
   checkLoaded();
   return getVariable().getValueAsString();
}

void YoParameter::setToString(const std::string& valueString)
{
   getVariable().parseValue(valueString);
}

void YoParameter::load(const std::string& valueString)
{
   loadStatus_ = ParameterLoadStatus::LOADED;
   setToString(valueString);
}

void YoParameter::loadDefault()
{
   loadStatus_ = ParameterLoadStatus::DEFAULT;
   setToDefault();
}

ParameterLoadStatus YoParameter::getLoadStatus() const
{
   return loadStatus_;
}

void YoParameter::setLoadStatus(ParameterLoadStatus status)
{
   loadStatus_ = status;
}

void YoParameter::checkLoaded() const
{
   if (!isLoaded())
      throw exceptions::IllegalOperationException("The parameter " + getFullNameString() + " has not been loaded. This is required to enable its use.");
}

bool YoParameter::isLoaded() const
{
   return loadStatus_ != ParameterLoadStatus::UNLOADED;
}

std::string YoParameter::toString()
{
   return getVariable().toString();
}
}
