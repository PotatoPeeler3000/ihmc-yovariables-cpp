#include "ihmc/yovariables/parameters/double_parameter.h"

#include <cmath>

namespace ihmc::yovariables::parameters
{
DoubleParameter::DoubleParameter(const std::string& name, registry::YoRegistry* registry) : DoubleParameter(name, "", registry, std::nan(""))
{
}

DoubleParameter::DoubleParameter(const std::string& name, registry::YoRegistry* registry, double lowerBound, double upperBound)
   : DoubleParameter(name, "", registry, lowerBound, upperBound)
{
}

DoubleParameter::DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : DoubleParameter(name, description, registry, std::nan(""))
{
}

DoubleParameter::DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, double lowerBound,
                                  double upperBound)
   : DoubleParameter(name, description, registry, std::nan(""), lowerBound, upperBound)
{
}

DoubleParameter::DoubleParameter(const std::string& name, registry::YoRegistry* registry, double initialValue)
   : DoubleParameter(name, "", registry, initialValue)
{
}

DoubleParameter::DoubleParameter(const std::string& name, registry::YoRegistry* registry, double initialValue, double lowerBound, double upperBound)
   : DoubleParameter(name, "", registry, initialValue, lowerBound, upperBound)
{
}

DoubleParameter::DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, double initialValue)
   : DoubleParameter(name, description, registry, initialValue, kDefaultSuggestedMinimum, kDefaultSuggestedMaximum)
{
}

DoubleParameter::DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, double initialValue,
                                  double lowerBound, double upperBound)
   : value_(std::make_unique<BackingVariable>(*this, name, description, registry)), initialValue_(initialValue)
{
   setParameterBounds(lowerBound, upperBound);
}

double DoubleParameter::getValue() const
{
   checkLoaded();
   return value_->getDoubleValue();
}

void DoubleParameter::setParameterBounds(double lowerBound, double upperBound)
{
   YoParameter::setParameterBounds(lowerBound, upperBound);
}

variable::YoDouble& DoubleParameter::getVariable() const
{
   return *value_;
}

void DoubleParameter::setToDefault()
{
   value_->set(initialValue_);
}

DoubleParameter::BackingVariable::BackingVariable(DoubleParameter& owner, const std::string& name, const std::string& description,
                                                    registry::YoRegistry* registry)
   : variable::YoDouble(name, description, nullptr), owner_(owner)
{
   // See BooleanParameter::BackingVariable::BackingVariable for why registration is deferred here.
   setRegistry(registry);
}

std::unique_ptr<variable::YoVariable> DoubleParameter::BackingVariable::duplicate(registry::YoRegistry* newRegistry) const
{
   // Deliberately leaked - see BooleanParameter::BackingVariable::duplicate for why.
   DoubleParameter* newParameter =
      new DoubleParameter(getName(), getDescription(), newRegistry, owner_.initialValue_, getLowerBound(), getUpperBound());
   newParameter->value_->set(getValue());
   newParameter->setLoadStatus(owner_.getLoadStatus());
   return std::unique_ptr<variable::YoVariable>(newParameter->value_.get());
}
}
