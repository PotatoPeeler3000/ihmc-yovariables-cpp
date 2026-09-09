#include "ihmc/yovariables/parameters/integer_parameter.h"

namespace ihmc::yovariables::parameters
{
IntegerParameter::IntegerParameter(const std::string& name, registry::YoRegistry* registry) : IntegerParameter(name, "", registry, 0)
{
}

IntegerParameter::IntegerParameter(const std::string& name, registry::YoRegistry* registry, std::int32_t lowerBound, std::int32_t upperBound)
   : IntegerParameter(name, "", registry, lowerBound, upperBound)
{
}

IntegerParameter::IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : IntegerParameter(name, description, registry, 0)
{
}

IntegerParameter::IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int32_t lowerBound,
                                    std::int32_t upperBound)
   : IntegerParameter(name, description, registry, 0, lowerBound, upperBound)
{
}

IntegerParameter::IntegerParameter(const std::string& name, registry::YoRegistry* registry, std::int32_t initialValue)
   : IntegerParameter(name, "", registry, initialValue)
{
}

IntegerParameter::IntegerParameter(const std::string& name, registry::YoRegistry* registry, std::int32_t initialValue, std::int32_t lowerBound,
                                    std::int32_t upperBound)
   : IntegerParameter(name, "", registry, initialValue, lowerBound, upperBound)
{
}

IntegerParameter::IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int32_t initialValue)
   : IntegerParameter(name, description, registry, initialValue, kDefaultSuggestedMinimum, kDefaultSuggestedMaximum)
{
}

IntegerParameter::IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int32_t initialValue,
                                    std::int32_t lowerBound, std::int32_t upperBound)
   : value_(std::make_unique<BackingVariable>(*this, name, description, registry)), initialValue_(initialValue)
{
   setParameterBounds(lowerBound, upperBound);
}

std::int32_t IntegerParameter::getValue() const
{
   checkLoaded();
   return value_->getIntegerValue();
}

void IntegerParameter::setParameterBounds(std::int32_t lowerBound, std::int32_t upperBound)
{
   YoParameter::setParameterBounds(lowerBound, upperBound);
}

variable::YoInteger& IntegerParameter::getVariable() const
{
   return *value_;
}

void IntegerParameter::setToDefault()
{
   value_->set(initialValue_);
}

IntegerParameter::BackingVariable::BackingVariable(IntegerParameter& owner, const std::string& name, const std::string& description,
                                                     registry::YoRegistry* registry)
   : variable::YoInteger(name, description, nullptr), owner_(owner)
{
   // See BooleanParameter::BackingVariable::BackingVariable for why registration is deferred here.
   setRegistry(registry);
}

std::unique_ptr<variable::YoVariable> IntegerParameter::BackingVariable::duplicate(registry::YoRegistry* newRegistry) const
{
   // Deliberately leaked - see BooleanParameter::BackingVariable::duplicate for why.
   IntegerParameter* newParameter = new IntegerParameter(getName(),
                                                           getDescription(),
                                                           newRegistry,
                                                           owner_.initialValue_,
                                                           static_cast<std::int32_t>(getLowerBound()),
                                                           static_cast<std::int32_t>(getUpperBound()));
   newParameter->value_->set(getValue());
   newParameter->setLoadStatus(owner_.getLoadStatus());
   return std::unique_ptr<variable::YoVariable>(newParameter->value_.get());
}
}
