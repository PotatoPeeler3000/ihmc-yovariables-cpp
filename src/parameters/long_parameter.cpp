#include "ihmc/yovariables/parameters/long_parameter.h"

namespace ihmc::yovariables::parameters
{
LongParameter::LongParameter(const std::string& name, registry::YoRegistry* registry) : LongParameter(name, "", registry, 0)
{
}

LongParameter::LongParameter(const std::string& name, registry::YoRegistry* registry, std::int64_t lowerBound, std::int64_t upperBound)
   : LongParameter(name, "", registry, lowerBound, upperBound)
{
}

LongParameter::LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : LongParameter(name, "", registry, 0)
{
}

LongParameter::LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int64_t lowerBound,
                              std::int64_t upperBound)
   : LongParameter(name, "", registry, 0, lowerBound, upperBound)
{
}

LongParameter::LongParameter(const std::string& name, registry::YoRegistry* registry, std::int64_t initialValue)
   : LongParameter(name, "", registry, initialValue)
{
}

LongParameter::LongParameter(const std::string& name, registry::YoRegistry* registry, std::int64_t initialValue, std::int64_t lowerBound,
                              std::int64_t upperBound)
   : LongParameter(name, "", registry, initialValue, lowerBound, upperBound)
{
}

LongParameter::LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int64_t initialValue)
   : LongParameter(name, description, registry, initialValue, kDefaultSuggestedMinimum, kDefaultSuggestedMaximum)
{
}

LongParameter::LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int64_t initialValue,
                              std::int64_t lowerBound, std::int64_t upperBound)
   : value_(std::make_unique<BackingVariable>(*this, name, description, registry)), initialValue_(initialValue)
{
   setParameterBounds(lowerBound, upperBound);
}

std::int64_t LongParameter::getValue() const
{
   checkLoaded();
   return value_->getLongValue();
}

void LongParameter::setParameterBounds(std::int64_t lowerBound, std::int64_t upperBound)
{
   YoParameter::setParameterBounds(static_cast<double>(lowerBound), static_cast<double>(upperBound));
}

variable::YoLong& LongParameter::getVariable() const
{
   return *value_;
}

void LongParameter::setToDefault()
{
   value_->set(initialValue_);
}

LongParameter::BackingVariable::BackingVariable(LongParameter& owner, const std::string& name, const std::string& description,
                                                  registry::YoRegistry* registry)
   : variable::YoLong(name, description, nullptr), owner_(owner)
{
   // See BooleanParameter::BackingVariable::BackingVariable for why registration is deferred here.
   setRegistry(registry);
}

std::unique_ptr<variable::YoVariable> LongParameter::BackingVariable::duplicate(registry::YoRegistry* newRegistry) const
{
   // Deliberately leaked - see BooleanParameter::BackingVariable::duplicate for why.
   LongParameter* newParameter = new LongParameter(getName(),
                                                     getDescription(),
                                                     newRegistry,
                                                     owner_.initialValue_,
                                                     static_cast<std::int64_t>(getLowerBound()),
                                                     static_cast<std::int64_t>(getUpperBound()));
   newParameter->value_->set(getValue());
   newParameter->setLoadStatus(owner_.getLoadStatus());
   return std::unique_ptr<variable::YoVariable>(newParameter->value_.get());
}
}
