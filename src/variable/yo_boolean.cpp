#include "ihmc/yovariables/variable/yo_boolean.h"

namespace ihmc::yovariables::variable
{
YoBoolean::YoBoolean(const std::string& name, registry::YoRegistry* registry) : YoBoolean(name, "", registry)
{
}

YoBoolean::YoBoolean(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : YoVariable(YoVariableType::BOOLEAN, name, description, registry)
{
   set(false);
}

bool YoBoolean::valueEquals(bool value) const
{
   return value_ == value;
}

bool YoBoolean::getValue() const
{
   return value_;
}

bool YoBoolean::getBooleanValue() const
{
   return value_;
}

bool YoBoolean::set(bool value)
{
   return set(value, true);
}

bool YoBoolean::set(bool value, bool notify)
{
   if (value_ != value)
   {
      value_ = value;
      if (notify)
         notifyListeners();
      return true;
   }
   return false;
}

double YoBoolean::getValueAsDouble() const
{
   return value_ ? 1.0 : 0.0;
}

bool YoBoolean::setValueFromDouble(double value, bool notifyListeners)
{
   return set(value >= 0.5, notifyListeners);
}

std::int64_t YoBoolean::getValueAsLongBits() const
{
   return value_ ? 1 : 0;
}

bool YoBoolean::setValueFromLongBits(std::int64_t value, bool notifyListeners)
{
   return set(value == 1, notifyListeners);
}

bool YoBoolean::setValue(YoVariable& other, bool notifyListeners)
{
   return set(static_cast<YoBoolean&>(other).getValue(), notifyListeners);
}

std::string YoBoolean::getValueAsString(const std::optional<std::string>&) const
{
   return value_ ? "true" : "false";
}

bool YoBoolean::parseValue(const std::string& valueAsString, bool notifyListeners)
{
   return set(valueAsString == "true", notifyListeners);
}

std::string YoBoolean::convertDoubleValueToString(const std::optional<std::string>&, double value) const
{
   return value >= 0.5 ? "true" : "false";
}

bool YoBoolean::isZero() const
{
   return !value_;
}

std::unique_ptr<YoVariable> YoBoolean::duplicate(registry::YoRegistry* newRegistry) const
{
   auto duplicate = std::make_unique<YoBoolean>(getName(), getDescription(), newRegistry);
   duplicate->set(value_);
   return duplicate;
}

std::string YoBoolean::toString() const
{
   return getName() + ": " + (value_ ? "true" : "false");
}
}
