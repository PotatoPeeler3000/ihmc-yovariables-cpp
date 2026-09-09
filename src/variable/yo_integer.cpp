#include "ihmc/yovariables/variable/yo_integer.h"

#include <cmath>

namespace ihmc::yovariables::variable
{
YoInteger::YoInteger(const std::string& name, registry::YoRegistry* registry) : YoInteger(name, "", registry)
{
}

YoInteger::YoInteger(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : YoVariable(YoVariableType::INTEGER, name, description, registry)
{
   set(0);
}

void YoInteger::increment()
{
   set(value_ + 1);
}

void YoInteger::decrement()
{
   set(value_ - 1);
}

void YoInteger::add(std::int32_t value)
{
   set(value_ + value);
}

void YoInteger::sub(std::int32_t value)
{
   set(value_ - value);
}

bool YoInteger::valueEquals(std::int32_t value) const
{
   return value_ == value;
}

std::int32_t YoInteger::getValue() const
{
   return value_;
}

std::int32_t YoInteger::getIntegerValue() const
{
   return value_;
}

bool YoInteger::set(std::int32_t value)
{
   return set(value, true);
}

bool YoInteger::set(std::int32_t value, bool notify)
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

double YoInteger::getValueAsDouble() const
{
   return value_;
}

bool YoInteger::setValueFromDouble(double value, bool notifyListeners)
{
   return set(static_cast<std::int32_t>(std::lround(value)), notifyListeners);
}

std::int64_t YoInteger::getValueAsLongBits() const
{
   return value_;
}

bool YoInteger::setValueFromLongBits(std::int64_t value, bool notifyListeners)
{
   return set(static_cast<std::int32_t>(value), notifyListeners);
}

bool YoInteger::setValue(YoVariable& other, bool notifyListeners)
{
   return set(static_cast<YoInteger&>(other).getValue(), notifyListeners);
}

std::string YoInteger::getValueAsString(const std::optional<std::string>&) const
{
   return std::to_string(value_);
}

bool YoInteger::parseValue(const std::string& valueAsString, bool notifyListeners)
{
   return set(std::stoi(valueAsString), notifyListeners);
}

std::string YoInteger::convertDoubleValueToString(const std::optional<std::string>&, double value) const
{
   return std::to_string(static_cast<std::int32_t>(value));
}

bool YoInteger::isZero() const
{
   return value_ == 0;
}

std::unique_ptr<YoVariable> YoInteger::duplicate(registry::YoRegistry* newRegistry) const
{
   auto duplicate = std::make_unique<YoInteger>(getName(), getDescription(), newRegistry);
   duplicate->setVariableBounds(getLowerBound(), getUpperBound());
   duplicate->set(value_);
   return duplicate;
}

std::string YoInteger::toString() const
{
   return getName() + ": " + std::to_string(value_);
}
}
