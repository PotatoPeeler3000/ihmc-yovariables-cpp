#include "ihmc/yovariables/variable/yo_long.h"

#include <cmath>

namespace ihmc::yovariables::variable
{
YoLong::YoLong(const std::string& name, registry::YoRegistry* registry) : YoLong(name, "", registry)
{
}

YoLong::YoLong(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : YoVariable(YoVariableType::LONG, name, description, registry)
{
   set(0);
}

void YoLong::increment()
{
   set(value_ + 1);
}

void YoLong::decrement()
{
   set(value_ - 1);
}

void YoLong::add(std::int64_t value)
{
   set(value_ + value);
}

void YoLong::subtract(std::int64_t value)
{
   set(value_ - value);
}

bool YoLong::valueEquals(std::int64_t value) const
{
   return value_ == value;
}

std::int64_t YoLong::getValue() const
{
   return value_;
}

std::int64_t YoLong::getLongValue() const
{
   return value_;
}

bool YoLong::set(std::int64_t value)
{
   return set(value, true);
}

bool YoLong::set(std::int64_t value, bool notify)
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

double YoLong::getValueAsDouble() const
{
   return static_cast<double>(value_);
}

bool YoLong::setValueFromDouble(double value, bool notifyListeners)
{
   return set(std::llround(value), notifyListeners);
}

std::int64_t YoLong::getValueAsLongBits() const
{
   return getLongValue();
}

bool YoLong::setValueFromLongBits(std::int64_t value, bool notifyListeners)
{
   return set(value, notifyListeners);
}

bool YoLong::setValue(YoVariable& other, bool notifyListeners)
{
   return set(static_cast<YoLong&>(other).getValue(), notifyListeners);
}

std::string YoLong::getValueAsString(const std::optional<std::string>&) const
{
   return std::to_string(value_);
}

bool YoLong::parseValue(const std::string& valueAsString, bool notifyListeners)
{
   return set(std::stoll(valueAsString), notifyListeners);
}

std::string YoLong::convertDoubleValueToString(const std::optional<std::string>&, double value) const
{
   return std::to_string(static_cast<std::int64_t>(value));
}

bool YoLong::isZero() const
{
   return getLongValue() == 0;
}

std::unique_ptr<YoVariable> YoLong::duplicate(registry::YoRegistry* newRegistry) const
{
   auto duplicate = std::make_unique<YoLong>(getName(), getDescription(), newRegistry);
   duplicate->setVariableBounds(getLowerBound(), getUpperBound());
   duplicate->set(getLongValue());
   return duplicate;
}

std::string YoLong::toString() const
{
   return getName() + ": " + std::to_string(value_);
}
}
