#include "ihmc/yovariables/variable/yo_double.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ihmc::yovariables::variable
{
namespace
{
std::string formatDouble(const std::optional<std::string>& format, double value)
{
   if (!format.has_value())
      return std::to_string(value);

   int size = std::snprintf(nullptr, 0, format->c_str(), value);
   std::string result(size, '\0');
   std::snprintf(result.data(), size + 1, format->c_str(), value);
   return result;
}
} // namespace

YoDouble::YoDouble(const std::string& name, registry::YoRegistry* registry) : YoDouble(name, "", registry)
{
}

YoDouble::YoDouble(const std::string& name, const std::string& description, registry::YoRegistry* registry)
   : YoVariable(YoVariableType::DOUBLE, name, description, registry)
{
   set(0.0);
}

bool YoDouble::isNaN() const
{
   return std::isnan(value_);
}

void YoDouble::add(const YoDouble& other)
{
   set(value_ + other.value_);
}

void YoDouble::sub(const YoDouble& other)
{
   set(value_ - other.value_);
}

void YoDouble::sub(double value)
{
   set(value_ - value);
}

void YoDouble::add(double value)
{
   set(value_ + value);
}

void YoDouble::mul(double value)
{
   set(value_ * value);
}

void YoDouble::mul(const YoDouble& other)
{
   set(value_ * other.value_);
}

bool YoDouble::valueEquals(double value) const
{
   return value_ == value;
}

void YoDouble::setToNaN()
{
   set(std::nan(""));
}

double YoDouble::getValue() const
{
   return value_;
}

double YoDouble::getDoubleValue() const
{
   return value_;
}

bool YoDouble::set(double value)
{
   return set(value, true);
}

bool YoDouble::set(double value, bool notify)
{
   // Compare via bit pattern (rather than ==) so that setting to NaN is detected as a change.
   if (std::memcmp(&value_, &value, sizeof(double)) != 0)
   {
      value_ = value;
      if (notify)
         notifyListeners();
      return true;
   }
   return false;
}

double YoDouble::getValueAsDouble() const
{
   return getDoubleValue();
}

bool YoDouble::setValueFromDouble(double value, bool notifyListeners)
{
   return set(value, notifyListeners);
}

std::int64_t YoDouble::getValueAsLongBits() const
{
   std::int64_t bits;
   std::memcpy(&bits, &value_, sizeof(double));
   return bits;
}

bool YoDouble::setValueFromLongBits(std::int64_t value, bool notifyListeners)
{
   double asDouble;
   std::memcpy(&asDouble, &value, sizeof(double));
   return set(asDouble, notifyListeners);
}

bool YoDouble::setValue(YoVariable& other, bool notifyListeners)
{
   return set(static_cast<YoDouble&>(other).getValue(), notifyListeners);
}

std::string YoDouble::getValueAsString(const std::optional<std::string>& format) const
{
   return convertDoubleValueToString(format, value_);
}

bool YoDouble::parseValue(const std::string& valueAsString, bool notifyListeners)
{
   return set(std::stod(valueAsString), notifyListeners);
}

std::string YoDouble::convertDoubleValueToString(const std::optional<std::string>& format, double value) const
{
   return formatDouble(format, value);
}

bool YoDouble::isZero() const
{
   return value_ == 0.0;
}

std::unique_ptr<YoVariable> YoDouble::duplicate(registry::YoRegistry* newRegistry) const
{
   auto duplicate = std::make_unique<YoDouble>(getName(), getDescription(), newRegistry);
   duplicate->setVariableBounds(getLowerBound(), getUpperBound());
   duplicate->set(value_);
   return duplicate;
}

std::string YoDouble::toString() const
{
   return getName() + ": " + std::to_string(value_);
}
}
