#pragma once

#include <cstdint>

#include "ihmc/yovariables/providers/integer_provider.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::variable
{
class YoInteger : public YoVariable, public providers::IntegerProvider
{
public:
   YoInteger(const std::string& name, registry::YoRegistry* registry);
   YoInteger(const std::string& name, const std::string& description, registry::YoRegistry* registry);

   // Un-hide YoVariable's 1-arg convenience overloads: declaring the 2-arg virtuals below with the
   // same names would otherwise hide all base overloads of those names, including these.
   using YoVariable::setValueFromDouble;
   using YoVariable::setValueFromLongBits;
   using YoVariable::parseValue;

   void increment();
   void decrement();
   void add(std::int32_t value);
   void sub(std::int32_t value);
   bool valueEquals(std::int32_t value) const;

   std::int32_t getValue() const override;
   std::int32_t getIntegerValue() const;

   bool set(std::int32_t value);
   bool set(std::int32_t value, bool notifyListeners);

   double getValueAsDouble() const override;
   bool setValueFromDouble(double value, bool notifyListeners) override;
   std::int64_t getValueAsLongBits() const override;
   bool setValueFromLongBits(std::int64_t value, bool notifyListeners) override;
   bool setValue(YoVariable& other, bool notifyListeners) override;
   std::string getValueAsString(const std::optional<std::string>& format) const override;
   bool parseValue(const std::string& valueAsString, bool notifyListeners) override;
   std::string convertDoubleValueToString(const std::optional<std::string>& format, double value) const override;
   bool isZero() const override;
   std::unique_ptr<YoVariable> duplicate(registry::YoRegistry* newRegistry) const override;
   std::string toString() const override;

private:
   std::int32_t value_ = 0;
};
}
