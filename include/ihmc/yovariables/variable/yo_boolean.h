#pragma once

#include "ihmc/yovariables/providers/boolean_provider.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::variable
{
class YoBoolean : public YoVariable, public providers::BooleanProvider
{
public:
   YoBoolean(const std::string& name, registry::YoRegistry* registry);
   YoBoolean(const std::string& name, const std::string& description, registry::YoRegistry* registry);

   // Un-hide YoVariable's 1-arg convenience overloads: declaring the 2-arg virtuals below with the
   // same names would otherwise hide all base overloads of those names, including these.
   using YoVariable::setValueFromDouble;
   using YoVariable::setValueFromLongBits;
   using YoVariable::parseValue;

   bool valueEquals(bool value) const;

   bool getValue() const override;
   bool getBooleanValue() const;

   bool set(bool value);
   bool set(bool value, bool notifyListeners);

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
   bool value_ = false;
};
}
