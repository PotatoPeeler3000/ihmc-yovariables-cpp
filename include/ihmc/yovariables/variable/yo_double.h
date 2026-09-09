#pragma once

#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::variable
{
class YoDouble : public YoVariable, public providers::DoubleProvider
{
public:
   YoDouble(const std::string& name, registry::YoRegistry* registry);
   YoDouble(const std::string& name, const std::string& description, registry::YoRegistry* registry);

   bool isNaN() const;

   void add(const YoDouble& other);
   void sub(const YoDouble& other);
   void add(double value);
   void sub(double value);
   void mul(double value);
   void mul(const YoDouble& other);

   bool valueEquals(double value) const;
   void setToNaN();

   double getValue() const override;
   double getDoubleValue() const;

   bool set(double value);
   bool set(double value, bool notifyListeners);

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
   double value_ = 0.0;
};
}
