#pragma once

#include <optional>
#include <string>

namespace ihmc::yovariables::parameters
{
class YoParameter;

/** Minimalist data structure used in parameter readers. */
class ParameterData
{
public:
   /** min/max default to 0.0/1.0 if either is absent. */
   ParameterData(const std::string& value, const std::optional<std::string>& min, const std::optional<std::string>& max);

   /** min/max default to 0.0/1.0. */
   explicit ParameterData(const std::string& value);

   /** Initializes parameter with this data's value and bounds. */
   void setParameterFromThis(YoParameter& parameter) const;

   const std::string& getValue() const;
   double getMin() const;
   double getMax() const;

   bool operator==(const ParameterData& other) const;

   std::string toString() const;

private:
   std::string value_;
   double min_;
   double max_;
};
}
