#include "ihmc/yovariables/parameters/parameter_data.h"

#include <sstream>

#include "ihmc/yovariables/parameters/yo_parameter.h"

namespace ihmc::yovariables::parameters
{
ParameterData::ParameterData(const std::string& value, const std::optional<std::string>& min, const std::optional<std::string>& max) : value_(value)
{
   if (min.has_value() && max.has_value())
   {
      min_ = std::stod(*min);
      max_ = std::stod(*max);
   }
   else
   {
      min_ = 0.0;
      max_ = 1.0;
   }
}

ParameterData::ParameterData(const std::string& value) : value_(value), min_(0.0), max_(1.0)
{
}

void ParameterData::setParameterFromThis(YoParameter& parameter) const
{
   parameter.load(value_);
   parameter.setParameterBounds(min_, max_);
}

const std::string& ParameterData::getValue() const
{
   return value_;
}

double ParameterData::getMin() const
{
   return min_;
}

double ParameterData::getMax() const
{
   return max_;
}

bool ParameterData::operator==(const ParameterData& other) const
{
   return value_ == other.value_ && min_ == other.min_ && max_ == other.max_;
}

std::string ParameterData::toString() const
{
   std::ostringstream out;
   out << "[value=" << value_ << ", min=" << min_ << ", max=" << max_ << "]";
   return out.str();
}
}
