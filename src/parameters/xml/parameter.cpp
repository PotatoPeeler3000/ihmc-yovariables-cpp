#include "ihmc/yovariables/parameters/xml/parameter.h"

namespace ihmc::yovariables::parameters::xml
{
Parameter::Parameter(const std::string& name, const std::string& type, const std::optional<std::string>& value, const std::optional<std::string>& min,
                      const std::optional<std::string>& max)
   : name_(name), type_(type), min_(min), max_(max), value_(value)
{
}

const std::string& Parameter::getName() const
{
   return name_;
}

void Parameter::setName(const std::string& name)
{
   name_ = name;
}

const std::string& Parameter::getType() const
{
   return type_;
}

void Parameter::setType(const std::string& type)
{
   type_ = type;
}

const std::optional<std::string>& Parameter::getValue() const
{
   return value_;
}

void Parameter::setValue(const std::optional<std::string>& value)
{
   value_ = value;
}

const std::optional<std::string>& Parameter::getDescription() const
{
   return description_;
}

void Parameter::setDescription(const std::optional<std::string>& description)
{
   description_ = description;
}

const std::optional<std::string>& Parameter::getMin() const
{
   return min_;
}

void Parameter::setMin(const std::optional<std::string>& min)
{
   min_ = min;
}

const std::optional<std::string>& Parameter::getMax() const
{
   return max_;
}

void Parameter::setMax(const std::optional<std::string>& max)
{
   max_ = max;
}
}
