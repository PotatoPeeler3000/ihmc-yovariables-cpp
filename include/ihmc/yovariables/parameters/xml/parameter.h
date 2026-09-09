#pragma once

#include <optional>
#include <string>

namespace ihmc::yovariables::parameters::xml
{
/**
 * XML token used with XmlParameterReader/XmlParameterWriter to import/export a YoParameter with
 * its value.
 * <p>
 * Maps to a {@code <parameter name="..." type="..." min="..." max="..." value="..."><description>
 * ...</description></parameter>} element; see XmlParameterReader/XmlParameterWriter for the
 * hand-written DOM (de)serialization.
 * </p>
 */
class Parameter
{
public:
   Parameter() = default;
   Parameter(const std::string& name, const std::string& type, const std::optional<std::string>& value, const std::optional<std::string>& min,
             const std::optional<std::string>& max);

   const std::string& getName() const;
   void setName(const std::string& name);

   const std::string& getType() const;
   void setType(const std::string& type);

   const std::optional<std::string>& getValue() const;
   void setValue(const std::optional<std::string>& value);

   const std::optional<std::string>& getDescription() const;
   void setDescription(const std::optional<std::string>& description);

   const std::optional<std::string>& getMin() const;
   void setMin(const std::optional<std::string>& min);

   const std::optional<std::string>& getMax() const;
   void setMax(const std::optional<std::string>& max);

private:
   std::string name_;
   std::string type_;
   std::optional<std::string> min_;
   std::optional<std::string> max_;
   std::optional<std::string> value_;
   std::optional<std::string> description_;
};
}
