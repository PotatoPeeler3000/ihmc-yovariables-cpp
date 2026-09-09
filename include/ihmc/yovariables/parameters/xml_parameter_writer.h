#pragma once

#include <iosfwd>
#include <string>
#include <unordered_map>

#include "ihmc/yovariables/parameters/abstract_parameter_writer.h"
#include "ihmc/yovariables/parameters/xml/parameters.h"

namespace ihmc::yovariables::parameters
{
/** A parameter writer that exports parameters to an XML file. */
class XmlParameterWriter : public AbstractParameterWriter
{
public:
   XmlParameterWriter();

   /** Writes the parameter tree previously built via addParameters(YoRegistry&). */
   void write(std::ostream& outputStream) const;

protected:
   void setValue(const registry::YoNamespace& namespaceValue, const std::string& name, const std::string& description, const std::string& type,
                 const std::string& value, const std::string& min, const std::string& max) override;

private:
   void addNamespace(const registry::YoNamespace& namespaceValue);

   std::unordered_map<std::string, xml::Registry*> registries_;
   xml::Parameters parameterRoot_;
};
}
