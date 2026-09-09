#pragma once

#include <iosfwd>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ihmc/yovariables/parameters/abstract_parameter_reader.h"
#include "ihmc/yovariables/parameters/xml/parameters.h"

namespace ihmc::yovariables::parameters
{
/** A parameter reader that initializes parameters from an XML file. */
class XmlParameterReader : public AbstractParameterReader
{
public:
   /**
    * Reads the given streams. If more than one stream is passed, multiple occurrences of the same
    * parameter cause its value to be overwritten with the new one.
    *
    * @param rootNamespace (optional) filters the data read, useful when the stream is shared by
    *                      multiple registries (e.g. controller and estimator).
    * @param debug         whether to print additional information when overwriting a value.
    */
   explicit XmlParameterReader(const std::vector<std::istream*>& dataStreams, std::optional<std::string> rootNamespace = std::nullopt, bool debug = false);

   /**
    * Overwrites parameters with values from overwriteParameters.
    *
    * @throws std::runtime_error if any parameter to overwrite does not already exist.
    */
   void overwrite(const std::vector<std::istream*>& overwriteParameters);

   /**
    * Like overwrite(), but does not require the parameter to already exist - it is added/replaced
    * either way.
    */
   void readAndOverwrite(const std::vector<std::istream*>& overwriteParameters);

protected:
   const std::unordered_map<std::string, ParameterData>& getValues() const override;

private:
   void readStream(std::istream& data, bool forceOverwrite);
   void addRegistry(const std::string& path, const xml::Registry& registry, bool checkParameterExists);

   static xml::Parameters parseParameters(std::istream& data);

   bool debug_;
   std::optional<std::string> rootNamespace_;
   std::unordered_map<std::string, ParameterData> parameterValues_;
};
}
