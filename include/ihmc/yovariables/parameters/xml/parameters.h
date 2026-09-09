#pragma once

#include <memory>
#include <vector>

#include "ihmc/yovariables/parameters/xml/registry.h"

namespace ihmc::yovariables::parameters::xml
{
/**
 * XML root token used with XmlParameterReader/XmlParameterWriter.
 * <p>
 * Maps to the {@code <parameters>} root element containing {@code <registry>} children; see
 * XmlParameterReader/XmlParameterWriter for the hand-written DOM (de)serialization.
 * </p>
 */
class Parameters
{
public:
   const std::vector<std::unique_ptr<Registry>>& getRegistries() const;
   std::vector<std::unique_ptr<Registry>>& getRegistries();
   void setRegistries(std::vector<std::unique_ptr<Registry>> registries);

private:
   std::vector<std::unique_ptr<Registry>> registries_;
};
}
