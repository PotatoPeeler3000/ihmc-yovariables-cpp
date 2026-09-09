#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ihmc/yovariables/parameters/xml/parameter.h"

namespace ihmc::yovariables::parameters::xml
{
/**
 * XML token used with XmlParameterReader/XmlParameterWriter representing a registry, which can
 * have sub-registries and parameters.
 * <p>
 * Maps to a {@code <registry name="...">} element containing nested {@code <registry>} and
 * {@code <parameter>} children; see XmlParameterReader/XmlParameterWriter for the hand-written DOM
 * (de)serialization.
 * </p>
 * <p>
 * Child registries are held by std::unique_ptr (rather than by value) specifically so
 * XmlParameterWriter can keep a stable pointer to a Registry, in a lookup map, while more children
 * keep getting appended elsewhere in the tree - matching the Java source's reference semantics,
 * where appending to a sibling's list never invalidates a previously-held Registry reference. A
 * plain std::vector<Registry> would invalidate such a pointer on reallocation.
 * </p>
 */
class Registry
{
public:
   Registry() = default;
   explicit Registry(const std::string& name);

   const std::string& getName() const;
   void setName(const std::string& name);

   const std::vector<std::unique_ptr<Registry>>& getRegistries() const;
   std::vector<std::unique_ptr<Registry>>& getRegistries();
   void setRegistries(std::vector<std::unique_ptr<Registry>> registries);

   const std::vector<Parameter>& getParameters() const;
   std::vector<Parameter>& getParameters();
   void setParameters(std::vector<Parameter> parameters);

private:
   std::string name_;
   std::vector<std::unique_ptr<Registry>> registries_;
   std::vector<Parameter> parameters_;
};
}
