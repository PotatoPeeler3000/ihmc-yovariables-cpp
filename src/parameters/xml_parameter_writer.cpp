#include "ihmc/yovariables/parameters/xml_parameter_writer.h"

#include <ostream>

#include <pugixml.hpp>

#include "ihmc/yovariables/registry/yo_namespace.h"

namespace ihmc::yovariables::parameters
{
namespace
{
void appendRegistry(pugi::xml_node parentElement, const xml::Registry& registry)
{
   pugi::xml_node registryElement = parentElement.append_child("registry");
   registryElement.append_attribute("name") = registry.getName().c_str();

   for (const std::unique_ptr<xml::Registry>& child : registry.getRegistries())
      appendRegistry(registryElement, *child);

   for (const xml::Parameter& parameter : registry.getParameters())
   {
      pugi::xml_node parameterElement = registryElement.append_child("parameter");
      parameterElement.append_attribute("name") = parameter.getName().c_str();
      parameterElement.append_attribute("type") = parameter.getType().c_str();
      if (parameter.getMin().has_value())
         parameterElement.append_attribute("min") = parameter.getMin()->c_str();
      if (parameter.getMax().has_value())
         parameterElement.append_attribute("max") = parameter.getMax()->c_str();
      if (parameter.getValue().has_value())
         parameterElement.append_attribute("value") = parameter.getValue()->c_str();

      if (parameter.getDescription().has_value())
         parameterElement.append_child("description").text().set(parameter.getDescription()->c_str());
   }
}
} // namespace

XmlParameterWriter::XmlParameterWriter() = default;

void XmlParameterWriter::addNamespace(const registry::YoNamespace& namespaceValue)
{
   auto newRegistry = std::make_unique<xml::Registry>(namespaceValue.getShortName());
   xml::Registry* newRegistryPtr = newRegistry.get();

   if (namespaceValue.isRoot())
   {
      parameterRoot_.getRegistries().push_back(std::move(newRegistry));
   }
   else
   {
      registry::YoNamespace parent = *namespaceValue.removeEnd(1);
      auto existing = registries_.find(parent.getName());
      if (existing == registries_.end())
      {
         addNamespace(parent);
         existing = registries_.find(parent.getName());
      }
      existing->second->getRegistries().push_back(std::move(newRegistry));
   }

   registries_[namespaceValue.getName()] = newRegistryPtr;
}

void XmlParameterWriter::setValue(const registry::YoNamespace& namespaceValue, const std::string& name, const std::string& description,
                                   const std::string& type, const std::string& value, const std::string& min, const std::string& max)
{
   std::string namespaceAsString = namespaceValue.getName();

   if (registries_.find(namespaceAsString) == registries_.end())
      addNamespace(namespaceValue);

   xml::Parameter newParameter(name, type, value, min, max);
   if (!description.empty())
      newParameter.setDescription(description);

   registries_[namespaceAsString]->getParameters().push_back(std::move(newParameter));
}

void XmlParameterWriter::write(std::ostream& outputStream) const
{
   pugi::xml_document document;
   pugi::xml_node root = document.append_child("parameters");

   for (const std::unique_ptr<xml::Registry>& registry : parameterRoot_.getRegistries())
      appendRegistry(root, *registry);

   document.save(outputStream, "  ");
}
}
