#include "ihmc/yovariables/parameters/xml_parameter_reader.h"

#include <iostream>
#include <stdexcept>

#include <pugixml.hpp>

namespace ihmc::yovariables::parameters
{
namespace
{
/**
 * The direct child elements of parent matching tagName - mirrors XmlParameterReader.java's
 * getChildElementsByTagName, restricted to direct children (not all descendants, since <registry>
 * nests recursively) via pugixml's node.children(name), which already only walks direct children.
 */
pugi::xml_object_range<pugi::xml_named_node_iterator> childElementsByTagName(const pugi::xml_node& parent, const char* tagName)
{
   return parent.children(tagName);
}

std::optional<std::string> optionalAttribute(const pugi::xml_node& element, const char* attributeName)
{
   pugi::xml_attribute attribute = element.attribute(attributeName);
   return attribute ? std::optional<std::string>(attribute.value()) : std::nullopt;
}

xml::Parameter parseParameter(const pugi::xml_node& parameterElement)
{
   xml::Parameter parameter(parameterElement.attribute("name").value(),
                             parameterElement.attribute("type").value(),
                             optionalAttribute(parameterElement, "value"),
                             optionalAttribute(parameterElement, "min"),
                             optionalAttribute(parameterElement, "max"));

   pugi::xml_node descriptionElement = parameterElement.child("description");
   if (descriptionElement)
      parameter.setDescription(std::string(descriptionElement.text().get()));

   return parameter;
}

std::vector<xml::Parameter> parseParameterList(const pugi::xml_node& parent)
{
   std::vector<xml::Parameter> parameters;
   for (const pugi::xml_node& parameterElement : childElementsByTagName(parent, "parameter"))
      parameters.push_back(parseParameter(parameterElement));
   return parameters;
}

std::unique_ptr<xml::Registry> parseRegistry(const pugi::xml_node& registryElement);

std::vector<std::unique_ptr<xml::Registry>> parseRegistries(const pugi::xml_node& parent)
{
   std::vector<std::unique_ptr<xml::Registry>> registries;
   for (const pugi::xml_node& registryElement : childElementsByTagName(parent, "registry"))
      registries.push_back(parseRegistry(registryElement));
   return registries;
}

std::unique_ptr<xml::Registry> parseRegistry(const pugi::xml_node& registryElement)
{
   auto registry = std::make_unique<xml::Registry>(registryElement.attribute("name").value());
   registry->setRegistries(parseRegistries(registryElement));
   registry->setParameters(parseParameterList(registryElement));
   return registry;
}
} // namespace

XmlParameterReader::XmlParameterReader(const std::vector<std::istream*>& dataStreams, std::optional<std::string> rootNamespace, bool debug)
   : debug_(debug), rootNamespace_(std::move(rootNamespace))
{
   for (std::istream* dataStream : dataStreams)
      readStream(*dataStream, false);
}

void XmlParameterReader::overwrite(const std::vector<std::istream*>& overwriteParameters)
{
   for (std::istream* dataStream : overwriteParameters)
      readStream(*dataStream, true);
}

void XmlParameterReader::readAndOverwrite(const std::vector<std::istream*>& overwriteParameters)
{
   for (std::istream* dataStream : overwriteParameters)
      readStream(*dataStream, false);
}

void XmlParameterReader::readStream(std::istream& data, bool forceOverwrite)
{
   xml::Parameters parameterRoot = parseParameters(data);

   for (const std::unique_ptr<xml::Registry>& registry : parameterRoot.getRegistries())
   {
      if (!rootNamespace_.has_value() || registry->getName() == *rootNamespace_)
         addRegistry(registry->getName(), *registry, forceOverwrite);
   }
}

xml::Parameters XmlParameterReader::parseParameters(std::istream& data)
{
   // Unlike Java's javax.xml parsers, pugixml has no DTD/external-entity support to begin with, so
   // there is no equivalent XXE hardening needed here.
   pugi::xml_document document;
   pugi::xml_parse_result result = document.load(data);
   if (!result)
      throw std::runtime_error(std::string("Failed to parse parameters XML: ") + result.description());

   xml::Parameters parameters;
   parameters.setRegistries(parseRegistries(document.document_element()));
   return parameters;
}

void XmlParameterReader::addRegistry(const std::string& path, const xml::Registry& registry, bool checkParameterExists)
{
   for (const xml::Parameter& param : registry.getParameters())
   {
      std::string name = path + "." + param.getName();
      ParameterData data(param.getValue().value(), param.getMin(), param.getMax());

      auto existing = parameterValues_.find(name);
      bool alreadyPresent = existing != parameterValues_.end();
      parameterValues_.insert_or_assign(name, data);

      if (alreadyPresent)
      {
         if (debug_)
            std::cout << "[XmlParameterReader]: overwriting " << param.getName() << std::endl;
      }
      else if (checkParameterExists)
      {
         throw std::runtime_error("[XmlParameterReader]: trying to overwrite parameter " + param.getName() + " but it does not exist.");
      }
   }

   for (const std::unique_ptr<xml::Registry>& child : registry.getRegistries())
      addRegistry(path + "." + child->getName(), *child, checkParameterExists);
}

const std::unordered_map<std::string, ParameterData>& XmlParameterReader::getValues() const
{
   return parameterValues_;
}
}
