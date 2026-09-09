#include "ihmc/yovariables/parameters/xml/registry.h"

namespace ihmc::yovariables::parameters::xml
{
Registry::Registry(const std::string& name) : name_(name)
{
}

const std::string& Registry::getName() const
{
   return name_;
}

void Registry::setName(const std::string& name)
{
   name_ = name;
}

const std::vector<std::unique_ptr<Registry>>& Registry::getRegistries() const
{
   return registries_;
}

std::vector<std::unique_ptr<Registry>>& Registry::getRegistries()
{
   return registries_;
}

void Registry::setRegistries(std::vector<std::unique_ptr<Registry>> registries)
{
   registries_ = std::move(registries);
}

const std::vector<Parameter>& Registry::getParameters() const
{
   return parameters_;
}

std::vector<Parameter>& Registry::getParameters()
{
   return parameters_;
}

void Registry::setParameters(std::vector<Parameter> parameters)
{
   parameters_ = std::move(parameters);
}
}
