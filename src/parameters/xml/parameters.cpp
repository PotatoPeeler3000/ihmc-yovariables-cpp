#include "ihmc/yovariables/parameters/xml/parameters.h"

namespace ihmc::yovariables::parameters::xml
{
const std::vector<std::unique_ptr<Registry>>& Parameters::getRegistries() const
{
   return registries_;
}

std::vector<std::unique_ptr<Registry>>& Parameters::getRegistries()
{
   return registries_;
}

void Parameters::setRegistries(std::vector<std::unique_ptr<Registry>> registries)
{
   registries_ = std::move(registries);
}
}
