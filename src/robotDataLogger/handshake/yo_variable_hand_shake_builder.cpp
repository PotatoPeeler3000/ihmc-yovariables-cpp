#include "ihmc/robotDataLogger/handshake/yo_variable_hand_shake_builder.h"

#include <stdexcept>

#include "ihmc/yovariables/parameters/parameter_load_status.h"
#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_enum_holder.h"
#include "ihmc/yovariables/variable/yo_variable.h"
#include "ihmc/yovariables/variable/yo_variable_type.h"

namespace ihmc::robotDataLogger::handshake
{
namespace
{
using yovariables::parameters::ParameterLoadStatus;
using yovariables::registry::YoRegistry;
using yovariables::variable::YoEnumHolder;
using yovariables::variable::YoVariable;
using yovariables::variable::YoVariableType;

/** Mirrors Java's String.substring(0,255) truncation applied to descriptions/enum values. */
std::string truncateFront(const std::string& value, std::size_t maxLength)
{
   return value.size() > maxLength ? value.substr(0, maxLength) : value;
}

/** Mirrors Java's `name.substring(name.length()-255)` (+ drop a leading '.') for long class names. */
std::string truncateBack(const std::string& value, std::size_t maxLength)
{
   if (value.size() <= maxLength)
      return value;
   std::string truncated = value.substr(value.size() - maxLength);
   if (!truncated.empty() && truncated.front() == '.')
      truncated = truncated.substr(1);
   return truncated;
}

YoType toYoType(YoVariableType type)
{
   switch (type)
   {
      case YoVariableType::DOUBLE:
         return YoType::DoubleYoVariable;
      case YoVariableType::BOOLEAN:
         return YoType::BooleanYoVariable;
      case YoVariableType::INTEGER:
         return YoType::IntegerYoVariable;
      case YoVariableType::LONG:
         return YoType::LongYoVariable;
      case YoVariableType::ENUM:
         return YoType::EnumYoVariable;
   }
   throw std::runtime_error("Unknown variable type");
}

LoadStatus toLoadStatus(ParameterLoadStatus status)
{
   switch (status)
   {
      case ParameterLoadStatus::UNLOADED:
         return LoadStatus::Unloaded;
      case ParameterLoadStatus::DEFAULT:
         return LoadStatus::Default;
      case ParameterLoadStatus::LOADED:
         return LoadStatus::Loaded;
   }
   throw std::runtime_error("Unknown load status");
}
}

YoVariableHandShakeBuilder::YoVariableHandShakeBuilder(const std::string& rootRegistryName, double dt)
{
   // Mirrors Java's createRootRegistry(): a synthetic root at index 0, self-referential parent=0 -
   // never used for anything else. The actual top-level registry passed to build() becomes index 1.
   YoRegistryDefinition root;
   root.name = rootRegistryName;
   root.parent = 0;
   handshake_.registries.push_back(root);

   handshake_.dt = dt;
}

void YoVariableHandShakeBuilder::build(YoRegistry& rootRegistry)
{
   addRegistry(0, rootRegistry);
}

int YoVariableHandShakeBuilder::addRegistry(int parentID, YoRegistry& registry)
{
   int myID = nextRegistryID_++;

   YoRegistryDefinition definition;
   definition.name = registry.getName();
   definition.parent = static_cast<std::uint16_t>(parentID);
   handshake_.registries.push_back(definition);

   addVariables(myID, registry);

   // Pre-order: this registry's own variables are added before recursing into children, matching
   // Java's addRegistry() exactly - this ordering is what RegistrySendBuffer's wire-value order
   // depends on.
   for (YoRegistry* child : registry.getChildRegistries())
      addRegistry(myID, *child);

   return myID;
}

void YoVariableHandShakeBuilder::addVariables(int registryID, YoRegistry& registry)
{
   for (YoVariable* variable : registry.getVariables())
   {
      YoVariableDefinition definition;
      definition.name = variable->getName();
      definition.description = truncateFront(variable->getDescription(), 255);
      definition.registry = static_cast<std::uint16_t>(registryID);
      definition.isParameter = variable->isParameter();
      definition.min = variable->getLowerBound();
      definition.max = variable->getUpperBound();

      if (variable->isParameter())
         definition.loadStatus = toLoadStatus(variable->getParameter()->getLoadStatus());
      else
         definition.loadStatus = LoadStatus::NoParameter;

      definition.type = toYoType(variable->getType());

      if (variable->getType() == YoVariableType::ENUM)
      {
         auto* enumHolder = dynamic_cast<YoEnumHolder*>(variable);
         if (enumHolder == nullptr)
            throw std::logic_error("YoVariable reports type ENUM but does not implement YoEnumHolder: " + variable->getFullNameString());

         std::string key = enumHolder->isBackedByEnum() ? enumHolder->getEnumTypeKey() : variable->getFullNameString() + ".EnumType";
         definition.enumType = getOrAddEnumType(key, enumHolder->getEnumValuesAsString());
         definition.allowNullValues = enumHolder->isNullAllowed();
      }

      handshake_.variables.push_back(definition);
      variablesInWireOrder_.push_back(variable);
   }
}

std::uint16_t YoVariableHandShakeBuilder::getOrAddEnumType(const std::string& key, const std::vector<std::string>& enumValues)
{
   auto it = enumTypeIndexByKey_.find(key);
   if (it != enumTypeIndexByKey_.end())
      return it->second;

   std::uint16_t myID = static_cast<std::uint16_t>(handshake_.enumTypes.size());

   EnumType enumType;
   enumType.name = truncateBack(key, 255);
   enumType.enumValues.reserve(enumValues.size());
   for (const std::string& value : enumValues)
      enumType.enumValues.push_back(truncateFront(value, 255));

   handshake_.enumTypes.push_back(std::move(enumType));
   enumTypeIndexByKey_[key] = myID;
   return myID;
}
}
