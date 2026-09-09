#include "ihmc/robotDataLogger/handshake/json_serialization.h"

#include <nlohmann/json.hpp>

namespace ihmc::robotDataLogger::handshake
{
namespace
{
using json = nlohmann::json;

// Matches CDRInterchangeSerializer.convertYoTypeToString(byte) exactly.
const char* yoTypeToString(YoType type)
{
   switch (type)
   {
      case YoType::DoubleYoVariable:
         return "DoubleYoVariable";
      case YoType::BooleanYoVariable:
         return "BooleanYoVariable";
      case YoType::IntegerYoVariable:
         return "IntegerYoVariable";
      case YoType::LongYoVariable:
         return "LongYoVariable";
      case YoType::EnumYoVariable:
         return "EnumYoVariable";
   }
   return "DoubleYoVariable";
}

// Matches CDRInterchangeSerializer.convertLoadStatusToString(byte) exactly.
const char* loadStatusToString(LoadStatus status)
{
   switch (status)
   {
      case LoadStatus::NoParameter:
         return "NoParameter";
      case LoadStatus::Unloaded:
         return "Unloaded";
      case LoadStatus::Default:
         return "Default";
      case LoadStatus::Loaded:
         return "Loaded";
   }
   return "NoParameter";
}

json toJson(const YoRegistryDefinition& registry)
{
   return json{{"parent", registry.parent}, {"name", registry.name}};
}

json toJson(const YoVariableDefinition& variable)
{
   return json{{"name", variable.name},
               {"description", variable.description},
               {"type", yoTypeToString(variable.type)},
               {"registry", variable.registry},
               {"enumType", variable.enumType},
               {"allowNullValues", variable.allowNullValues},
               {"isParameter", variable.isParameter},
               {"min", variable.min},
               {"max", variable.max},
               {"loadStatus", loadStatusToString(variable.loadStatus)}};
}

json toJson(const EnumType& enumType)
{
   return json{{"name", enumType.name}, {"enumValues", enumType.enumValues}};
}

json toJson(const JointDefinition& joint)
{
   // JointDefinition.type serializes as SiXDoFJoint/OneDoFJoint (see
   // CDRInterchangeSerializer.convertJointTypeToString); always empty in this phase, but kept
   // spec-correct in case joints are populated later.
   return json{{"name", joint.name}, {"type", joint.type == 0 ? "SiXDoFJoint" : "OneDoFJoint"}};
}

json toJson(const Summary& summary)
{
   return json{{"createSummary", summary.createSummary},
               {"summaryTriggerVariable", summary.summaryTriggerVariable},
               {"summarizedVariables", summary.summarizedVariables}};
}

json toJson(const ReferenceFrameInformation& frames)
{
   return json{{"frameIndices", frames.frameIndices}, {"frameNames", frames.frameNames}};
}

json toJson(const ModelFileDescription& model)
{
   return json{{"hasModel", model.hasModel},
               {"name", model.name},
               {"modelLoaderClass", model.modelLoaderClass},
               {"resourceDirectories", model.resourceDirectories},
               {"modelFileSize", model.modelFileSize},
               {"hasResourceZip", model.hasResourceZip},
               {"resourceZipSize", model.resourceZipSize}};
}

json toJson(const Handshake& handshake)
{
   json registries = json::array();
   for (const YoRegistryDefinition& registry : handshake.registries)
      registries.push_back(toJson(registry));

   json variables = json::array();
   for (const YoVariableDefinition& variable : handshake.variables)
      variables.push_back(toJson(variable));

   json joints = json::array();
   for (const JointDefinition& joint : handshake.joints)
      joints.push_back(toJson(joint));

   json enumTypes = json::array();
   for (const EnumType& enumType : handshake.enumTypes)
      enumTypes.push_back(toJson(enumType));

   return json{{"dt", handshake.dt},
               {"registries", registries},
               {"variables", variables},
               {"joints", joints},
               // graphicObjects/artifacts/scs2YoGraphicDefinitions are always empty in this
               // phase (YoGraphics support is deferred scope) - Java always emits these keys
               // (every getter is serialized via reflection, empty sequence or not).
               {"graphicObjects", json::array()},
               {"artifacts", json::array()},
               {"scs2YoGraphicDefinitions", json::array()},
               {"enumTypes", enumTypes},
               {"referenceFrameInformation", toJson(handshake.referenceFrameInformation)},
               {"summary", toJson(handshake.summary)}};
}

json toJson(const Announcement& announcement)
{
   return json{{"identifier", announcement.identifier},
               {"name", announcement.name},
               {"hostName", announcement.hostName},
               {"reconnectKey", announcement.reconnectKey},
               {"modelFileDescription", toJson(announcement.modelFileDescription)},
               {"log", announcement.log}};
}
}

std::string toJsonString(const Handshake& handshake)
{
   json root;
   root["us::ihmc::robotDataLogger::Handshake"] = toJson(handshake);
   return root.dump();
}

std::string toJsonString(const Announcement& announcement)
{
   json root;
   root["us::ihmc::robotDataLogger::Announcement"] = toJson(announcement);
   return root.dump();
}
}
