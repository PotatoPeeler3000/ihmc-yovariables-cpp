#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ihmc::robotDataLogger::handshake
{
/** Mirrors logger_msgs.YoType - on the wire (JSON) these serialize as the string names below. */
enum class YoType
{
   DoubleYoVariable,
   BooleanYoVariable,
   IntegerYoVariable,
   LongYoVariable,
   EnumYoVariable
};

/** Mirrors logger_msgs.LoadStatus - on the wire (JSON) these serialize as the string names below. */
enum class LoadStatus
{
   NoParameter,
   Unloaded,
   Default,
   Loaded
};

/** Mirrors logger_msgs.YoRegistryDefinition. */
struct YoRegistryDefinition
{
   std::uint16_t parent = 0;
   std::string name;
};

/** Mirrors logger_msgs.YoVariableDefinition. */
struct YoVariableDefinition
{
   std::string name;
   std::string description;
   YoType type = YoType::DoubleYoVariable;
   std::uint16_t registry = 0;
   std::uint16_t enumType = 0;
   bool allowNullValues = false;
   bool isParameter = false;
   double min = 0.0;
   double max = 0.0;
   LoadStatus loadStatus = LoadStatus::NoParameter;
};

/** Mirrors logger_msgs.EnumType. */
struct EnumType
{
   std::string name;
   std::vector<std::string> enumValues;
};

/** Mirrors logger_msgs.JointDefinition. Always empty in this phase - joints are deferred scope. */
struct JointDefinition
{
   std::string name;
   std::uint8_t type = 0;
};

/** Mirrors logger_msgs.Summary. Always default (createSummary=false) in this phase. */
struct Summary
{
   bool createSummary = false;
   std::string summaryTriggerVariable;
   std::vector<std::string> summarizedVariables;
};

/** Mirrors logger_msgs.ReferenceFrameInformation. Always empty in this phase. */
struct ReferenceFrameInformation
{
   std::vector<std::int32_t> frameIndices;
   std::vector<std::string> frameNames;
};

/**
 * Mirrors logger_msgs.Handshake. `joints`, YoGraphics fields, `referenceFrameInformation`, and
 * `summary` are always emitted as Java-spec-correct empty/default values in this phase (see
 * YoVariableHandShakeBuilder) - full support is deferred scope.
 */
struct Handshake
{
   double dt = 0.0;
   std::vector<YoRegistryDefinition> registries;
   std::vector<YoVariableDefinition> variables;
   std::vector<JointDefinition> joints;
   std::vector<EnumType> enumTypes;
   ReferenceFrameInformation referenceFrameInformation;
   Summary summary;
};

/** Mirrors logger_msgs.ModelFileDescription. */
struct ModelFileDescription
{
   bool hasModel = false;
   std::string name;
   std::string modelLoaderClass;
   std::vector<std::string> resourceDirectories;
   std::int32_t modelFileSize = 0;
   bool hasResourceZip = false;
   std::int32_t resourceZipSize = 0;
};

/** Mirrors logger_msgs.Announcement. */
struct Announcement
{
   std::string identifier;
   std::string name;
   std::string hostName;
   std::string reconnectKey;
   ModelFileDescription modelFileDescription;
   bool log = false;
};
}
