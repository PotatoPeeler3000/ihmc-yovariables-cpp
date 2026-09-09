#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "ihmc/robotDataLogger/handshake/handshake_types.h"

namespace ihmc::yovariables::registry
{
class YoRegistry;
}

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::robotDataLogger::handshake
{
/**
 * Builds a Handshake from a YoRegistry tree, mirroring Java's
 * `us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder`. Registry IDs are assigned by the
 * exact same pre-order depth-first traversal Java uses (a registry's own variables are appended
 * before its children are visited), starting at 1 - ID 0 is reserved for a synthetic root entry the
 * constructor creates, matching Java's `createRootRegistry()`. This ordering is load-bearing: the
 * Java `YoVariableClient` reconstructs its registry tree and resolves variable values purely by
 * position/ID from this handshake, with no name-based lookup on the wire.
 * <p>
 * Joints, YoGraphics, `Summary`, and `ReferenceFrameInformation` are not populated by this phase -
 * they are always emitted as Java-spec-correct empty/default values (see handshake_types.h).
 * </p>
 */
class YoVariableHandShakeBuilder
{
public:
   YoVariableHandShakeBuilder(const std::string& rootRegistryName, double dt);

   /**
    * Walks rootRegistry's subtree, populating the handshake and the wire-order variable list.
    * May only be called once per builder instance (mirrors Java's addRegistryBuffer(), which a
    * YoVariableServer calls exactly once per top-level registry it publishes).
    */
   void build(yovariables::registry::YoRegistry& rootRegistry);

   const Handshake& getHandshake() const
   {
      return handshake_;
   }

   /**
    * The flat, ordered list of variables collected during build(), in exactly the order their
    * values must be packed on the wire (see RegistrySendBuffer) - mirrors Java's
    * `variableListToPack`/`RegistrySendBufferBuilder`'s variable list.
    */
   const std::vector<yovariables::variable::YoVariable*>& getVariablesInWireOrder() const
   {
      return variablesInWireOrder_;
   }

private:
   int addRegistry(int parentID, yovariables::registry::YoRegistry& registry);
   void addVariables(int registryID, yovariables::registry::YoRegistry& registry);
   std::uint16_t getOrAddEnumType(const std::string& key, const std::vector<std::string>& enumValues);

   Handshake handshake_;
   int nextRegistryID_ = 1;
   std::unordered_map<std::string, std::uint16_t> enumTypeIndexByKey_;
   std::vector<yovariables::variable::YoVariable*> variablesInWireOrder_;
};
}
