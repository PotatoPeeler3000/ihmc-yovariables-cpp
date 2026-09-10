#pragma once

#include <cstdint>
#include <vector>

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::robotDataLogger
{
/**
 * Packs a snapshot of variable values (in handshake wire order - see
 * handshake::YoVariableHandShakeBuilder::getVariablesInWireOrder()) into the big-endian byte
 * layout that gets LZ4-compressed, mirroring Java's
 * `RegistrySendBuffer`/`RegistrySendBufferBuilder.updateBufferFromVariables()`. Joint states are
 * not modeled - joints are deferred scope for this phase (see the plan).
 */
class RegistrySendBuffer
{
public:
   explicit RegistrySendBuffer(std::vector<yovariables::variable::YoVariable*> variablesInWireOrder);

   /** Re-reads every variable's current value into the packed buffer. */
   void updateFromVariables();

   const std::vector<std::uint8_t>& rawValueBytes() const
   {
      return rawValueBytes_;
   }

   std::size_t numberOfVariables() const
   {
      return variables_.size();
   }

private:
   std::vector<yovariables::variable::YoVariable*> variables_;
   std::vector<std::uint8_t> rawValueBytes_;
};
}
