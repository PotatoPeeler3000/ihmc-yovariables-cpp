#include "ihmc/robotDataLogger/registry_send_buffer.h"

#include "ihmc/robotDataLogger/wire/cdr_buffer.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::robotDataLogger
{
RegistrySendBuffer::RegistrySendBuffer(std::vector<yovariables::variable::YoVariable*> variablesInWireOrder)
   : variables_(std::move(variablesInWireOrder))
{
   rawValueBytes_.reserve(variables_.size() * 8);
}

void RegistrySendBuffer::updateFromVariables()
{
   rawValueBytes_.clear();
   for (yovariables::variable::YoVariable* variable : variables_)
      wire::appendBigEndianInt64(rawValueBytes_, variable->getValueAsLongBits());
}
}
