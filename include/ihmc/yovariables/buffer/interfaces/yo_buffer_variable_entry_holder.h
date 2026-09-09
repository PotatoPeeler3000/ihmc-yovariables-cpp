#pragma once

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::buffer::interfaces
{
class YoBufferVariableEntryReader;

/** Minimalist interface for a class managing variable buffers. */
class YoBufferVariableEntryHolder
{
public:
   virtual ~YoBufferVariableEntryHolder() = default;

   /** This variable's buffer entry, or nullptr if it could not be found. */
   virtual YoBufferVariableEntryReader* getEntry(variable::YoVariable& variable) = 0;
};
}
