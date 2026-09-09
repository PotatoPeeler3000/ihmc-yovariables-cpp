#pragma once

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::listener
{
/** Listener on the backing value of a YoVariable. */
class YoVariableChangedListener
{
public:
   virtual ~YoVariableChangedListener() = default;
   virtual void changed(variable::YoVariable& source) = 0;
};
}
