#pragma once

namespace ihmc::yovariables::parameters
{
class YoParameter;
}

namespace ihmc::yovariables::listener
{
/** Listener for changes to a YoParameter. */
class YoParameterChangedListener
{
public:
   virtual ~YoParameterChangedListener() = default;
   virtual void changed(parameters::YoParameter& source) = 0;
};
}
