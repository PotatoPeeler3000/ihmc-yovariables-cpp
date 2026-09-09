#pragma once

namespace ihmc::yovariables::providers
{
class BooleanProvider
{
public:
   virtual ~BooleanProvider() = default;
   virtual bool getValue() const = 0;
};
}
