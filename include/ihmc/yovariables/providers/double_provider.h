#pragma once

namespace ihmc::yovariables::providers
{
class DoubleProvider
{
public:
   virtual ~DoubleProvider() = default;
   virtual double getValue() const = 0;
};
}
