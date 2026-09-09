#pragma once

#include <cstdint>

namespace ihmc::yovariables::providers
{
class IntegerProvider
{
public:
   virtual ~IntegerProvider() = default;
   virtual std::int32_t getValue() const = 0;
};
}
