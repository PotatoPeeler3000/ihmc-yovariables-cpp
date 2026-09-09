#pragma once

#include <cstdint>

namespace ihmc::yovariables::providers
{
class LongProvider
{
public:
   virtual ~LongProvider() = default;
   virtual std::int64_t getValue() const = 0;
};
}
