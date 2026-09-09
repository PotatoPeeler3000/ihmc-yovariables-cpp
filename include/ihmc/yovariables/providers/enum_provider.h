#pragma once

#include <type_traits>

namespace ihmc::yovariables::providers
{
template <typename E>
class EnumProvider
{
   static_assert(std::is_enum<E>::value, "E must be an enum type");

public:
   virtual ~EnumProvider() = default;
   virtual E getValue() const = 0;
};
}
