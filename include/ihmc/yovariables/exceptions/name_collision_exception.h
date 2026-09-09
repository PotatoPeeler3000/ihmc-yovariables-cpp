#pragma once

#include <stdexcept>

namespace ihmc::yovariables::exceptions
{
/** Thrown to indicate that an operation attempted on an object resulted in a name collision. */
class NameCollisionException : public std::runtime_error
{
public:
   using std::runtime_error::runtime_error;
};
}
