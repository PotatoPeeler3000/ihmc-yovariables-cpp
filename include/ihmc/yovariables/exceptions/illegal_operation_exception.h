#pragma once

#include <stdexcept>

namespace ihmc::yovariables::exceptions
{
/** Thrown to indicate that an operation attempted on an object resulted in an illegal operation. */
class IllegalOperationException : public std::runtime_error
{
public:
   using std::runtime_error::runtime_error;
};
}
