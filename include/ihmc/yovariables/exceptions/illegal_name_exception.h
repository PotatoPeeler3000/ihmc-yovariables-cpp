#pragma once

#include <stdexcept>

namespace ihmc::yovariables::exceptions
{
/** Thrown to indicate that the name of an object contains illegal characters or is malformed. */
class IllegalNameException : public std::runtime_error
{
public:
   using std::runtime_error::runtime_error;
};
}
