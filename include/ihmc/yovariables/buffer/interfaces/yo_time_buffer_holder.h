#pragma once

#include <vector>

namespace ihmc::yovariables::buffer::interfaces
{
/** Minimalist interface for a class managing a time variable buffer. */
class YoTimeBufferHolder
{
public:
   virtual ~YoTimeBufferHolder() = default;
   virtual std::vector<double> getTimeBuffer() const = 0;
};
}
