#pragma once

namespace ihmc::yovariables::registry
{
class YoVariableHolder;
}

namespace ihmc::yovariables::buffer::interfaces
{
/** A function that can be used to read and/or modify the data in a YoBuffer. */
class YoBufferProcessor
{
public:
   virtual ~YoBufferProcessor() = default;

   /** True to travel the buffer in-point to out-point, false for out-point to in-point. */
   virtual bool goForward() const
   {
      return true;
   }

   virtual void initialize(registry::YoVariableHolder& yoVariableHolder)
   {
      (void) yoVariableHolder;
   }

   virtual void process(int startIndex, int endIndex, int currentIndex) = 0;
};
}
