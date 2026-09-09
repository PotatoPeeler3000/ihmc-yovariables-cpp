#pragma once

namespace ihmc::yovariables::buffer::interfaces
{
/** Receives notifications of changes to the current index of a YoBuffer. */
class YoBufferIndexChangedListener
{
public:
   virtual ~YoBufferIndexChangedListener() = default;
   virtual void indexChanged(int newIndex) = 0;
};
}
