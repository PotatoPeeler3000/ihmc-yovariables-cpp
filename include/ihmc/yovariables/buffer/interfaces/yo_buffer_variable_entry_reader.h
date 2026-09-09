#pragma once

#include <string>
#include <vector>

#include "ihmc/yovariables/buffer/yo_buffer_bounds.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::buffer::interfaces
{
/** Base interface for reading from a YoBufferVariableEntry, which manages the buffer for a single YoVariable. */
class YoBufferVariableEntryReader
{
public:
   virtual ~YoBufferVariableEntryReader() = default;

   virtual variable::YoVariable& getVariable() const = 0;

   std::string getVariableName() const
   {
      return getVariable().getName();
   }

   std::string getVariableFullNameString() const
   {
      return getVariable().getFullNameString();
   }

   virtual int getBufferSize() const = 0;

   virtual double readBufferAt(int index) const = 0;
   virtual std::vector<double> getBuffer() const = 0;
   virtual std::vector<double> getBufferWindow(int startIndex, int length) const = 0;

   /** Marks an internal flag; the next call to haveBoundsChanged() is guaranteed to return false. */
   virtual void resetBoundsChangedFlag() = 0;

   virtual bool haveBoundsChanged() const = 0;

   /**
    * The current bounds for the data in this buffer, updated if necessary. Returned by value:
    * YoBufferBounds is a small immutable value type, and the concurrency-safe implementation
    * backing this internally swaps bounds instances behind a reference-counted pointer that a
    * caller must not outlive a reference into - a copy sidesteps that entirely.
    */
   virtual buffer::YoBufferBounds getBounds() = 0;

   double getLowerBound()
   {
      return getBounds().getLowerBound();
   }

   double getUpperBound()
   {
      return getBounds().getUpperBound();
   }

   /** Bounds on the buffer values for the index interval [startIndex, endIndex]. */
   virtual buffer::YoBufferBounds getWindowBounds(int startIndex, int endIndex) = 0;

   double getWindowLowerBound(int startIndex, int endIndex)
   {
      return getWindowBounds(startIndex, endIndex).getLowerBound();
   }

   double getWindowUpperBound(int startIndex, int endIndex)
   {
      return getWindowBounds(startIndex, endIndex).getUpperBound();
   }

   /** Convenience flag, does not affect the rest of this entry's internal state. */
   virtual void useCustomBounds(bool useCustomBounds) = 0;
   virtual bool isUsingCustomBounds() const = 0;

   /** Custom bounds, saved as the variable's bounds; does not affect this entry's internal state. */
   void setCustomBounds(double customLowerBound, double customUpperBound)
   {
      getVariable().setVariableBounds(customLowerBound, customUpperBound);
   }

   virtual buffer::YoBufferBounds getCustomBounds() = 0;

   double getCustomLowerBound()
   {
      return getCustomBounds().getLowerBound();
   }

   double getCustomUpperBound()
   {
      return getCustomBounds().getUpperBound();
   }

   /** Convenience flag, does not affect the rest of this entry's internal state. */
   virtual void setInverted(bool inverted) = 0;
   virtual bool getInverted() const = 0;
};
}
