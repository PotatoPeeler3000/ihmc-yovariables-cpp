#pragma once

namespace ihmc::yovariables::buffer::interfaces
{
/**
 * Base interface for reading from a YoBuffer, which manages the buffers for a collection of
 * YoVariables.
 */
class YoBufferReader
{
public:
   virtual ~YoBufferReader() = default;

   virtual int getInPoint() const = 0;
   virtual int getOutPoint() const = 0;
   virtual int getCurrentIndex() const = 0;
   virtual int getBufferSize() const = 0;

   /** Sets the current buffer index, reads the buffer at the new index, updates the variables. */
   virtual void setCurrentIndex(int index) = 0;

   /**
    * Increments/decrements the current buffer index by stepSize, reads the buffer at the new
    * index, updates the variables.
    *
    * @return true if the current index rolled over to the beginning of the buffer.
    */
   virtual bool tickAndReadFromBuffer(int stepSize) = 0;

   /** The length of [inPoint, outPoint]. */
   virtual int getBufferInOutLength() const
   {
      if (getOutPoint() > getInPoint())
         return getOutPoint() - getInPoint() + 1;
      return getBufferSize() - (getInPoint() - getOutPoint()) + 1;
   }

   /** True if indexToCheck lies in [inPoint, outPoint], accounting for wraparound. */
   virtual bool isIndexBetweenBounds(int indexToCheck) const
   {
      if (indexToCheck < 0 || indexToCheck >= getBufferSize())
         return false;
      if (getInPoint() <= getOutPoint())
         return indexToCheck >= getInPoint() && indexToCheck <= getOutPoint();
      return indexToCheck <= getOutPoint() || indexToCheck > getInPoint();
   }
};
}
