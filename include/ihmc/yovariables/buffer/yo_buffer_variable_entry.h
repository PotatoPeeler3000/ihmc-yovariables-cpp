#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "ihmc/yovariables/buffer/interfaces/yo_buffer_variable_entry_reader.h"
#include "ihmc/yovariables/buffer/yo_buffer_bounds.h"

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::buffer
{
class AtomicDoubleArray;

/**
 * Manages the buffer to store history for a single YoVariable.
 * <p>
 * The write/read/bounds-tracking path (writeIntoBufferAt, readBufferAt, haveBoundsChanged,
 * resetBoundsChangedFlag, getBounds, getWindowBounds) is lock-free: bufferData_ is an array of
 * std::atomic&lt;double&gt; swapped as a whole via an atomic pointer, currentBounds_ is an
 * immutable YoBufferBounds swapped behind a reference-counted pointer via a compare-and-swap
 * retry loop (so a reader can never see a torn (lowerBound, upperBound) pair), and
 * boundsChanged_/boundsDirty_ are std::atomic&lt;bool&gt;. This is correct under any number of
 * concurrent writers, not just the single-writer pattern seen in this class's actual usage.
 * </p>
 * <p>
 * The buffer-resizing operations (enlargeBufferSize, cropBuffer, cutBuffer, shiftBuffer,
 * thinData, clearBuffer, fillBuffer) remain single-thread-only, exactly as in the Java source -
 * they were never synchronized/thread-safe even before this class used atomics. Unlike the Java
 * version, which relies on the JVM's garbage collector to keep an old bufferData_ array alive for
 * as long as some thread might still be reading it, this port frees the old array immediately on
 * swap: safe only because callers must not invoke a resize operation concurrently with any other
 * call on this object, which is the documented contract in both versions - the C++ port just has
 * no GC to blur the failure mode of violating it.
 * </p>
 */
class YoBufferVariableEntry : public interfaces::YoBufferVariableEntryReader
{
public:
   YoBufferVariableEntry(variable::YoVariable& variable, int bufferSize);

   /** Clone constructor. */
   explicit YoBufferVariableEntry(const YoBufferVariableEntry& other);
   YoBufferVariableEntry& operator=(const YoBufferVariableEntry&) = delete;
   ~YoBufferVariableEntry() override;

   void clearBuffer(int bufferSize);

   void setInverted(bool inverted) override;
   bool getInverted() const override;

   int getBufferSize() const override;

   /** Writes the current variable value into the buffer at the given index. */
   void writeIntoBufferAt(int index);

   /** Reads the buffer at the given index and updates the variable's current value. */
   void readFromBufferAt(int index);

   double readBufferAt(int index) const override;
   std::vector<double> getBuffer() const override;
   std::vector<double> getBufferWindow(int startIndex, int length) const override;

   void useCustomBounds(bool autoScale) override;
   bool isUsingCustomBounds() const override;

   variable::YoVariable& getVariable() const override;

   void fillBuffer();
   void enlargeBufferSize(int newSize);
   int cropBuffer(int start, int end);
   int cutBuffer(int start, int end);
   int thinData(int keepEveryNthPoint);
   void shiftBuffer(int shiftIndex);

   static int computeBufferSizeAfterCrop(int start, int end, int previousBufferSize);
   static int computeBufferSizeAfterCut(int start, int end, int previousBufferSize);

   void resetBoundsChangedFlag() override;
   bool haveBoundsChanged() const override;

   YoBufferBounds getBounds() override;
   YoBufferBounds getCustomBounds() override;

   /** The value average of the variable over the entire buffer. */
   double computeAverage() const;

   /**
    * The value average of the variable over a portion of the buffer.
    *
    * @param start  first buffer index to include. Should be in [0, getBufferSize()).
    * @param length number of elements to include. Should be in (0, getBufferSize()].
    */
   double computeAverage(int start, int length) const;

   YoBufferBounds getWindowBounds(int startIndex, int endIndex) override;

   /**
    * Tests whether this buffer and other are equal to an epsilon: same size, same variable full
    * name, and equal data.
    */
   bool epsilonEquals(const YoBufferVariableEntry& other, double epsilon) const;

   std::string toString() const;

private:
   /**
    * Writes value into this buffer at index. Private: nothing outside this class should be able to
    * write into the buffer without going through writeIntoBufferAt's well-defined semantics
    * (mirrors the Java source's package-private access, made stricter since C++ has no
    * package-private equivalent).
    */
   void writeBufferAt(double value, int index);
   void updateBounds();
   static std::vector<double> toDoubleArray(const AtomicDoubleArray& buffer);

   variable::YoVariable& variable_;
   std::atomic<AtomicDoubleArray*> bufferData_;
   std::shared_ptr<const YoBufferBounds> currentBounds_;
   std::atomic<bool> boundsChanged_{true};
   std::atomic<bool> boundsDirty_{true};
   bool useCustomBounds_ = false;
   YoBufferBounds customBounds_ = YoBufferBounds::EMPTY;
   bool inverted_ = false;
};
}
