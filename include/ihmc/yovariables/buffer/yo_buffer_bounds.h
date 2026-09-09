#pragma once

#include <string>
#include <vector>

namespace ihmc::yovariables::buffer
{
/**
 * Immutable value type storing the lower and upper bounds to a series of double values,
 * typically from an indexed buffer.
 * <p>
 * Immutable specifically so YoBufferVariableEntry can hold it behind an atomically-swapped
 * reference-counted pointer and swap one instance for another as a single atomic operation - a
 * reader can then never observe a torn (lowerBound, upperBound) pair while a writer thread is
 * concurrently widening the bounds, which two separate mutable fields could not guarantee.
 * </p>
 */
class YoBufferBounds
{
public:
   /** A cleared instance: indices -1, lower bound +infinity, upper bound -infinity. */
   static const YoBufferBounds EMPTY;

   YoBufferBounds withInterval(int startIndex, int endIndex) const;
   YoBufferBounds withBounds(double lowerBound, double upperBound) const;

   /** Computes the bounds of buffer within the interval [startIndex, endIndex]. */
   static YoBufferBounds computed(int startIndex, int endIndex, const std::vector<double>& buffer);

   /**
    * A new bounds widened to include value, keeping this instance's index window - or this same
    * value if value is already inside the current bounds.
    */
   YoBufferBounds widenedToInclude(double value) const;

   bool isInsideBounds(double value) const;

   int getStartIndex() const;
   int getEndIndex() const;
   double getLowerBound() const;
   double getUpperBound() const;

   bool operator==(const YoBufferBounds& other) const;
   bool operator!=(const YoBufferBounds& other) const;

   std::string toString() const;

private:
   YoBufferBounds(int startIndex, int endIndex, double lowerBound, double upperBound);

   int startIndex_;
   int endIndex_;
   double lowerBound_;
   double upperBound_;
};
}
