#include "ihmc/yovariables/buffer/yo_buffer_variable_entry.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::buffer
{
namespace
{
bool epsilonEquals(double a, double b, double epsilon)
{
   return std::abs(a - b) <= epsilon;
}
} // namespace

/** Fixed-size array of atomic doubles with intrinsic length, mirroring java.util.concurrent.atomic.AtomicLongArray's shape. */
class AtomicDoubleArray
{
public:
   explicit AtomicDoubleArray(int length) : data_(std::make_unique<std::atomic<double>[]>(static_cast<std::size_t>(length))), length_(length)
   {
      for (int i = 0; i < length; i++)
         data_[static_cast<std::size_t>(i)].store(0.0, std::memory_order_relaxed);
   }

   double get(int index) const
   {
      return data_[static_cast<std::size_t>(index)].load(std::memory_order_acquire);
   }

   void set(int index, double value)
   {
      data_[static_cast<std::size_t>(index)].store(value, std::memory_order_release);
   }

   int length() const
   {
      return length_;
   }

private:
   std::unique_ptr<std::atomic<double>[]> data_;
   int length_;
};

YoBufferVariableEntry::YoBufferVariableEntry(variable::YoVariable& variable, int bufferSize) : variable_(variable), bufferData_(nullptr)
{
   clearBuffer(bufferSize);
}

YoBufferVariableEntry::YoBufferVariableEntry(const YoBufferVariableEntry& other) : variable_(other.variable_), bufferData_(nullptr)
{
   AtomicDoubleArray* otherData = other.bufferData_.load(std::memory_order_acquire);
   auto* copy = new AtomicDoubleArray(otherData->length());
   for (int i = 0; i < otherData->length(); i++)
      copy->set(i, otherData->get(i));
   bufferData_.store(copy, std::memory_order_release);

   currentBounds_ = std::atomic_load(&other.currentBounds_);
   boundsChanged_.store(other.boundsChanged_.load());
   boundsDirty_.store(other.boundsDirty_.load());
   useCustomBounds_ = other.useCustomBounds_;
   customBounds_ = other.customBounds_;
   inverted_ = other.inverted_;
}

YoBufferVariableEntry::~YoBufferVariableEntry()
{
   delete bufferData_.load(std::memory_order_acquire);
}

void YoBufferVariableEntry::clearBuffer(int bufferSize)
{
   delete bufferData_.load(std::memory_order_acquire);
   bufferData_.store(new AtomicDoubleArray(bufferSize), std::memory_order_release);
   std::atomic_store(&currentBounds_, std::make_shared<const YoBufferBounds>(YoBufferBounds::EMPTY));
   boundsDirty_.store(true);
}

void YoBufferVariableEntry::setInverted(bool inverted)
{
   inverted_ = inverted;
}

bool YoBufferVariableEntry::getInverted() const
{
   return inverted_;
}

int YoBufferVariableEntry::getBufferSize() const
{
   return bufferData_.load(std::memory_order_acquire)->length();
}

void YoBufferVariableEntry::writeIntoBufferAt(int index)
{
   writeBufferAt(variable_.getValueAsDouble(), index);
}

void YoBufferVariableEntry::writeBufferAt(double value, int index)
{
   AtomicDoubleArray* buffer = bufferData_.load(std::memory_order_acquire);

   if (buffer->get(index) == value)
      return;

   buffer->set(index, value);

   std::shared_ptr<const YoBufferBounds> current = std::atomic_load(&currentBounds_);
   std::shared_ptr<const YoBufferBounds> widened;

   while (true)
   {
      YoBufferBounds widenedValue = current->widenedToInclude(value);
      if (widenedValue == *current)
         return;

      widened = std::make_shared<const YoBufferBounds>(widenedValue);
      if (std::atomic_compare_exchange_weak(&currentBounds_, &current, widened))
         break;
   }

   boundsChanged_.store(true, std::memory_order_release);
}

void YoBufferVariableEntry::readFromBufferAt(int index)
{
   variable_.setValueFromDouble(bufferData_.load(std::memory_order_acquire)->get(index));
}

double YoBufferVariableEntry::readBufferAt(int index) const
{
   return bufferData_.load(std::memory_order_acquire)->get(index);
}

std::vector<double> YoBufferVariableEntry::getBuffer() const
{
   return getBufferWindow(0, bufferData_.load(std::memory_order_acquire)->length());
}

std::vector<double> YoBufferVariableEntry::getBufferWindow(int startIndex, int length) const
{
   AtomicDoubleArray* buffer = bufferData_.load(std::memory_order_acquire);
   std::vector<double> sample(static_cast<std::size_t>(length));
   int n = startIndex;

   for (int i = 0; i < length; i++)
   {
      sample[static_cast<std::size_t>(i)] = buffer->get(n);
      n++;
      if (n >= buffer->length())
         n = 0;
   }

   return sample;
}

void YoBufferVariableEntry::useCustomBounds(bool autoScale)
{
   useCustomBounds_ = !autoScale;
}

bool YoBufferVariableEntry::isUsingCustomBounds() const
{
   return !useCustomBounds_;
}

variable::YoVariable& YoBufferVariableEntry::getVariable() const
{
   return variable_;
}

void YoBufferVariableEntry::fillBuffer()
{
   double value = variable_.getValueAsDouble();
   AtomicDoubleArray* buffer = bufferData_.load(std::memory_order_acquire);

   for (int i = 0; i < buffer->length(); i++)
      buffer->set(i, value);

   std::atomic_store(&currentBounds_, std::make_shared<const YoBufferBounds>(YoBufferBounds::EMPTY));
}

void YoBufferVariableEntry::enlargeBufferSize(int newSize)
{
   AtomicDoubleArray* oldData = bufferData_.load(std::memory_order_acquire);
   int oldNPoints = oldData->length();

   auto* newData = new AtomicDoubleArray(newSize);

   for (int i = 0; i < oldNPoints; i++)
      newData->set(i, oldData->get(i));

   double lastValue = oldData->get(oldNPoints - 1);
   for (int i = oldNPoints; i < newData->length(); i++)
      newData->set(i, lastValue);

   bufferData_.store(newData, std::memory_order_release);
   delete oldData;
   boundsDirty_.store(true);
}

int YoBufferVariableEntry::cropBuffer(int start, int end)
{
   AtomicDoubleArray* oldData = bufferData_.load(std::memory_order_acquire);

   if (start < 0 || end > oldData->length())
      return -1;

   int oldNPoints = oldData->length();
   int nPoints = computeBufferSizeAfterCrop(start, end, oldNPoints);

   auto* newData = new AtomicDoubleArray(nPoints);

   for (int i = 0; i < newData->length(); i++)
      newData->set(i, oldData->get((i + start) % oldNPoints));

   bufferData_.store(newData, std::memory_order_release);
   delete oldData;
   boundsDirty_.store(true);

   return newData->length();
}

int YoBufferVariableEntry::cutBuffer(int start, int end)
{
   if (start > end)
      return -1;

   AtomicDoubleArray* oldData = bufferData_.load(std::memory_order_acquire);

   if (start < 0 || end > oldData->length())
      return -1;

   int oldNPoints = oldData->length();
   int nPoints = computeBufferSizeAfterCut(start, end, oldNPoints);

   if (nPoints == 0)
      nPoints = oldNPoints;
   auto* newData = new AtomicDoubleArray(nPoints);

   int difference = end - start + 1;
   for (int i = 0; i < start; i++)
      newData->set(i, oldData->get(i));

   for (int i = end + 1; i < oldNPoints; i++)
      newData->set(i - difference, oldData->get(i));

   bufferData_.store(newData, std::memory_order_release);
   delete oldData;
   boundsDirty_.store(true);

   return newData->length();
}

int YoBufferVariableEntry::thinData(int keepEveryNthPoint)
{
   AtomicDoubleArray* oldData = bufferData_.load(std::memory_order_acquire);
   int oldNPoints = oldData->length();

   int newNumberOfPoints = oldNPoints / keepEveryNthPoint;
   auto* newData = new AtomicDoubleArray(newNumberOfPoints);

   int oldDataIndex = 0;
   for (int index = 0; index < newNumberOfPoints; index++)
   {
      newData->set(index, oldData->get(oldDataIndex));
      oldDataIndex += keepEveryNthPoint;
   }

   bufferData_.store(newData, std::memory_order_release);
   delete oldData;

   return newNumberOfPoints;
}

int YoBufferVariableEntry::computeBufferSizeAfterCrop(int start, int end, int previousBufferSize)
{
   int newBufferSize = (end - start + 1 + previousBufferSize) % previousBufferSize;
   return newBufferSize == 0 ? previousBufferSize : newBufferSize;
}

int YoBufferVariableEntry::computeBufferSizeAfterCut(int start, int end, int previousBufferSize)
{
   return previousBufferSize - (end - start + 1);
}

void YoBufferVariableEntry::shiftBuffer(int shiftIndex)
{
   AtomicDoubleArray* oldData = bufferData_.load(std::memory_order_acquire);
   int nPoints = oldData->length();

   if (shiftIndex <= 0 || shiftIndex >= nPoints)
      return;

   auto* newData = new AtomicDoubleArray(nPoints);

   for (int i = 0; i < nPoints; i++)
      newData->set(i, oldData->get((i + shiftIndex) % nPoints));

   bufferData_.store(newData, std::memory_order_release);
   delete oldData;
   boundsDirty_.store(true);
}

void YoBufferVariableEntry::resetBoundsChangedFlag()
{
   boundsChanged_.store(false);
}

bool YoBufferVariableEntry::haveBoundsChanged() const
{
   return boundsChanged_.load();
}

std::vector<double> YoBufferVariableEntry::toDoubleArray(const AtomicDoubleArray& buffer)
{
   std::vector<double> result(static_cast<std::size_t>(buffer.length()));
   for (int i = 0; i < buffer.length(); i++)
      result[static_cast<std::size_t>(i)] = buffer.get(i);
   return result;
}

void YoBufferVariableEntry::updateBounds()
{
   AtomicDoubleArray* buffer = bufferData_.load(std::memory_order_acquire);
   if (buffer == nullptr)
   {
      boundsChanged_.store(false);
      return;
   }

   std::vector<double> bufferSnapshot = toDoubleArray(*buffer);
   std::shared_ptr<const YoBufferBounds> oldBounds = std::atomic_load(&currentBounds_);
   auto newBounds = std::make_shared<const YoBufferBounds>(YoBufferBounds::computed(0, static_cast<int>(bufferSnapshot.size()) - 1, bufferSnapshot));
   std::atomic_store(&currentBounds_, newBounds);
   boundsChanged_.store(newBounds->getLowerBound() != oldBounds->getLowerBound() || newBounds->getUpperBound() != oldBounds->getUpperBound());
}

YoBufferBounds YoBufferVariableEntry::getBounds()
{
   if (boundsDirty_.load())
      updateBounds();

   return *std::atomic_load(&currentBounds_);
}

YoBufferBounds YoBufferVariableEntry::getCustomBounds()
{
   YoBufferBounds updated = YoBufferBounds::EMPTY.withInterval(0, getBufferSize() - 1).withBounds(variable_.getLowerBound(), variable_.getUpperBound());
   customBounds_ = updated;
   return updated;
}

double YoBufferVariableEntry::computeAverage() const
{
   return computeAverage(0, getBufferSize());
}

double YoBufferVariableEntry::computeAverage(int start, int length) const
{
   if (start < 0 || start >= getBufferSize())
      throw std::out_of_range("start should be in [0, " + std::to_string(getBufferSize()) + "), but was: " + std::to_string(start));
   if (length <= 0 || length > getBufferSize())
      throw std::out_of_range("length should be in (0, " + std::to_string(getBufferSize()) + "], but was: " + std::to_string(length));

   AtomicDoubleArray* buffer = bufferData_.load(std::memory_order_acquire);
   double total = 0.0;
   int count = 0;
   int index = 0;

   while (count < length)
   {
      total += buffer->get(index);

      count++;
      index++;
      if (index >= getBufferSize())
         index = 0;
   }

   return total / length;
}

YoBufferBounds YoBufferVariableEntry::getWindowBounds(int startIndex, int endIndex)
{
   AtomicDoubleArray* buffer = bufferData_.load(std::memory_order_acquire);
   if (buffer == nullptr)
      return *std::atomic_load(&currentBounds_);

   std::shared_ptr<const YoBufferBounds> oldBounds = std::atomic_load(&currentBounds_);

   if (boundsDirty_.load() || startIndex != oldBounds->getStartIndex() || endIndex != oldBounds->getEndIndex())
   {
      std::vector<double> bufferSnapshot = toDoubleArray(*buffer);
      auto newBounds = std::make_shared<const YoBufferBounds>(YoBufferBounds::computed(startIndex, endIndex, bufferSnapshot));
      std::atomic_store(&currentBounds_, newBounds);
      boundsChanged_.store(newBounds->getLowerBound() != oldBounds->getLowerBound() || newBounds->getUpperBound() != oldBounds->getUpperBound());
      boundsDirty_.store(false);
      return *newBounds;
   }

   return *oldBounds;
}

bool YoBufferVariableEntry::epsilonEquals(const YoBufferVariableEntry& other, double epsilon) const
{
   if (getBufferSize() != other.getBufferSize())
      return false;
   if (getVariableFullNameString() != other.getVariableFullNameString())
      return false;

   AtomicDoubleArray* thisData = bufferData_.load(std::memory_order_acquire);
   AtomicDoubleArray* otherData = other.bufferData_.load(std::memory_order_acquire);

   for (int i = 0; i < getBufferSize(); i++)
   {
      double thisDataPoint = thisData->get(i);
      double otherDataPoint = otherData->get(i);

      if (thisDataPoint != otherDataPoint && !buffer::epsilonEquals(thisDataPoint, otherDataPoint, epsilon))
         return false;
   }

   return true;
}

std::string YoBufferVariableEntry::toString() const
{
   std::ostringstream out;
   out << "variable: " << variable_.getName() << ", buffer size: " << getBufferSize();
   return out.str();
}
}
