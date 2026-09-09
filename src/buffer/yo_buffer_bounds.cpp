#include "ihmc/yovariables/buffer/yo_buffer_bounds.h"

#include <algorithm>
#include <limits>
#include <sstream>

namespace ihmc::yovariables::buffer
{
const YoBufferBounds YoBufferBounds::EMPTY(-1, -1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());

YoBufferBounds::YoBufferBounds(int startIndex, int endIndex, double lowerBound, double upperBound)
   : startIndex_(startIndex), endIndex_(endIndex), lowerBound_(lowerBound), upperBound_(upperBound)
{
}

YoBufferBounds YoBufferBounds::withInterval(int startIndex, int endIndex) const
{
   return YoBufferBounds(startIndex, endIndex, lowerBound_, upperBound_);
}

YoBufferBounds YoBufferBounds::withBounds(double lowerBound, double upperBound) const
{
   return YoBufferBounds(startIndex_, endIndex_, lowerBound, upperBound);
}

YoBufferBounds YoBufferBounds::computed(int startIndex, int endIndex, const std::vector<double>& buffer)
{
   double newLowerBound = std::numeric_limits<double>::infinity();
   double newUpperBound = -std::numeric_limits<double>::infinity();

   if (startIndex < endIndex)
   {
      for (int i = startIndex; i < endIndex; i++)
      {
         double value = buffer[i];
         if (value < newLowerBound)
            newLowerBound = value;
         if (value > newUpperBound)
            newUpperBound = value;
      }
   }
   else
   {
      for (std::size_t i = static_cast<std::size_t>(startIndex); i < buffer.size(); i++)
      {
         double value = buffer[i];
         if (value < newLowerBound)
            newLowerBound = value;
         if (value > newUpperBound)
            newUpperBound = value;
      }

      for (int i = 0; i < endIndex; i++)
      {
         double value = buffer[i];
         if (value < newLowerBound)
            newLowerBound = value;
         if (value > newUpperBound)
            newUpperBound = value;
      }
   }

   return YoBufferBounds(startIndex, endIndex, newLowerBound, newUpperBound);
}

YoBufferBounds YoBufferBounds::widenedToInclude(double value) const
{
   if (value >= lowerBound_ && value <= upperBound_)
      return *this;
   return YoBufferBounds(startIndex_, endIndex_, std::min(lowerBound_, value), std::max(upperBound_, value));
}

bool YoBufferBounds::isInsideBounds(double value) const
{
   return value >= lowerBound_ && value <= upperBound_;
}

int YoBufferBounds::getStartIndex() const
{
   return startIndex_;
}

int YoBufferBounds::getEndIndex() const
{
   return endIndex_;
}

double YoBufferBounds::getLowerBound() const
{
   return lowerBound_;
}

double YoBufferBounds::getUpperBound() const
{
   return upperBound_;
}

bool YoBufferBounds::operator==(const YoBufferBounds& other) const
{
   return startIndex_ == other.startIndex_ && endIndex_ == other.endIndex_ && lowerBound_ == other.lowerBound_ && upperBound_ == other.upperBound_;
}

bool YoBufferBounds::operator!=(const YoBufferBounds& other) const
{
   return !(*this == other);
}

std::string YoBufferBounds::toString() const
{
   std::ostringstream out;
   out << "[" << startIndex_ << ", " << endIndex_ << "] -> [" << lowerBound_ << ", " << upperBound_ << "]";
   return out.str();
}
}
