#include "ihmc/yovariables/buffer/key_points_handler.h"

#include <algorithm>

namespace ihmc::yovariables::buffer
{
namespace
{
class KeyPointsChange : public interfaces::KeyPointsChangedListener::Change
{
public:
   KeyPointsChange(bool wasToggled, bool areKeyPointsEnabled, std::vector<int> addedKeyPoints, std::vector<int> removedKeyPoints)
      : wasToggled_(wasToggled), areKeyPointsEnabled_(areKeyPointsEnabled), addedKeyPoints_(std::move(addedKeyPoints)),
        removedKeyPoints_(std::move(removedKeyPoints))
   {
   }

   bool wasToggled() const override { return wasToggled_; }
   bool areKeyPointsEnabled() const override { return areKeyPointsEnabled_; }
   const std::vector<int>& getAddedKeyPoints() const override { return addedKeyPoints_; }
   const std::vector<int>& getRemovedKeyPoints() const override { return removedKeyPoints_; }

private:
   bool wasToggled_;
   bool areKeyPointsEnabled_;
   std::vector<int> addedKeyPoints_;
   std::vector<int> removedKeyPoints_;
};
} // namespace

void KeyPointsHandler::clear()
{
   keyPoints_.clear();
   listeners_.clear();
}

void KeyPointsHandler::addListener(interfaces::KeyPointsChangedListener* listenerToAdd)
{
   listeners_.push_back(listenerToAdd);
}

void KeyPointsHandler::removeListeners()
{
   listeners_.clear();
}

bool KeyPointsHandler::removeListener(interfaces::KeyPointsChangedListener* listenerToRemove)
{
   auto it = std::find(listeners_.begin(), listeners_.end(), listenerToRemove);
   if (it == listeners_.end())
      return false;
   listeners_.erase(it);
   return true;
}

void KeyPointsHandler::toggleKeyPoints()
{
   enableKeyPoints_ = !enableKeyPoints_;
   KeyPointsChange change(true, enableKeyPoints_, {}, {});
   for (interfaces::KeyPointsChangedListener* listener : listeners_)
      listener->changed(change);
}

void KeyPointsHandler::enableKeyPoints(bool enable)
{
   if (enable != enableKeyPoints_)
      toggleKeyPoints();
}

bool KeyPointsHandler::areKeyPointsEnabled() const
{
   return enableKeyPoints_;
}

bool KeyPointsHandler::toggleKeyPoint(int bufferIndex)
{
   for (std::size_t i = 0; i < keyPoints_.size(); i++)
   {
      if (keyPoints_[i] == bufferIndex)
      {
         keyPoints_.erase(keyPoints_.begin() + static_cast<std::ptrdiff_t>(i));
         notifyRemovedKeyPoint(bufferIndex);
         return false;
      }

      if (keyPoints_[i] > bufferIndex)
      {
         keyPoints_.insert(keyPoints_.begin() + static_cast<std::ptrdiff_t>(i), bufferIndex);
         notifyAddedKeyPoint(bufferIndex);
         return true;
      }
   }

   keyPoints_.push_back(bufferIndex);
   return true;
}

bool KeyPointsHandler::addKeyPoint(int bufferIndex)
{
   for (std::size_t i = 0; i < keyPoints_.size(); i++)
   {
      if (keyPoints_[i] == bufferIndex)
         return false;

      if (keyPoints_[i] > bufferIndex)
      {
         keyPoints_.insert(keyPoints_.begin() + static_cast<std::ptrdiff_t>(i), bufferIndex);
         notifyAddedKeyPoint(bufferIndex);
         return true;
      }
   }

   keyPoints_.push_back(bufferIndex);
   notifyAddedKeyPoint(bufferIndex);
   return true;
}

bool KeyPointsHandler::removeKeyPoint(int bufferIndex)
{
   for (std::size_t i = 0; i < keyPoints_.size(); i++)
   {
      if (keyPoints_[i] == bufferIndex)
      {
         keyPoints_.erase(keyPoints_.begin() + static_cast<std::ptrdiff_t>(i));
         notifyRemovedKeyPoint(bufferIndex);
         return true;
      }

      if (keyPoints_[i] > bufferIndex)
         return false;
   }
   return false;
}

int KeyPointsHandler::getNextKeyPoint(int bufferIndex) const
{
   for (int keyPoint : keyPoints_)
   {
      if (keyPoint > bufferIndex)
         return keyPoint;
   }

   if (!keyPoints_.empty())
      return keyPoints_.front();

   return bufferIndex;
}

int KeyPointsHandler::getPreviousKeyPoint(int bufferIndex) const
{
   for (auto it = keyPoints_.rbegin(); it != keyPoints_.rend(); ++it)
   {
      if (*it < bufferIndex)
         return *it;
   }

   if (!keyPoints_.empty())
      return keyPoints_.back();

   return bufferIndex;
}

void KeyPointsHandler::trimKeyPoints(int startBufferIndex, int endBufferIndex)
{
   std::vector<int> removedKeyPoints;

   for (std::size_t i = 0; i < keyPoints_.size();)
   {
      bool outsideInterval;
      if (startBufferIndex < endBufferIndex)
         outsideInterval = keyPoints_[i] < startBufferIndex || keyPoints_[i] > endBufferIndex;
      else
         outsideInterval = keyPoints_[i] < startBufferIndex && keyPoints_[i] > endBufferIndex;

      if (outsideInterval)
      {
         removedKeyPoints.push_back(keyPoints_[i]);
         keyPoints_.erase(keyPoints_.begin() + static_cast<std::ptrdiff_t>(i));
      }
      else
      {
         i++;
      }
   }

   notifyRemovedKeyPoints(removedKeyPoints);
}

const std::vector<int>& KeyPointsHandler::getKeyPoints() const
{
   return keyPoints_;
}

void KeyPointsHandler::notifyAddedKeyPoint(int addedKeyPoint)
{
   notifyAddedKeyPoints({addedKeyPoint});
}

void KeyPointsHandler::notifyAddedKeyPoints(const std::vector<int>& addedKeyPoints)
{
   KeyPointsChange change(false, enableKeyPoints_, addedKeyPoints, {});
   for (interfaces::KeyPointsChangedListener* listener : listeners_)
      listener->changed(change);
}

void KeyPointsHandler::notifyRemovedKeyPoint(int removedKeyPoint)
{
   notifyRemovedKeyPoints({removedKeyPoint});
}

void KeyPointsHandler::notifyRemovedKeyPoints(const std::vector<int>& removedKeyPoints)
{
   KeyPointsChange change(false, enableKeyPoints_, {}, removedKeyPoints);
   for (interfaces::KeyPointsChangedListener* listener : listeners_)
      listener->changed(change);
}
}
