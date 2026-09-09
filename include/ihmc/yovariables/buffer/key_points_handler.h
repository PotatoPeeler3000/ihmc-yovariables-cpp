#pragma once

#include <vector>

#include "ihmc/yovariables/buffer/interfaces/key_points_holder.h"

namespace ihmc::yovariables::buffer
{
/**
 * Allows selecting and storing buffer indices as key points.
 * <p>
 * A key point can be used to highlight a specific index in the buffer for any reason - typically
 * to let a user highlight particular events in the buffer and visualize them in a GUI.
 * </p>
 * <p>
 * Maintains an increasing order in index value when updating its key points.
 * </p>
 */
class KeyPointsHandler : public interfaces::KeyPointsHolder
{
public:
   void clear();

   void addListener(interfaces::KeyPointsChangedListener* listenerToAdd) override;
   void removeListeners();
   bool removeListener(interfaces::KeyPointsChangedListener* listenerToRemove);

   void toggleKeyPoints() override;

   /** Convenience flag; this class only tracks its value, see areKeyPointsEnabled(). */
   void enableKeyPoints(bool enable);
   bool areKeyPointsEnabled() const override;

   /** Toggles the presence of a key point at bufferIndex: adds if absent, removes if present. */
   bool toggleKeyPoint(int bufferIndex);

   /** Adds a key point at bufferIndex. Does nothing if one is already there. */
   bool addKeyPoint(int bufferIndex);

   /** Removes the key point at bufferIndex. Does nothing if there is none. */
   bool removeKeyPoint(int bufferIndex);

   /** The next key point index greater than bufferIndex, wrapping to the smallest if none. */
   int getNextKeyPoint(int bufferIndex) const;

   /** The previous key point index less than bufferIndex, wrapping to the largest if none. */
   int getPreviousKeyPoint(int bufferIndex) const;

   /** Removes every key point outside [startBufferIndex, endBufferIndex]. */
   void trimKeyPoints(int startBufferIndex, int endBufferIndex);

   const std::vector<int>& getKeyPoints() const;

private:
   void notifyAddedKeyPoint(int addedKeyPoint);
   void notifyAddedKeyPoints(const std::vector<int>& addedKeyPoints);
   void notifyRemovedKeyPoint(int removedKeyPoint);
   void notifyRemovedKeyPoints(const std::vector<int>& removedKeyPoints);

   bool enableKeyPoints_ = false;
   std::vector<int> keyPoints_;
   std::vector<interfaces::KeyPointsChangedListener*> listeners_;
};
}
