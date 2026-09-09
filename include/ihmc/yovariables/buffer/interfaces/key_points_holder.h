#pragma once

#include "ihmc/yovariables/buffer/interfaces/key_points_changed_listener.h"

namespace ihmc::yovariables::buffer::interfaces
{
/** Base interface for a class that manages a collection of buffer key points. */
class KeyPointsHolder
{
public:
   virtual ~KeyPointsHolder() = default;
   virtual void toggleKeyPoints() = 0;
   virtual bool areKeyPointsEnabled() const = 0;
   virtual void addListener(KeyPointsChangedListener* listenerToAdd) = 0;
};
}
