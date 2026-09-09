#pragma once

#include <vector>

namespace ihmc::yovariables::buffer::interfaces
{
class KeyPointsHolder;

/** Receives notifications of changes to a KeyPointsHolder. */
class KeyPointsChangedListener
{
public:
   class Change
   {
   public:
      virtual ~Change() = default;
      virtual bool wasToggled() const = 0;
      virtual bool areKeyPointsEnabled() const = 0;
      virtual const std::vector<int>& getAddedKeyPoints() const = 0;
      virtual const std::vector<int>& getRemovedKeyPoints() const = 0;
   };

   virtual ~KeyPointsChangedListener() = default;
   virtual void changed(const Change& change) = 0;
};
}
