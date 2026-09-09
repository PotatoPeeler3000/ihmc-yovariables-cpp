#pragma once

#include <optional>

#include "ihmc/yovariables/providers/integer_provider.h"
#include "ihmc/yovariables/variable/yo_integer.h"

namespace ihmc::yovariables::filters
{
class GlitchFilteredYoInteger : public variable::YoInteger
{
public:
   GlitchFilteredYoInteger(const std::string& name, int windowSize, registry::YoRegistry* registry);
   GlitchFilteredYoInteger(const std::string& name, int windowSize, providers::IntegerProvider* position, registry::YoRegistry* registry);
   GlitchFilteredYoInteger(const std::string& name, providers::IntegerProvider& windowSize, registry::YoRegistry* registry);
   GlitchFilteredYoInteger(const std::string& name, providers::IntegerProvider& windowSize, providers::IntegerProvider* position,
                            registry::YoRegistry* registry);

   bool set(int value);
   bool set(int value, bool notifyListeners);

   /** @throws std::runtime_error if constructed without a position variable. */
   void update();
   void update(int currentValue);

   int getWindowSize() const;

   /** @throws std::runtime_error if windowSize was given externally rather than constructed internally. */
   void setWindowSize(int windowSize);

private:
   providers::IntegerProvider* position_ = nullptr;
   variable::YoInteger previousPosition_;
   providers::IntegerProvider* windowSize_;
   variable::YoInteger counter_;
   std::optional<variable::YoInteger> internalWindowSize_;
};
}
