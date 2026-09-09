#include "ihmc/yovariables/filters/glitch_filtered_yo_integer.h"

#include <stdexcept>

namespace ihmc::yovariables::filters
{
GlitchFilteredYoInteger::GlitchFilteredYoInteger(const std::string& name, int windowSize, registry::YoRegistry* registry)
   : GlitchFilteredYoInteger(name, windowSize, nullptr, registry)
{
}

GlitchFilteredYoInteger::GlitchFilteredYoInteger(const std::string& name, int windowSize, providers::IntegerProvider* position,
                                                   registry::YoRegistry* registry)
   : YoInteger(name, "GlitchFilteredYoInteger", registry), position_(position), previousPosition_(name + "PrevValue", registry),
     counter_(name + "Count", registry)
{
   internalWindowSize_.emplace(name + "WindowSize", registry);
   internalWindowSize_->set(windowSize);
   windowSize_ = &*internalWindowSize_;
}

GlitchFilteredYoInteger::GlitchFilteredYoInteger(const std::string& name, providers::IntegerProvider& windowSize, registry::YoRegistry* registry)
   : GlitchFilteredYoInteger(name, windowSize, nullptr, registry)
{
}

GlitchFilteredYoInteger::GlitchFilteredYoInteger(const std::string& name, providers::IntegerProvider& windowSize, providers::IntegerProvider* position,
                                                   registry::YoRegistry* registry)
   : YoInteger(name, "GlitchFilteredYoInteger", registry), position_(position), previousPosition_(name + "PrevValue", registry),
     windowSize_(&windowSize), counter_(name + "Count", registry)
{
}

bool GlitchFilteredYoInteger::set(int value)
{
   counter_.set(0);
   return YoInteger::set(value);
}

bool GlitchFilteredYoInteger::set(int value, bool notifyListeners)
{
   counter_.set(0);
   return YoInteger::set(value, notifyListeners);
}

void GlitchFilteredYoInteger::update()
{
   if (position_ == nullptr)
      throw std::runtime_error("GlitchFilteredYoInteger must be constructed with a non-null position variable to call update(); use update(int) instead.");
   update(position_->getValue());
}

void GlitchFilteredYoInteger::update(int currentValue)
{
   if (currentValue == previousPosition_.getIntegerValue())
      counter_.increment();
   else
      counter_.set(0);

   if (counter_.getIntegerValue() >= windowSize_->getValue())
   {
      set(currentValue);
      counter_.set(0);
   }

   previousPosition_.set(currentValue);
}

int GlitchFilteredYoInteger::getWindowSize() const
{
   return windowSize_->getValue();
}

void GlitchFilteredYoInteger::setWindowSize(int windowSize)
{
   auto* asYoInteger = dynamic_cast<variable::YoInteger*>(windowSize_);
   if (asYoInteger != nullptr)
      asYoInteger->set(windowSize);
   else
      throw std::runtime_error("Setting the window size is not supported");
}
}
