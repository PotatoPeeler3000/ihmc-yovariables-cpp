#include "ihmc/yovariables/filters/glitch_filtered_yo_boolean.h"

#include <stdexcept>

namespace ihmc::yovariables::filters
{
GlitchFilteredYoBoolean::GlitchFilteredYoBoolean(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                                                   variable::YoBoolean* yoVariableToFilter, variable::YoInteger& windowSize)
   : YoBoolean(name, description, registry), variableToFilter_(yoVariableToFilter), windowSize_(&windowSize), counter_(name + "Count", description, registry)
{
   if (windowSize.getIntegerValue() < 0)
      throw std::runtime_error("window size must be greater than 0");

   if (variableToFilter_ != nullptr)
      set(variableToFilter_->getBooleanValue());

   set(false);
}

GlitchFilteredYoBoolean::GlitchFilteredYoBoolean(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                                                   variable::YoBoolean* yoVariableToFilter, int windowSize)
   : YoBoolean(name, description, registry), variableToFilter_(yoVariableToFilter), counter_(name + "Count", description, registry)
{
   internalWindowSize_.emplace(name + "WindowSize", registry);
   internalWindowSize_->set(windowSize);
   windowSize_ = &*internalWindowSize_;

   if (windowSize_->getIntegerValue() < 0)
      throw std::runtime_error("window size must be greater than 0");

   if (variableToFilter_ != nullptr)
      set(variableToFilter_->getBooleanValue());

   set(false);
}

GlitchFilteredYoBoolean::GlitchFilteredYoBoolean(const std::string& name, registry::YoRegistry* registry, int windowSize)
   : GlitchFilteredYoBoolean(name, "", registry, nullptr, windowSize)
{
}

bool GlitchFilteredYoBoolean::set(bool value)
{
   counter_.set(0);
   return YoBoolean::set(value);
}

void GlitchFilteredYoBoolean::update()
{
   if (variableToFilter_ == nullptr)
      throw std::runtime_error("variableToFilter was not initialized. Use the other constructor.");
   update(variableToFilter_->getBooleanValue());
}

void GlitchFilteredYoBoolean::update(bool value)
{
   if (value != getBooleanValue())
      counter_.set(counter_.getIntegerValue() + 1);
   else
      counter_.set(0);

   if (counter_.getIntegerValue() >= windowSize_->getIntegerValue())
      set(value);
}

int GlitchFilteredYoBoolean::getWindowSize() const
{
   return windowSize_->getIntegerValue();
}

void GlitchFilteredYoBoolean::setWindowSize(int windowSize)
{
   windowSize_->set(windowSize);
}
}
