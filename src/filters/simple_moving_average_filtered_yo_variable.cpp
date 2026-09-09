#include "ihmc/yovariables/filters/simple_moving_average_filtered_yo_variable.h"

namespace ihmc::yovariables::filters
{
SimpleMovingAverageFilteredYoVariable::SimpleMovingAverageFilteredYoVariable(const std::string& name, int windowSize, registry::YoRegistry* registry)
   : SimpleMovingAverageFilteredYoVariable(name, windowSize, nullptr, registry)
{
}

SimpleMovingAverageFilteredYoVariable::SimpleMovingAverageFilteredYoVariable(const std::string& name, int windowSize,
                                                                              variable::YoDouble* yoVariableToFilter, registry::YoRegistry* registry)
   : YoDouble(name, registry), windowSize_(name + "WindowSize", registry), yoVariableToFilter_(yoVariableToFilter),
     previousUpdateValues_(static_cast<std::size_t>(windowSize), 0.0)
{
   windowSize_.set(windowSize);
}

void SimpleMovingAverageFilteredYoVariable::setWindowSize(int windowSize)
{
   windowSize_.set(windowSize);
   reset();
}

void SimpleMovingAverageFilteredYoVariable::update()
{
   update(yoVariableToFilter_->getDoubleValue());
}

void SimpleMovingAverageFilteredYoVariable::update(double value)
{
   if (static_cast<int>(previousUpdateValues_.size()) != windowSize_.getIntegerValue())
      reset();

   previousUpdateValues_[static_cast<std::size_t>(bufferPosition_)] = value;

   bufferPosition_++;

   if (bufferPosition_ >= windowSize_.getIntegerValue())
   {
      bufferPosition_ = 0;
      bufferHasBeenFilled_ = true;
   }

   double average = 0.0;
   for (int i = 0; i < windowSize_.getIntegerValue(); i++)
      average += previousUpdateValues_[static_cast<std::size_t>(i)];

   int size = bufferHasBeenFilled_ ? windowSize_.getIntegerValue() : bufferPosition_;
   set(average / static_cast<double>(size));
}

void SimpleMovingAverageFilteredYoVariable::reset()
{
   bufferPosition_ = 0;
   bufferHasBeenFilled_ = false;
   previousUpdateValues_.assign(static_cast<std::size_t>(windowSize_.getIntegerValue()), 0.0);
}

bool SimpleMovingAverageFilteredYoVariable::getHasBufferWindowFilled() const
{
   return bufferHasBeenFilled_;
}
}
