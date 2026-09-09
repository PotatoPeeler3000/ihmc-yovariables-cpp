#include "ihmc/yovariables/filters/running_average_yo_double.h"

#include <stdexcept>

namespace ihmc::yovariables::filters
{
RunningAverageYoDouble::RunningAverageYoDouble(const std::string& name, registry::YoRegistry* registry)
   : RunningAverageYoDouble(name, nullptr, registry)
{
}

RunningAverageYoDouble::RunningAverageYoDouble(const std::string& name, providers::DoubleProvider* dataSource, registry::YoRegistry* registry)
   : YoDouble(name, registry), sampleSize_(name + "SampleSize", registry), dataSource_(dataSource)
{
}

void RunningAverageYoDouble::update()
{
   if (dataSource_ == nullptr)
      throw std::logic_error("RunningAverageYoDouble must be constructed with a non-null dataSource variable to call update().");
   update(dataSource_->getValue());
}

void RunningAverageYoDouble::update(double dataSource)
{
   sampleSize_.increment();
   add((dataSource - getValue()) / sampleSize_.getValue());
}

void RunningAverageYoDouble::reset()
{
   sampleSize_.set(0);
}

int RunningAverageYoDouble::getSampleSize() const
{
   return sampleSize_.getValue();
}
}
