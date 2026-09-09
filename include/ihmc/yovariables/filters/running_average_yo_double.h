#pragma once

#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_integer.h"

namespace ihmc::yovariables::filters
{
class RunningAverageYoDouble : public variable::YoDouble
{
public:
   explicit RunningAverageYoDouble(const std::string& name, registry::YoRegistry* registry);
   RunningAverageYoDouble(const std::string& name, providers::DoubleProvider* dataSource, registry::YoRegistry* registry);

   /** @throws std::logic_error if constructed without a data source. */
   void update();
   void update(double dataSource);
   void reset();

   int getSampleSize() const;

private:
   variable::YoInteger sampleSize_;
   providers::DoubleProvider* dataSource_ = nullptr;
};
}
