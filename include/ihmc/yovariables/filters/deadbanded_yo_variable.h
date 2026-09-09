#pragma once

#include "ihmc/yovariables/filters/processing_yo_variable.h"
#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
class DeadbandedYoVariable : public variable::YoDouble, public ProcessingYoVariable
{
public:
   DeadbandedYoVariable(const std::string& name, providers::DoubleProvider& deadzoneSize, registry::YoRegistry* registry);
   DeadbandedYoVariable(const std::string& name, providers::DoubleProvider& inputVariable, providers::DoubleProvider& deadzoneSize,
                         registry::YoRegistry* registry);

   /** @throws std::logic_error if constructed without an input variable. */
   void update() override;
   void update(double valueToBeCorrected);

private:
   providers::DoubleProvider& deadzoneSize_;
   providers::DoubleProvider* inputVariable_ = nullptr;
};
}
