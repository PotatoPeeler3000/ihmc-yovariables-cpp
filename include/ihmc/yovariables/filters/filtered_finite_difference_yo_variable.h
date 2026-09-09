#pragma once

#include <optional>

#include "ihmc/yovariables/filters/filter_math.h"
#include "ihmc/yovariables/filters/processing_yo_variable.h"
#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
/**
 * Computes the velocity of a position signal via finite differencing, then alpha-filters it:
 * vel_n = alpha*vel_{n-1} + (1-alpha)*(pos_n - pos_{n-1}).
 */
class FilteredFiniteDifferenceYoVariable : public variable::YoDouble, public ProcessingYoVariable
{
public:
   FilteredFiniteDifferenceYoVariable(const std::string& name, const std::string& description, double alpha, double dt, registry::YoRegistry* registry,
                                       variable::YoDouble* positionVariable = nullptr);
   FilteredFiniteDifferenceYoVariable(const std::string& name, const std::string& description, providers::DoubleProvider& alphaVariable, double dt,
                                       registry::YoRegistry* registry, variable::YoDouble* positionVariable = nullptr);
   FilteredFiniteDifferenceYoVariable(const std::string& name, const std::string& description, providers::DoubleProvider& alphaVariable,
                                       providers::DoubleProvider& dt, registry::YoRegistry* registry, variable::YoDouble* positionVariable = nullptr);

   void reset() override;

   /** @throws std::logic_error if constructed without a position variable. */
   void update() override;
   void updateForAngles();

   void update(double currentPosition);
   void updateForAngles(double currentPosition);

private:
   void updateUsingDifference(double difference);

   providers::DoubleProvider* alphaVariable_ = nullptr;
   providers::DoubleProvider* dt_ = nullptr;
   variable::YoDouble* position_ = nullptr;

   variable::YoDouble lastPosition_;
   variable::YoBoolean hasBeenCalled_;

   std::optional<variable::YoDouble> internalAlpha_;
   std::optional<ConstantDoubleProvider> internalDt_;
};
}
