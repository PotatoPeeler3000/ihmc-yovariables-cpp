#pragma once

#include "ihmc/yovariables/filters/backlash_state.h"
#include "ihmc/yovariables/filters/filtered_finite_difference_yo_variable.h"
#include "ihmc/yovariables/filters/processing_yo_variable.h"
#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"

namespace ihmc::yovariables::filters
{
/**
 * Computes the velocity of a position signal that may contain backlash, via
 * FilteredFiniteDifferenceYoVariable. When that velocity changes sign, a "slop" period begins,
 * during which this variable's output ramps from the previous velocity to the new measured value.
 */
class BacklashCompensatingVelocityYoVariable : public variable::YoDouble, public ProcessingYoVariable
{
public:
   BacklashCompensatingVelocityYoVariable(const std::string& name, const std::string& description, providers::DoubleProvider& alphaVariable, double dt,
                                           providers::DoubleProvider& slopTime, registry::YoRegistry* registry,
                                           providers::DoubleProvider* positionVariable = nullptr);

   void reset() override;

   /** @throws std::logic_error if constructed without a position variable. */
   void update() override;
   void update(double currentPosition);

private:
   void updateUsingDifference(double difference);

   double dt_;

   FilteredFiniteDifferenceYoVariable finiteDifferenceVelocity_;

   providers::DoubleProvider& alphaVariable_;
   providers::DoubleProvider* position_ = nullptr;

   variable::YoDouble lastPosition_;
   variable::YoBoolean hasBeenCalled_;

   variable::YoEnum<BacklashState> backlashState_;
   providers::DoubleProvider& slopTime_;

   variable::YoDouble timeSinceSloppy_;
};
}
