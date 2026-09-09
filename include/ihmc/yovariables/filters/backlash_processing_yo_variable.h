#pragma once

#include "ihmc/yovariables/filters/backlash_state.h"
#include "ihmc/yovariables/filters/processing_yo_variable.h"
#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"

namespace ihmc::yovariables::filters
{
/**
 * Estimates a velocity signal that may contain backlash by zeroing the estimate whenever the
 * input changes sign, then ramping it back up to the raw value over slopTime. See
 * BacklashCompensatingVelocityYoVariable, which does the same starting from a position signal.
 */
class BacklashProcessingYoVariable : public variable::YoDouble, public ProcessingYoVariable
{
public:
   BacklashProcessingYoVariable(const std::string& name, const std::string& description, double dt, providers::DoubleProvider& slopTime,
                                 registry::YoRegistry* registry, variable::YoDouble* velocityVariable = nullptr);

   void reset() override;

   /** @throws std::logic_error if constructed without a velocity variable. */
   void update() override;
   void update(double currentVelocity);

private:
   variable::YoDouble* velocity_ = nullptr;

   variable::YoBoolean hasBeenCalled_;

   variable::YoEnum<BacklashState> backlashState_;
   providers::DoubleProvider& slopTime_;

   variable::YoDouble timeSinceSloppy_;

   double dt_;
};
}
