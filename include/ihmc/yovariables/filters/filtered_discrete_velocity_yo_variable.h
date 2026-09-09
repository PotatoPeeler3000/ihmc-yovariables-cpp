#pragma once

#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"

namespace ihmc::yovariables::filters
{
enum class FilteredDiscreteVelocityDirection
{
   NONE,
   FORWARD,
   BACKWARD
};

/**
 * A filtered velocity of a position: finite-differences the position, then alpha-filters the
 * result. vel_n = alpha*vel_{n-1} + (1-alpha)*(pos_n - pos_{n-1}).
 */
class FilteredDiscreteVelocityYoVariable : public variable::YoDouble
{
public:
   FilteredDiscreteVelocityYoVariable(const std::string& name, const std::string& description, providers::DoubleProvider& alphaVariable,
                                       variable::YoDouble& positionVariable, variable::YoDouble& time, registry::YoRegistry* registry);

   void reset();
   void update();
   void update(double currentPosition);

   double getUnfilteredVelocity() const;

private:
   variable::YoDouble& time_;

   providers::DoubleProvider& alphaVariable_;
   variable::YoDouble& position_;

   variable::YoDouble lastUpdateTime_;
   variable::YoEnum<FilteredDiscreteVelocityDirection> lastUpdateDirection_;
   variable::YoDouble unfilteredVelocity_;

   variable::YoDouble lastPosition_;
   bool hasBeenCalled_ = false;
};
}
