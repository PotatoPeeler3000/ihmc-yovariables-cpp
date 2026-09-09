#pragma once

#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
/**
 * Filtered version of a YoDouble estimating the velocity of the value it tracks (alpha-beta / g-h
 * filter):
 * <pre>
 *   xp = x + dt*v            // position prediction
 *   x+ = xp + alpha*(xmeas - xp)  // adjusted position estimate
 *   v+ = v + beta*(xmeas - xp)    // adjusted velocity estimate
 * </pre>
 * This object's own value (inherited from YoDouble) is the velocity estimate v+.
 */
class AlphaBetaFilteredYoVariable : public variable::YoDouble
{
public:
   AlphaBetaFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, double alpha, double beta, variable::YoDouble& positionVariable,
                                variable::YoDouble& xMeasuredVariable, double dt);

   void reset();

   variable::YoDouble& getPositionEstimation();
   variable::YoDouble& getVelocityEstimation();

   void update();
   void update(double position);

private:
   double alpha_;
   double beta_;
   double dt_;

   variable::YoDouble& positionState_;
   variable::YoDouble& xMeasuredVariable_;
};
}
