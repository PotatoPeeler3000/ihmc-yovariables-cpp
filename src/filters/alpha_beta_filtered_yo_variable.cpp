#include "ihmc/yovariables/filters/alpha_beta_filtered_yo_variable.h"

namespace ihmc::yovariables::filters
{
AlphaBetaFilteredYoVariable::AlphaBetaFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, double alpha, double beta,
                                                           variable::YoDouble& positionVariable, variable::YoDouble& xMeasuredVariable, double dt)
   : YoDouble(name, registry), alpha_(alpha), beta_(beta), dt_(dt), positionState_(positionVariable), xMeasuredVariable_(xMeasuredVariable)
{
   reset();
}

void AlphaBetaFilteredYoVariable::reset()
{
}

variable::YoDouble& AlphaBetaFilteredYoVariable::getPositionEstimation()
{
   return positionState_;
}

variable::YoDouble& AlphaBetaFilteredYoVariable::getVelocityEstimation()
{
   return *this;
}

void AlphaBetaFilteredYoVariable::update()
{
   update(positionState_.getDoubleValue());
}

void AlphaBetaFilteredYoVariable::update(double position)
{
   double velocity = getDoubleValue();

   double prediction = position + dt_ * velocity;
   double error = xMeasuredVariable_.getDoubleValue() - prediction;
   double newPosition = prediction + alpha_ * error;
   double newVelocity = velocity + beta_ * error;

   positionState_.set(newPosition);
   set(newVelocity);
}
}
