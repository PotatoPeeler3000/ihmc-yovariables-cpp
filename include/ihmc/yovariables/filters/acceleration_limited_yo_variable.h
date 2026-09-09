#pragma once

#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
class AccelerationLimitedYoVariable : public variable::YoDouble
{
public:
   AccelerationLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider* maxRate,
                                  providers::DoubleProvider* maxAcceleration, double dt);
   AccelerationLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider* maxRate,
                                  providers::DoubleProvider* maxAcceleration, providers::DoubleProvider* inputVariable, double dt);

   void setGainsByPolePlacement(double w0, double zeta);

   variable::YoDouble& getPositionGain();
   variable::YoDouble& getVelocityGain();

   /** @throws std::logic_error if constructed without an input variable. */
   void update();
   void update(double input);
   void initialize(double input);
   void reset();

   variable::YoDouble& getSmoothedRate();
   variable::YoDouble& getSmoothedAcceleration();

   bool hasBeenInitialized() const;
   double getMaximumRate() const;
   double getMaximumAcceleration() const;

private:
   double dt_;

   variable::YoBoolean hasBeenInitialized_;

   variable::YoDouble smoothedRate_;
   variable::YoDouble smoothedAcceleration_;

   variable::YoDouble positionGain_;
   variable::YoDouble velocityGain_;

   // Preserves a latent quirk from the Java source: if only one of maxRate/maxAcceleration is
   // given, NEITHER is set (both stay null/nullptr), not just the missing one.
   providers::DoubleProvider* maximumRate_ = nullptr;
   providers::DoubleProvider* maximumAcceleration_ = nullptr;

   providers::DoubleProvider* inputVariable_ = nullptr;
};
}
