#pragma once

#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
class JerkLimitedYoDouble : public variable::YoDouble
{
public:
   JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration, variable::YoDouble& maxJerk, double dt);
   JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration, variable::YoDouble& maxJerk,
                        variable::YoDouble* inputPosition, double dt);
   JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration, variable::YoDouble& maxJerk,
                        variable::YoDouble* inputPosition, variable::YoDouble* inputVelocity, double dt);
   JerkLimitedYoDouble(const std::string& name, registry::YoRegistry* registry, variable::YoDouble& maxAcceleration, variable::YoDouble& maxJerk,
                        variable::YoDouble* inputPosition, variable::YoDouble* inputVelocity, variable::YoDouble* inputAcceleration, double dt);

   void setMaximumAcceleration(double maximumAcceleration);
   void setMaximumJerk(double maximumJerk);
   void setGainsByPolePlacement(double w0, double w1, double zeta);

   void update();
   void update(double inputPosition);
   void update(double inputPosition, double inputVelocity);
   void update(double inputPosition, double inputVelocity, double inputAcceleration);

   void initialize(double inputPosition, double inputVelocity, double inputAcceleration);
   void reset();

private:
   double dt_;

   variable::YoBoolean hasBeenInitialized_;

   variable::YoDouble smoothedRate_;
   variable::YoDouble smoothedAcceleration_;
   variable::YoDouble smoothedJerk_;

   variable::YoDouble positionGain_;
   variable::YoDouble velocityGain_;
   variable::YoDouble accelerationGain_;

   variable::YoDouble& maximumJerk_;
   variable::YoDouble& maximumAcceleration_;

   variable::YoDouble* inputPosition_ = nullptr;
   variable::YoDouble* inputVelocity_ = nullptr;
   variable::YoDouble* inputAcceleration_ = nullptr;
};
}
