#pragma once

#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
class FirstOrderFilteredYoDouble : public variable::YoDouble
{
public:
   enum class FirstOrderFilterType
   {
      LOW_PASS,
      HIGH_PASS
   };

   /** @throws std::invalid_argument if highOrLowPass isn't a recognized value (defensive; enum class makes this unreachable in practice). */
   FirstOrderFilteredYoDouble(const std::string& name, const std::string& description, double cutoffFrequencyHz, providers::DoubleProvider* yoTime,
                               FirstOrderFilterType highOrLowPass, registry::YoRegistry* registry);
   FirstOrderFilteredYoDouble(const std::string& name, const std::string& description, double cutoffFrequencyHz, double dt,
                               FirstOrderFilterType highOrLowPass, registry::YoRegistry* registry);

   void reset();
   void setCutoffFrequencyHz(double cutoffHz);

   /** @throws std::runtime_error if the cutoff frequency is not greater than zero, or if the resulting alpha isn't in (0, 1). */
   void update(double filterInput);

private:
   double computeLowPassUpdate(double filterInput, double dt);
   double computeHighPassUpdate(double filterInput, double dt);
   static double computeAlpha(double dt, double cutoffFrequencyHz);

   variable::YoDouble cutoffFrequencyHz_;

   bool hasBeenCalled_ = false;

   double filterInputOld_ = 0.0;
   double filterUpdateTimeOld_ = 0.0;

   providers::DoubleProvider* yoTime_ = nullptr;
   double dt_ = 0.0;

   FirstOrderFilterType filterType_;
};
}
