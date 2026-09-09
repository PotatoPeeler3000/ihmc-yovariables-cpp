#pragma once

#include "ihmc/yovariables/filters/first_order_filtered_yo_double.h"

namespace ihmc::yovariables::filters
{
class FirstOrderBandPassFilteredYoDouble : public FirstOrderFilteredYoDouble
{
public:
   FirstOrderBandPassFilteredYoDouble(const std::string& name, const std::string& description, double minPassThroughFrequencyHz,
                                       double maxPassThroughFrequencyHz, providers::DoubleProvider& yoTime, registry::YoRegistry* registry);
   FirstOrderBandPassFilteredYoDouble(const std::string& name, const std::string& description, double minPassThroughFrequencyHz,
                                       double maxPassThroughFrequencyHz, double dt, registry::YoRegistry* registry);

   void update(double filterInput);
   void setPassBand(double minPassThroughFreqHz, double maxPassThroughFreqHz);

private:
   /** @throws std::runtime_error if minPassThroughFrequencyHz > maxPassThroughFrequencyHz. */
   static void checkPassband(double minPassThroughFrequencyHz, double maxPassThroughFrequencyHz);
   void updateHighPassFilterAndThenLowPassFilterThat(double filterInput);

   bool hasBeenCalled_ = false;
   FirstOrderFilteredYoDouble highPassFilteredInput_;
};
}
