#include "ihmc/yovariables/filters/first_order_band_pass_filtered_yo_double.h"

#include <stdexcept>

namespace ihmc::yovariables::filters
{
FirstOrderBandPassFilteredYoDouble::FirstOrderBandPassFilteredYoDouble(const std::string& name, const std::string& description,
                                                                         double minPassThroughFrequencyHz, double maxPassThroughFrequencyHz,
                                                                         providers::DoubleProvider& yoTime, registry::YoRegistry* registry)
   : FirstOrderFilteredYoDouble(name, description, maxPassThroughFrequencyHz, &yoTime, FirstOrderFilterType::LOW_PASS, registry),
     highPassFilteredInput_(name + "HighPassFilteredOnly", description, minPassThroughFrequencyHz, &yoTime, FirstOrderFilterType::HIGH_PASS, registry)
{
   setPassBand(minPassThroughFrequencyHz, maxPassThroughFrequencyHz);
}

FirstOrderBandPassFilteredYoDouble::FirstOrderBandPassFilteredYoDouble(const std::string& name, const std::string& description,
                                                                         double minPassThroughFrequencyHz, double maxPassThroughFrequencyHz, double dt,
                                                                         registry::YoRegistry* registry)
   : FirstOrderFilteredYoDouble(name, description, maxPassThroughFrequencyHz, dt, FirstOrderFilterType::LOW_PASS, registry),
     highPassFilteredInput_(name + "HighPassFilteredOnly", description, minPassThroughFrequencyHz, dt, FirstOrderFilterType::HIGH_PASS, registry)
{
}

void FirstOrderBandPassFilteredYoDouble::checkPassband(double minPassThroughFrequencyHz, double maxPassThroughFrequencyHz)
{
   if (minPassThroughFrequencyHz > maxPassThroughFrequencyHz)
      throw std::runtime_error("minPassThroughFrequency [ " + std::to_string(minPassThroughFrequencyHz) + " ] > maxPassThroughFrequency [ "
                                + std::to_string(maxPassThroughFrequencyHz) + " ]");
}

void FirstOrderBandPassFilteredYoDouble::update(double filterInput)
{
   if (!hasBeenCalled_)
   {
      hasBeenCalled_ = true;
      set(filterInput);
   }
   else
   {
      updateHighPassFilterAndThenLowPassFilterThat(filterInput);
   }
}

void FirstOrderBandPassFilteredYoDouble::setPassBand(double minPassThroughFreqHz, double maxPassThroughFreqHz)
{
   checkPassband(minPassThroughFreqHz, maxPassThroughFreqHz);

   highPassFilteredInput_.setCutoffFrequencyHz(minPassThroughFreqHz);
   setCutoffFrequencyHz(maxPassThroughFreqHz);
}

void FirstOrderBandPassFilteredYoDouble::updateHighPassFilterAndThenLowPassFilterThat(double filterInput)
{
   highPassFilteredInput_.update(filterInput);
   FirstOrderFilteredYoDouble::update(highPassFilteredInput_.getDoubleValue());
}
}
