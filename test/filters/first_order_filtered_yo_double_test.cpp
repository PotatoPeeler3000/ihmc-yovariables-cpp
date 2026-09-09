#include <gtest/gtest.h>

#include <cmath>

#include "ihmc/yovariables/filters/first_order_band_pass_filtered_yo_double.h"
#include "ihmc/yovariables/filters/first_order_filtered_yo_double.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
constexpr double kDt = 0.001;

double computeProperLowPassAttenuation(double inputFreqRadPerSec, double cutoffFreqRadPerSec)
{
   return cutoffFreqRadPerSec / std::sqrt(inputFreqRadPerSec * inputFreqRadPerSec + cutoffFreqRadPerSec * cutoffFreqRadPerSec);
}

double computeProperHighPassAttenuation(double inputFreqRadPerSec, double cutoffFreqRadPerSec)
{
   return inputFreqRadPerSec / std::sqrt(inputFreqRadPerSec * inputFreqRadPerSec + cutoffFreqRadPerSec * cutoffFreqRadPerSec);
}

double computeProperBandPassAttenuation(double inputFreqRadPerSec, double minFreqRadPerSec, double maxFreqRadPerSec)
{
   double highPass = computeProperHighPassAttenuation(inputFreqRadPerSec, minFreqRadPerSec);
   double lowPass = computeProperLowPassAttenuation(inputFreqRadPerSec, maxFreqRadPerSec);
   return highPass * lowPass;
}

double computeSteadyStateFilteredOutputAmplitude(variable::YoDouble& yoTime, double dt, double inputFrequencyRadPerSec,
                                                  FirstOrderFilteredYoDouble& filteredYoVariable)
{
   double filterOutputOldest = 0.0;
   double filterOutputOld = 0.0;

   double filterOutputPeak = 0.0;
   double filterOutputPeakOld = 0.0;
   double filterOutputPeakPercentChange = 100.0;

   bool filterOutputHasReachedSteadyState = false;

   int i = 0;

   while (!filterOutputHasReachedSteadyState)
   {
      double t = i * dt;
      yoTime.set(t);

      double sineWaveInput = std::sin(inputFrequencyRadPerSec * t);

      filteredYoVariable.update(sineWaveInput);

      double filterOutput = filteredYoVariable.getDoubleValue();

      bool filterOutputJustHitAPeak = filterOutputOld > filterOutputOldest && filterOutputOld > filterOutput;

      if (filterOutputJustHitAPeak)
      {
         filterOutputPeak = filterOutputOld;
         filterOutputPeakPercentChange = 100.0 * std::abs((filterOutputPeak - filterOutputPeakOld) / filterOutputPeak);
         filterOutputPeakOld = filterOutputPeak;
      }

      filterOutputHasReachedSteadyState = filterOutputPeakPercentChange < 1e-6;

      filterOutputOldest = filterOutputOld;
      filterOutputOld = filterOutput;
      i++;
   }

   return filterOutputPeak;
}

double computeSteadyStateFilteredOutputAmplitude(variable::YoDouble& yoTime, double dt, double inputFrequencyRadPerSec,
                                                  FirstOrderBandPassFilteredYoDouble& filteredYoVariable)
{
   double filterOutputOldest = 0.0;
   double filterOutputOld = 0.0;

   double filterOutputPeak = 0.0;
   double filterOutputPeakOld = 0.0;
   double filterOutputPeakPercentChange = 100.0;

   bool filterOutputHasReachedSteadyState = false;

   int i = 0;

   while (!filterOutputHasReachedSteadyState)
   {
      double t = i * dt;
      yoTime.set(t);

      double sineWaveInput = std::sin(inputFrequencyRadPerSec * t);

      filteredYoVariable.update(sineWaveInput);

      double filterOutput = filteredYoVariable.getDoubleValue();

      bool filterOutputJustHitAPeak = filterOutputOld > filterOutputOldest && filterOutputOld > filterOutput;

      if (filterOutputJustHitAPeak)
      {
         filterOutputPeak = filterOutputOld;
         filterOutputPeakPercentChange = 100.0 * std::abs((filterOutputPeak - filterOutputPeakOld) / filterOutputPeak);
         filterOutputPeakOld = filterOutputPeak;
      }

      filterOutputHasReachedSteadyState = filterOutputPeakPercentChange < 1e-6;

      filterOutputOldest = filterOutputOld;
      filterOutputOld = filterOutput;
      i++;
   }

   return filterOutputPeak;
}

TEST(FirstOrderFilteredYoDoubleTest, testHighPassAttenuationForSinusoidalInput)
{
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble yoTime("yoTime", &registry);

   double inputFrequencyRadPerSec = 15.0;
   double cutoffFrequencyRadPerSec = inputFrequencyRadPerSec / 5.0;
   double filterAttenuation = 1.0;

   FirstOrderFilteredYoDouble highPassFilteredYoVariable("highPass", "", cutoffFrequencyRadPerSec / (2.0 * M_PI), &yoTime,
                                                           FirstOrderFilteredYoDouble::FirstOrderFilterType::HIGH_PASS, &registry);

   while (filterAttenuation > 0.1 && cutoffFrequencyRadPerSec > 0.0)
   {
      highPassFilteredYoVariable.setCutoffFrequencyHz(cutoffFrequencyRadPerSec / (2.0 * M_PI));
      highPassFilteredYoVariable.reset();
      filterAttenuation = computeSteadyStateFilteredOutputAmplitude(yoTime, kDt, inputFrequencyRadPerSec, highPassFilteredYoVariable);

      double properHighPassAttenuation = computeProperHighPassAttenuation(inputFrequencyRadPerSec, cutoffFrequencyRadPerSec);

      EXPECT_NEAR(properHighPassAttenuation, filterAttenuation, 1e-2);

      cutoffFrequencyRadPerSec += 10.0;
   }
}

TEST(FirstOrderFilteredYoDoubleTest, testLowPassAttenuationForSinusoidalInput)
{
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble yoTime("yoTime", &registry);

   double inputFrequencyRadPerSec = 10.0;
   double cutoffFrequencyRadPerSec = inputFrequencyRadPerSec * 5.0;
   double filterAttenuation = 1.0;

   FirstOrderFilteredYoDouble lowPassFilteredYoVariable("lowPass", "", cutoffFrequencyRadPerSec / (2.0 * M_PI), &yoTime,
                                                          FirstOrderFilteredYoDouble::FirstOrderFilterType::LOW_PASS, &registry);

   while (filterAttenuation > 0.1 && cutoffFrequencyRadPerSec > 0.0)
   {
      lowPassFilteredYoVariable.setCutoffFrequencyHz(cutoffFrequencyRadPerSec / (2.0 * M_PI));
      lowPassFilteredYoVariable.reset();
      filterAttenuation = computeSteadyStateFilteredOutputAmplitude(yoTime, kDt, inputFrequencyRadPerSec, lowPassFilteredYoVariable);

      double properLowPassAttenuation = computeProperLowPassAttenuation(inputFrequencyRadPerSec, cutoffFrequencyRadPerSec);

      EXPECT_NEAR(properLowPassAttenuation, filterAttenuation, 1e-2);

      cutoffFrequencyRadPerSec -= 10.0;
   }
}

TEST(FirstOrderFilteredYoDoubleTest, testBandPassAttenuationForSinusoidalInput)
{
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble yoTime("yoTime", &registry);

   double inputFrequencyRadPerSec = 10.0;

   double a = inputFrequencyRadPerSec / 5.0;
   double b = inputFrequencyRadPerSec * 5.0;

   double filterAttenuation = 1.0;

   FirstOrderBandPassFilteredYoDouble bandPassFilteredYoVariable("sineWave", "", a, b, yoTime, &registry);

   while (filterAttenuation > 0.1 && a > 0.0 && b > 0.0)
   {
      bandPassFilteredYoVariable.setPassBand(a / (2.0 * M_PI), b / (2.0 * M_PI));
      bandPassFilteredYoVariable.reset();
      filterAttenuation = computeSteadyStateFilteredOutputAmplitude(yoTime, kDt, inputFrequencyRadPerSec, bandPassFilteredYoVariable);

      double properBandPassAttenuation = computeProperBandPassAttenuation(inputFrequencyRadPerSec, a, b);

      EXPECT_NEAR(properBandPassAttenuation, filterAttenuation, 1e-2);

      a -= 10.0;
      b -= 10.0;
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
