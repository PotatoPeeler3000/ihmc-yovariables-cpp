#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ihmc/yovariables/filters/alpha_filter_tools.h"
#include "ihmc/yovariables/filters/alpha_filtered_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
// Java's rng here has no fixed seed (relies only on statistical/invariant properties, not exact
// values); a fixed seed is used here for reproducibility, which doesn't change what's being tested.
TEST(AlphaFilteredYoVariableTest, testNoisyFixedPosition)
{
   std::mt19937 rng(1U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   // Use a reasonably large alpha for a reasonably large amount of noise
   double alpha = 0.8;

   registry::YoRegistry registry("testRegistry");
   variable::YoDouble positionVariable("positionVariable", &registry);
   AlphaFilteredYoVariable alphaFilteredYoVariable("alphaFilteredYoVariable", &registry, alpha, &positionVariable);

   double pseudoNoise = 0;

   positionVariable.set(10);
   for (int i = 0; i < 10000; i++)
   {
      // Oscillate the position about some uniformly distributed fixed point slightly larger than 10
      if (i % 2 == 0)
         pseudoNoise = dist(rng);
      positionVariable.add(std::pow(-1, i) * pseudoNoise);
      alphaFilteredYoVariable.update();
   }

   EXPECT_NEAR(10, alphaFilteredYoVariable.getDoubleValue(), 1);
}

TEST(AlphaFilteredYoVariableTest, testAlphaAndBreakFrequencyComputations)
{
   std::mt19937 rng(2U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   for (int i = 0; i < 1000; i++)
   {
      double dt = dist(rng);

      double expectedAlpha = dist(rng);
      double breakFrequency = computeBreakFrequencyGivenAlpha(expectedAlpha, dt);
      double actualAlpha = computeAlphaGivenBreakFrequencyProperly(breakFrequency, dt);

      EXPECT_NEAR(expectedAlpha, actualAlpha, 1e-10);

      double maxFrequency = 0.5 * 0.5 / dt;
      double expectedBreakFrequency = maxFrequency * dist(rng);
      double alpha = computeAlphaGivenBreakFrequencyProperly(expectedBreakFrequency, dt);
      double actualBreakFrequency = computeBreakFrequencyGivenAlpha(alpha, dt);

      EXPECT_NEAR(expectedBreakFrequency, actualBreakFrequency, 1e-7);
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
