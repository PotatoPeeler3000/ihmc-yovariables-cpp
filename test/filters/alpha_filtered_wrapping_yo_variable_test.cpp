#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <utility>

#include "ihmc/yovariables/filters/alpha_filter_tools.h"
#include "ihmc/yovariables/filters/alpha_filtered_wrapping_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
double getErrorConsideringWrap(double current, double target, double lowerLimit, double upperLimit)
{
   double range = upperLimit - lowerLimit;
   if (target > upperLimit)
      target = std::fmod(target - lowerLimit, range) + lowerLimit;

   if (target < lowerLimit)
   {
      double offset = std::fmod(target - upperLimit, range);
      target = offset + upperLimit;
   }

   double standardError = target - current;
   double wrappingError = 0.0;
   if (target > current)
      wrappingError = lowerLimit - current + target - upperLimit;
   else
      wrappingError = upperLimit - current + target - lowerLimit;

   if (std::abs(standardError) < std::abs(wrappingError))
      return standardError;
   return wrappingError;
}

TEST(AlphaFilteredWrappingYoVariableTest, testInputModulo)
{
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble alpha("alpha", &registry);
   alpha.set(0.0); // instantaneous correction so the result is just the input with modulo applied

   variable::YoDouble positionVariable("positionVariable", &registry);
   AlphaFilteredWrappingYoVariable alphaFilteredWrappingYoVariable("alphaFilteredWrappingYoVariable", "", &registry, positionVariable, alpha, -2.0, 8.0);

   // test at the boundaries
   positionVariable.set(8.0);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10);

   positionVariable.set(-2.0);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10);

   // test when the input is over the upperLimit
   positionVariable.set(33.0);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), 3.0, 1e-10);

   positionVariable.set(38.0);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10);

   positionVariable.set(42.0);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), 2.0, 1e-10);

   // test when the input is under the lowerLimit
   positionVariable.set(-22.0);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10);

   positionVariable.set(-23.5);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), 6.5, 1e-10);

   positionVariable.set(-42.0);
   alphaFilteredWrappingYoVariable.update();
   EXPECT_NEAR(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10);
}

// Java's rng here has no fixed seed; a fixed seed is used for reproducibility (see
// AlphaFilteredYoVariableTest's equivalent comment).
TEST(AlphaFilteredWrappingYoVariableTest, testNoisyFixedPosition)
{
   std::mt19937 random(3U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   registry::YoRegistry registry("testRegistry");
   variable::YoDouble alpha("alpha", &registry);
   alpha.set(0.8);

   variable::YoDouble positionVariable("positionVariable", &registry);
   AlphaFilteredWrappingYoVariable alphaFilteredWrappingYoVariable("alphaFilteredWrappingYoVariable", "", &registry, positionVariable, alpha, 0.0, 20.0);

   double pseudoNoise = 0;

   positionVariable.set(10.0);
   for (int i = 0; i < 10000; i++)
   {
      if (i % 2 == 0)
         pseudoNoise = dist(random);
      positionVariable.add(std::pow(-1, i) * pseudoNoise);
      alphaFilteredWrappingYoVariable.update();
   }

   EXPECT_NEAR(10.0, alphaFilteredWrappingYoVariable.getDoubleValue(), 1.0);
}

TEST(AlphaFilteredWrappingYoVariableTest, testErrorAlwaysDecreases)
{
   std::mt19937 random(4U);

   registry::YoRegistry registry("testRegistry");
   variable::YoDouble alpha("alpha", &registry);
   alpha.set(0.999999);

   variable::YoDouble positionVariable("positionVariable", &registry);
   std::uniform_real_distribution<double> limitDist(-100.0, 100.0);
   double lowerLimit = limitDist(random);
   double upperLimit = limitDist(random);
   if (upperLimit < lowerLimit)
      std::swap(lowerLimit, upperLimit);

   AlphaFilteredWrappingYoVariable alphaFilteredWrappingYoVariable("alphaFilteredWrappingYoVariable", "", &registry, positionVariable, alpha, lowerLimit,
                                                                     upperLimit);
   std::uniform_real_distribution<double> rangeDist(lowerLimit, upperLimit);
   positionVariable.set(rangeDist(random));
   alphaFilteredWrappingYoVariable.update();

   for (int iteration = 0; iteration < 10000; iteration++)
   {
      positionVariable.set(rangeDist(random));
      double lastError = getErrorConsideringWrap(alphaFilteredWrappingYoVariable.getDoubleValue(), positionVariable.getDoubleValue(), lowerLimit, upperLimit);
      for (int convergeAlphaCount = 0; convergeAlphaCount < 100; convergeAlphaCount++)
      {
         alphaFilteredWrappingYoVariable.update();
         double currentError =
            getErrorConsideringWrap(alphaFilteredWrappingYoVariable.getDoubleValue(), positionVariable.getDoubleValue(), lowerLimit, upperLimit);
         EXPECT_LT(std::abs(currentError), std::abs(lastError));
      }
   }
}

TEST(AlphaFilteredWrappingYoVariableTest, testWrappingError)
{
   double e = getErrorConsideringWrap(0.2, 0.8, 0.0, 1.0);
   EXPECT_NEAR(-0.4, e, 0.001);

   e = getErrorConsideringWrap(0.8, 0.2, 0.0, 1.0);
   EXPECT_NEAR(0.4, e, 0.001);

   e = getErrorConsideringWrap(0.0, 0.4, 0.0, 1.0);
   EXPECT_NEAR(0.4, e, 0.001);

   e = getErrorConsideringWrap(-0.2, 0.4, -1.0, 1.0);
   EXPECT_NEAR(0.6, e, 0.001);

   e = getErrorConsideringWrap(-1.0, 1.0, -1.0, 1.0);
   EXPECT_NEAR(0.0, e, 0.001);

   e = getErrorConsideringWrap(1.0, -1.0, -1.0, 1.0);
   EXPECT_NEAR(0.0, e, 0.001);

   e = getErrorConsideringWrap(0.4, 1.6, -1.0, 1.0);
   EXPECT_NEAR(-0.8, e, 0.001);

   e = getErrorConsideringWrap(-0.4, -1.6, -1.0, 1.0);
   EXPECT_NEAR(0.8, e, 0.001);

   e = getErrorConsideringWrap(0.4, -1.6, -1.0, 1.0);
   EXPECT_NEAR(0.0, e, 0.001);

   e = getErrorConsideringWrap(0.2, 0.2, -1.0, 1.0);
   EXPECT_NEAR(0.0, e, 0.001);

   e = getErrorConsideringWrap(-3.2, -4.0, -5.0, -1.0);
   EXPECT_NEAR(-0.8, e, 0.001);

   getErrorConsideringWrap(0.0, 0.0, -1.0, 1.0);
}

TEST(AlphaFilteredWrappingYoVariableTest, testAlphaAndBreakFrequencyComputations)
{
   std::mt19937 random(5U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   double dt = 0.1;
   double randomAlpha = dist(random);
   double computedBreakFrequency = computeBreakFrequencyGivenAlpha(randomAlpha, dt);
   double computedAlpha = computeAlphaGivenBreakFrequencyProperly(computedBreakFrequency, dt);

   EXPECT_NEAR(randomAlpha, computedAlpha, 1e-7);
   EXPECT_NEAR(computedBreakFrequency, computeBreakFrequencyGivenAlpha(computedAlpha, dt), 1e-7);
}
} // namespace
} // namespace ihmc::yovariables::filters
