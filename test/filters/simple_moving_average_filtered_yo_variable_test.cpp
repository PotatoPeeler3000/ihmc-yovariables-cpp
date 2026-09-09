#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "ihmc/yovariables/filters/simple_moving_average_filtered_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
TEST(SimpleMovingAverageFilteredYoVariableTest, testWithFixedSizeDoubleArrays)
{
   for (int i = 0; i < 100; i++)
   {
      registry::YoRegistry registry("Blop");
      std::mt19937 random(6541654U);
      std::uniform_int_distribution<int> windowSizeDist(1, 999);
      int windowSize = windowSizeDist(random);
      SimpleMovingAverageFilteredYoVariable sma("tested", windowSize, &registry);
      double amplitude = 100.0;
      std::uniform_real_distribution<double> valueDist(-amplitude, amplitude);
      std::vector<double> randomArray(windowSize);
      for (double& value : randomArray)
         value = valueDist(random);

      double expected = 0.0;
      for (double val : randomArray)
         expected += val / windowSize;

      for (double val : randomArray)
      {
         EXPECT_FALSE(sma.getHasBufferWindowFilled());
         sma.update(val);
      }

      EXPECT_TRUE(sma.getHasBufferWindowFilled());
      EXPECT_NEAR(expected, sma.getDoubleValue(), 1.0e-10);
   }
}

TEST(SimpleMovingAverageFilteredYoVariableTest, testBetaFilteredYoVariable)
{
   int beta = 5000;
   double pseudoNoise = 0;

   std::mt19937 random(1738U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble positionVariable("positionVariable", &registry);
   SimpleMovingAverageFilteredYoVariable betaFilteredYoVariable("betaFilteredYoVariable", beta, &positionVariable, &registry);

   positionVariable.set(10);

   for (int i = 0; i < 10000; i++)
   {
      if (i % 2 == 0)
         pseudoNoise = dist(random);
      positionVariable.add(std::pow(-1, i) * pseudoNoise);
      betaFilteredYoVariable.update();
   }

   EXPECT_NEAR(10, betaFilteredYoVariable.getDoubleValue(), 1);
}

TEST(SimpleMovingAverageFilteredYoVariableTest, testTrueMovingAverage)
{
   int beta = 10;

   registry::YoRegistry registry("testRegistry");
   SimpleMovingAverageFilteredYoVariable betaFilteredYoVariable("betaFilteredYoVariable", beta, &registry);

   double epsilon = 1e-10;

   betaFilteredYoVariable.update(1.0);
   EXPECT_NEAR(1.0, betaFilteredYoVariable.getDoubleValue(), epsilon);

   betaFilteredYoVariable.update(2.0);
   EXPECT_NEAR(1.5, betaFilteredYoVariable.getDoubleValue(), epsilon);

   betaFilteredYoVariable.update(3.0);
   betaFilteredYoVariable.update(4.0);
   betaFilteredYoVariable.update(5.0);
   betaFilteredYoVariable.update(6.0);
   betaFilteredYoVariable.update(7.0);
   betaFilteredYoVariable.update(8.0);
   betaFilteredYoVariable.update(9.0);
   betaFilteredYoVariable.update(10.0);

   EXPECT_NEAR(5.5, betaFilteredYoVariable.getDoubleValue(), epsilon);
}
} // namespace
} // namespace ihmc::yovariables::filters
