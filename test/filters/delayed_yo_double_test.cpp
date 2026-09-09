#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "ihmc/yovariables/filters/delayed_yo_double.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
TEST(DelayedYoDoubleTest, testDelayedYoVariableMultipleTickDelays)
{
   std::mt19937 rng(1U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   registry::YoRegistry registry("registry");
   variable::YoDouble variableToDelay("variableToDelay", &registry);

   for (int ticksToDelay = 0; ticksToDelay < 10; ticksToDelay++)
   {
      double firstValue = dist(rng);
      variableToDelay.set(firstValue);

      DelayedYoDouble delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", variableToDelay, ticksToDelay, &registry);

      int ticksToTest = 100;
      std::vector<double> valuesToSet(ticksToTest);

      for (int i = 0; i < ticksToTest; i++)
         valuesToSet[i] = dist(rng);

      EXPECT_NEAR(delayedYoVariable.getDoubleValue(), firstValue, 1e-7);

      for (int i = 0; i < ticksToTest; i++)
      {
         variableToDelay.set(valuesToSet[i]);
         delayedYoVariable.update();

         if (i < ticksToDelay)
            EXPECT_NEAR(delayedYoVariable.getDoubleValue(), firstValue, 1e-7);
         else
            EXPECT_NEAR(delayedYoVariable.getDoubleValue(), valuesToSet[i - ticksToDelay], 1e-7);
      }
   }
}

TEST(DelayedYoDoubleTest, testDelayedYoVariableOneTickDelay)
{
   registry::YoRegistry registry("registry");
   variable::YoDouble variableToDelay("variableToDelay", &registry);

   int ticksToDelay = 1;

   variableToDelay.set(0.0);
   DelayedYoDouble delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", variableToDelay, ticksToDelay, &registry);
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 0.0, 1e-7);

   variableToDelay.set(1.0);
   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 0.0, 1e-7);

   variableToDelay.set(2.0);
   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 1.0, 1e-7);

   variableToDelay.set(3.0);
   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 2.0, 1e-7);

   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);
}

TEST(DelayedYoDoubleTest, testDelayedYoVariableZeroTickDelay)
{
   registry::YoRegistry registry("registry");
   variable::YoDouble variableToDelay("variableToDelay", &registry);

   int ticksToDelay = 0;

   variableToDelay.set(0.0);
   DelayedYoDouble delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", variableToDelay, ticksToDelay, &registry);
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 0.0, 1e-7);

   variableToDelay.set(1.0);
   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 1.0, 1e-7);

   variableToDelay.set(2.0);
   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 2.0, 1e-7);

   variableToDelay.set(3.0);
   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);

   delayedYoVariable.update();
   EXPECT_NEAR(delayedYoVariable.getDoubleValue(), 3.0, 1e-7);
}
} // namespace
} // namespace ihmc::yovariables::filters
