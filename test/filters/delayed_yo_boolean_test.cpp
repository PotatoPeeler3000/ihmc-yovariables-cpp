#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "ihmc/yovariables/filters/delayed_yo_boolean.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"

// Java's getInternalState(String, boolean debug) calls (all with debug=false in this file, so they
// never actually print anything) have no C++ equivalent - not part of this port's DelayedYoBoolean
// API - and are dropped throughout this file.

namespace ihmc::yovariables::filters
{
namespace
{
class DelayedYoBooleanTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("registry");
      variableToDelay = std::make_unique<variable::YoBoolean>("variableToDelay", registry.get());
   }

   std::unique_ptr<registry::YoRegistry> registry;
   std::unique_ptr<variable::YoBoolean> variableToDelay;
};

TEST_F(DelayedYoBooleanTest, testDelayedYoVariableMultipleTickDelays)
{
   std::mt19937 rng(1U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   for (int ticksToDelay = 0; ticksToDelay < 10; ticksToDelay++)
   {
      variableToDelay->set(true);

      DelayedYoBoolean delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", *variableToDelay, ticksToDelay, registry.get());

      int ticksToTest = 100;
      std::vector<bool> valuesToSet(ticksToTest);

      for (int i = 0; i < ticksToTest; i++)
         valuesToSet[i] = dist(rng) < 0.5;

      EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

      for (int i = 0; i < ticksToTest; i++)
      {
         variableToDelay->set(valuesToSet[i]);
         delayedYoVariable.update();

         if (i < ticksToDelay)
            EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);
         else
            EXPECT_EQ(delayedYoVariable.getBooleanValue(), valuesToSet[i - ticksToDelay]);
      }
   }
}

TEST_F(DelayedYoBooleanTest, testDelayedYoVariableOneTickDelay)
{
   int ticksToDelay = 1;

   variableToDelay->set(false);
   DelayedYoBoolean delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", *variableToDelay, ticksToDelay, registry.get());
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);

   variableToDelay->set(true);
   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);

   variableToDelay->set(false);
   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   variableToDelay->set(true);
   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);

   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);
}

TEST_F(DelayedYoBooleanTest, testDelayedYoVariableZeroTickDelay)
{
   int ticksToDelay = 0;

   variableToDelay->set(false);
   DelayedYoBoolean delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", *variableToDelay, ticksToDelay, registry.get());
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);

   variableToDelay->set(true);
   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   variableToDelay->set(false);
   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);

   variableToDelay->set(true);
   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);
}

TEST_F(DelayedYoBooleanTest, testUpdateWithZero)
{
   int ticksToDelay = 0;
   variableToDelay->set(false);
   DelayedYoBoolean delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", *variableToDelay, ticksToDelay, registry.get());

   variableToDelay->set(true);
   delayedYoVariable.update();

   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);
   delayedYoVariable.update();
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);
}

TEST_F(DelayedYoBooleanTest, testReset)
{
   int ticksToDelay = 10;
   variableToDelay->set(false);
   DelayedYoBoolean delayedYoVariable("delayedVariable" + std::to_string(ticksToDelay), "", *variableToDelay, ticksToDelay, registry.get());

   for (int i = 0; i < ticksToDelay; i++)
   {
      EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);
      delayedYoVariable.update();
   }

   variableToDelay->set(true);
   delayedYoVariable.update();

   for (int i = 0; i < ticksToDelay; i++)
   {
      EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);
      delayedYoVariable.update();
   }
   EXPECT_EQ(delayedYoVariable.getBooleanValue(), true);

   variableToDelay->set(false);
   delayedYoVariable.update();

   delayedYoVariable.reset();
   delayedYoVariable.update();

   for (int i = 0; i < ticksToDelay; i++)
   {
      EXPECT_EQ(delayedYoVariable.getBooleanValue(), false);
      delayedYoVariable.update();
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
