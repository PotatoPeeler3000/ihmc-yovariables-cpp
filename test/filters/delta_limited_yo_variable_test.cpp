#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ihmc/yovariables/filters/delta_limited_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"

namespace ihmc::yovariables::filters
{
namespace
{
constexpr int kRandomLowerBound = 10;
constexpr int kRandomUpperBound = 30000;

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothNegativeNoOvershootInputGreaterThanReference)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random) * -1.0;
      double input = reference / 2.0;
      double delta = std::abs(input - reference);

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      EXPECT_LT(input, 0.0);
      EXPECT_LT(reference, 0.0);
      EXPECT_GT(input, reference);
      EXPECT_FALSE(variable.isLimitingActive());
      EXPECT_NEAR(input, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothNegativeNoOvershootReferenceGreaterThanInput)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random) * -1.0;
      double input = reference * 2;
      double delta = std::abs(input - reference);

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      EXPECT_LT(input, 0.0);
      EXPECT_LT(reference, 0.0);
      EXPECT_LT(input, reference);
      EXPECT_FALSE(variable.isLimitingActive());
      EXPECT_NEAR(input, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothPositiveNoOvershootReferenceGreaterThanInput)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random);
      double input = reference / 2.0;
      double delta = std::abs(input - reference);

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      EXPECT_GT(input, 0.0);
      EXPECT_GT(reference, 0.0);
      EXPECT_LT(input, reference);
      EXPECT_FALSE(variable.isLimitingActive());
      EXPECT_NEAR(input, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothPositiveNoOvershootInputGreaterThanReference)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random);
      double input = reference * 2;
      double delta = std::abs(input - reference);

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      EXPECT_GT(input, 0.0);
      EXPECT_GT(reference, 0.0);
      EXPECT_GT(input, reference);
      EXPECT_FALSE(variable.isLimitingActive());
      EXPECT_NEAR(input, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testPositiveReferenceNegativeInputNoOvershoot)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random);
      double input = reference * -0.5;
      double delta = std::abs(input - reference);

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      EXPECT_LT(input, 0.0);
      EXPECT_GT(reference, 0.0);
      EXPECT_LT(input, reference);
      EXPECT_FALSE(variable.isLimitingActive());
      EXPECT_NEAR(input, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testNegativeReferencePositiveInputNoOvershoot)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random) * -1.0;
      double input = reference * -0.5;
      double delta = std::abs(input - reference);

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      EXPECT_GT(input, 0.0);
      EXPECT_LT(reference, 0.0);
      EXPECT_GT(input, reference);
      EXPECT_FALSE(variable.isLimitingActive());
      EXPECT_NEAR(input, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothNegativeWithOvershootInputGreaterThanReference)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random) * -1.0;
      double input = reference / 2.0;
      double delta = std::abs(input - reference) / 2.0;

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      double overshoot = std::abs(input - reference) - delta;
      double expectedClip = input - overshoot;

      EXPECT_LT(input, 0.0);
      EXPECT_LT(reference, 0.0);
      EXPECT_GT(input, reference);
      EXPECT_TRUE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothNegativeWithOvershootReferenceGreaterThanInput)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random) * -1.0;
      double input = reference * 2;
      double delta = std::abs(input - reference) / 2.0;

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      double overshoot = std::abs(input - reference) - delta;
      double expectedClip = input + overshoot;

      EXPECT_LT(input, 0.0);
      EXPECT_LT(reference, 0.0);
      EXPECT_LT(input, reference);
      EXPECT_TRUE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothPositiveWithOvershootInputGreaterThanReference)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random);
      double input = reference * 2;
      double delta = std::abs(input - reference) / 2.0;

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      double overshoot = std::abs(input - reference) - delta;
      double expectedClip = input - overshoot;

      EXPECT_GT(input, 0.0);
      EXPECT_GT(reference, 0.0);
      EXPECT_GT(input, reference);
      EXPECT_TRUE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testReferenceAndInputBothPositiveWithOvershootReferenceGreaterThanInput)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random);
      double input = reference / 2.0;
      double delta = std::abs(input - reference) / 2.0;

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      double overshoot = std::abs(input - reference) - delta;
      double expectedClip = input + overshoot;

      EXPECT_GT(input, 0.0);
      EXPECT_GT(reference, 0.0);
      EXPECT_LT(input, reference);
      EXPECT_TRUE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testPositiveReferenceNegativeInputWithOvershoot)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random);
      double input = reference * -0.5;
      double delta = std::abs(input - reference) / 2.0;

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      double overshoot = std::abs(input - reference) - delta;
      double expectedClip = input + overshoot;

      EXPECT_LT(input, 0.0);
      EXPECT_GT(reference, 0.0);
      EXPECT_LT(input, reference);
      EXPECT_TRUE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testNegativeReferencePositiveInputWithOvershoot)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random) * -1.0;
      double input = reference * -0.5;
      double delta = std::abs(input - reference) / 2.0;

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      double overshoot = std::abs(input - reference) - delta;
      double expectedClip = input - overshoot;

      EXPECT_GT(input, 0.0);
      EXPECT_LT(reference, 0.0);
      EXPECT_GT(input, reference);
      EXPECT_TRUE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);
   }
}

TEST(DeltaLimitedYoVariableTest, testOvershootThenNoOvershoot)
{
   std::mt19937 random(1976U);
   std::uniform_int_distribution<int> dist(kRandomLowerBound, kRandomUpperBound);
   registry::YoRegistry registry("registry");
   DeltaLimitedYoVariable variable("testVar", &registry, 0.0);

   for (int i = 0; i < 60000; i++)
   {
      double reference = dist(random) * -1.0;
      double input = reference * -0.5;
      double delta = std::abs(input - reference) / 2.0;

      variable.setMaxDelta(delta);
      variable.updateOutput(reference, input);

      double overshoot = std::abs(input - reference) - delta;
      double expectedClip = input - overshoot;

      EXPECT_GT(input, 0.0);
      EXPECT_LT(reference, 0.0);
      EXPECT_GT(input, reference);
      EXPECT_TRUE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);

      input = variable.getDoubleValue();

      variable.updateOutput(reference, input);
      double newRequestedDelta = std::abs(input - reference);
      EXPECT_FALSE(newRequestedDelta > delta);
      EXPECT_FALSE(variable.isLimitingActive());
      EXPECT_NEAR(expectedClip, variable.getDoubleValue(), 1e-8);
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
