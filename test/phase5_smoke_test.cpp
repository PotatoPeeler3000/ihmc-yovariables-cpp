#include <gtest/gtest.h>

#include "ihmc/yovariables/filters/alpha_filtered_yo_variable.h"
#include "ihmc/yovariables/filters/backlash_processing_yo_variable.h"
#include "ihmc/yovariables/filters/delayed_yo_double.h"
#include "ihmc/yovariables/filters/glitch_filtered_yo_boolean.h"
#include "ihmc/yovariables/filters/rate_limited_yo_variable.h"
#include "ihmc/yovariables/filters/running_average_yo_double.h"
#include "ihmc/yovariables/filters/simple_moving_average_filtered_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"

using namespace ihmc::yovariables;

TEST(Phase5Smoke, AlphaFilteredYoVariableConvergesTowardConstantInput)
{
   registry::YoRegistry root("root");
   filters::AlphaFilteredYoVariable filtered("filtered", &root, 0.5);

   for (int i = 0; i < 50; i++)
      filtered.update(10.0);

   EXPECT_NEAR(filtered.getDoubleValue(), 10.0, 1e-6);
}

TEST(Phase5Smoke, RateLimitedYoVariableClampsStep)
{
   registry::YoRegistry root("root");
   filters::RateLimitedYoVariable limited("limited", &root, 1.0, 0.1);

   limited.update(0.0);
   limited.update(10.0);

   EXPECT_NEAR(limited.getDoubleValue(), 0.1, 1e-9);
}

TEST(Phase5Smoke, GlitchFilteredYoBooleanIgnoresShortGlitch)
{
   registry::YoRegistry root("root");
   filters::GlitchFilteredYoBoolean filtered("filtered", &root, 3);

   filtered.update(true);
   filtered.update(true);
   EXPECT_FALSE(filtered.getBooleanValue());

   filtered.update(false);
   filtered.update(true);
   filtered.update(true);
   filtered.update(true);
   EXPECT_TRUE(filtered.getBooleanValue());
}

TEST(Phase5Smoke, RunningAverageYoDoubleTracksMean)
{
   registry::YoRegistry root("root");
   filters::RunningAverageYoDouble average("average", &root);

   average.update(1.0);
   average.update(2.0);
   average.update(3.0);

   EXPECT_NEAR(average.getDoubleValue(), 2.0, 1e-9);
}

TEST(Phase5Smoke, SimpleMovingAverageFilteredYoVariable)
{
   registry::YoRegistry root("root");
   filters::SimpleMovingAverageFilteredYoVariable movingAverage("movingAverage", 3, &root);

   movingAverage.update(3.0);
   movingAverage.update(6.0);
   movingAverage.update(9.0);

   EXPECT_NEAR(movingAverage.getDoubleValue(), 6.0, 1e-9);
   EXPECT_TRUE(movingAverage.getHasBufferWindowFilled());
}

TEST(Phase5Smoke, DelayedYoDoubleDelaysByTickCount)
{
   registry::YoRegistry root("root");
   variable::YoDouble source("source", &root);
   filters::DelayedYoDouble delayed("delayed", "", source, 2, &root);

   source.set(1.0);
   delayed.update();
   source.set(2.0);
   delayed.update();
   source.set(3.0);
   delayed.update();

   EXPECT_DOUBLE_EQ(delayed.getValue(), 1.0);
}

TEST(Phase5Smoke, BacklashProcessingYoVariableZerosOnDirectionChange)
{
   registry::YoRegistry root("root");
   variable::YoDouble slopTime("slopTime", &root);
   slopTime.set(1.0);

   filters::BacklashProcessingYoVariable backlash("backlash", "", 0.1, slopTime, &root);

   backlash.update(1.0);
   backlash.update(1.0);
   backlash.update(-1.0);

   EXPECT_DOUBLE_EQ(backlash.getDoubleValue(), 0.0);
}
