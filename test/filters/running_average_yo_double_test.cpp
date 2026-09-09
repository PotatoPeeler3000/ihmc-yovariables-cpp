#include <gtest/gtest.h>

#include <random>

#include "ihmc/yovariables/filters/running_average_yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
// Java's original compares against org.apache.commons.math3.stat.descriptive.moment.Mean, an
// external dependency not part of this port. Apache's Mean uses the same incremental-mean
// recurrence as RunningAverageYoDouble::update() (mean += (x - mean) / n), so replicating that
// recurrence directly here reproduces the same (bit-exact, per the Java test's no-delta
// assertEquals) reference value without needing the dependency.
TEST(RunningAverageYoDoubleTest, testAgainstApacheMean)
{
   std::mt19937 random(1U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   RunningAverageYoDouble yoAverage("", nullptr);
   double meanResult = 0.0;
   int meanN = 0;
   double next = -10.0;

   for (int i = 0; i < 1000; i++)
   {
      next += dist(random);
      yoAverage.update(next);

      meanN++;
      meanResult += (next - meanResult) / meanN;

      EXPECT_EQ(meanResult, yoAverage.getValue());
      EXPECT_EQ(meanN, yoAverage.getSampleSize());
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
