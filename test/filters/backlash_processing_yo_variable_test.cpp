#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ihmc/yovariables/filters/alpha_filter_tools.h"
#include "ihmc/yovariables/filters/backlash_compensating_velocity_yo_variable.h"
#include "ihmc/yovariables/filters/backlash_processing_yo_variable.h"
#include "ihmc/yovariables/filters/filtered_finite_difference_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
TEST(BacklashProcessingYoVariableTest, testAgainstRevisedBacklash)
{
   registry::YoRegistry registry("dummy");
   variable::YoDouble slopTime("slopTime", &registry);
   double dt = 0.002;
   variable::YoDouble alpha("alpha", &registry);
   alpha.set(computeAlphaGivenBreakFrequencyProperly(16.0, dt));
   variable::YoDouble positionVariable("rawPosition", &registry);
   FilteredFiniteDifferenceYoVariable velocityVariable("fd", "", alpha, dt, &registry, &positionVariable);

   BacklashProcessingYoVariable blToTest("blTest", "", dt, slopTime, &registry, &velocityVariable);

   BacklashCompensatingVelocityYoVariable blExpected("blExpected", "", alpha, dt, slopTime, &registry, &positionVariable);

   std::mt19937 random(561651U);
   std::uniform_real_distribution<double> dist(-1.0, 1.0);

   for (double t = 0.0; t < 100.0; t += dt)
   {
      positionVariable.set(2.0 * std::sin(2.0 * M_PI * 10.0) + dist(random) * std::sin(2.0 * M_PI * 30.0 + 2.0 / 3.0 * M_PI));

      velocityVariable.update();

      blToTest.update();
      blExpected.update();

      EXPECT_NEAR(blToTest.getDoubleValue(), blExpected.getDoubleValue(), 1.0e-10);
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
