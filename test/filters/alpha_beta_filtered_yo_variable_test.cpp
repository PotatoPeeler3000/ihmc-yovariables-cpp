#include <gtest/gtest.h>

#include "ihmc/yovariables/filters/alpha_beta_filtered_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
constexpr double kDt = 0.1;

TEST(AlphaBetaFilteredYoVariableTest, testAlphaBetaFilteredVelocityAndPositionEstimatesWithNoVelocity)
{
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble positionVariable("positionVariable", &registry);
   variable::YoDouble xMeasuredVariable("xMeasuredVariable", &registry);

   double alpha = 0.2;
   double beta = 0.35;

   AlphaBetaFilteredYoVariable abFilteredYoVariable("abFilteredYoVariable", &registry, alpha, beta, positionVariable, xMeasuredVariable, kDt);

   abFilteredYoVariable.set(0);
   positionVariable.set(0);
   xMeasuredVariable.set(42);

   for (int i = 0; i < 10000; i++)
      abFilteredYoVariable.update();

   // Converges on 0 since position doesn't change (no dx/dt)
   EXPECT_NEAR(0, abFilteredYoVariable.getDoubleValue(), 1e-7);
}

TEST(AlphaBetaFilteredYoVariableTest, testAlphaBetaFilteredVelocityAndPositionEstimatesWithConstantVelocity)
{
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble positionVariable("positionVariable", &registry);
   variable::YoDouble xMeasuredVariable("xMeasuredVariable", &registry);

   double alpha = 0.2;
   double beta = 0.35;

   AlphaBetaFilteredYoVariable abFilteredYoVariable("abFilteredYoVariable", &registry, alpha, beta, positionVariable, xMeasuredVariable, kDt);

   for (int i = 0; i < 10000; i++)
   {
      abFilteredYoVariable.set(0);
      positionVariable.set(0);
      xMeasuredVariable.set(42);

      for (int j = 0; j < 10000; j++)
      {
         xMeasuredVariable.set(xMeasuredVariable.getDoubleValue() + 10); // Velocity = 100 distances per time
         abFilteredYoVariable.update();
      }

      EXPECT_NEAR(100, abFilteredYoVariable.getDoubleValue(), 1e-7);
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
