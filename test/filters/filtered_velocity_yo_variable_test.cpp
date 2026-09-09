#include <gtest/gtest.h>

#include <cmath>

#include "ihmc/yovariables/filters/filtered_finite_difference_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
constexpr double kDt = 0.1;

TEST(FilteredVelocityYoVariableTest, testUpdateForTranslationalVelocity)
{
   registry::YoRegistry registry("testRegistry");
   double alpha = 0.3;
   variable::YoDouble positionVariable("positionVariable", &registry);

   FilteredFiniteDifferenceYoVariable filteredVelocityYoVariable("filteredVelocityYoVariable", "test description", alpha, kDt, &registry, &positionVariable);

   filteredVelocityYoVariable.set(0);
   positionVariable.set(0);

   for (int i = 0; i < 10000; i++)
   {
      positionVariable.add(10);
      filteredVelocityYoVariable.update();
   }

   EXPECT_NEAR(100, filteredVelocityYoVariable.getDoubleValue(), 1e-7);
}

TEST(FilteredVelocityYoVariableTest, testUpdateForRotationalVelocity)
{
   registry::YoRegistry registry("testRegistry");
   double alpha = 0.005;
   variable::YoDouble positionVariable("positionVariable", &registry);

   FilteredFiniteDifferenceYoVariable filteredVelocityYoVariable("filteredVelocityYoVariable", "test description", alpha, kDt, &registry, &positionVariable);

   filteredVelocityYoVariable.set(0);
   positionVariable.set(-M_PI);

   for (int i = 0; i < 10000; i++)
   {
      if (positionVariable.getValueAsDouble() + 0.5 > M_PI)
         positionVariable.set(-M_PI + (M_PI - positionVariable.getValueAsDouble()));
      else
         positionVariable.add(0.5);

      filteredVelocityYoVariable.updateForAngles();
   }

   EXPECT_NEAR(5, filteredVelocityYoVariable.getDoubleValue(), 1e-5);
}
} // namespace
} // namespace ihmc::yovariables::filters
