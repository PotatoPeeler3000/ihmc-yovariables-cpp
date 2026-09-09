#include <gtest/gtest.h>

#include "ihmc/yovariables/filters/filter_math.h"
#include "ihmc/yovariables/filters/filtered_discrete_velocity_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
constexpr double kDt = 0.1;

TEST(FilteredDiscreteVelocityYoVariableTest, testFilteredDiscreteVelocityNoDirectionChange)
{
   // Java's raw double alpha argument is wrapped in a ConstantDoubleProvider, since this port's
   // constructor takes a DoubleProvider& rather than a plain double.
   ConstantDoubleProvider alpha(0.99);
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble positionVariable("positionVariable", &registry);
   variable::YoDouble time("time", &registry);
   FilteredDiscreteVelocityYoVariable filteredDiscreteVelocityYoVariable("filteredDiscreteVelocityYoVariable", "", alpha, positionVariable, time, &registry);

   positionVariable.set(10);
   time.set(0);

   for (int i = 0; i < static_cast<int>(1000 / kDt); i++)
   {
      time.add(kDt);
      positionVariable.add(1);
      filteredDiscreteVelocityYoVariable.update();
   }

   EXPECT_NEAR(10, filteredDiscreteVelocityYoVariable.getDoubleValue(), 1e-7);
}

TEST(FilteredDiscreteVelocityYoVariableTest, testFilteredDiscreteVelocityWithDirectionChange)
{
   ConstantDoubleProvider alpha(0.99);
   registry::YoRegistry registry("testRegistry");
   variable::YoDouble positionVariable("positionVariable", &registry);
   variable::YoDouble time("time", &registry);
   FilteredDiscreteVelocityYoVariable filteredDiscreteVelocityYoVariable("filteredDiscreteVelocityYoVariable", "", alpha, positionVariable, time, &registry);

   positionVariable.set(10);
   time.set(0);

   for (int i = 0; i < static_cast<int>(1000 / kDt); i++)
   {
      time.add(kDt);
      positionVariable.add(1);
      filteredDiscreteVelocityYoVariable.update();
   }

   EXPECT_NEAR(10, filteredDiscreteVelocityYoVariable.getDoubleValue(), 1e-7);

   for (int i = 0; i < static_cast<int>(1000 / kDt); i++)
   {
      time.add(kDt);
      positionVariable.add(-1);
      filteredDiscreteVelocityYoVariable.update();
   }

   EXPECT_NEAR(-10, filteredDiscreteVelocityYoVariable.getDoubleValue(), 1e-7);
}
} // namespace
} // namespace ihmc::yovariables::filters
