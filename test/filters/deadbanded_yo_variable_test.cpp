#include <gtest/gtest.h>

#include "ihmc/yovariables/filters/deadbanded_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
TEST(DeadbandedYoVariableTest, testDeadband)
{
   registry::YoRegistry registry("test");
   variable::YoDouble deadzoneSize("deadzoneSize", &registry);
   variable::YoDouble input("input", &registry);
   double deadzone = 2.0;
   deadzoneSize.set(deadzone);
   DeadbandedYoVariable testDeadzone("testDeadZone", input, deadzoneSize, &registry);

   double verySmallStep = 1e-4;

   input.set(deadzone - verySmallStep);
   testDeadzone.update();
   EXPECT_NEAR(testDeadzone.getDoubleValue(), 0.0, 1e-14);
   input.set(deadzone + verySmallStep);
   testDeadzone.update();
   EXPECT_NEAR(testDeadzone.getDoubleValue(), verySmallStep, 1e-14);

   input.set(-deadzone + verySmallStep);
   testDeadzone.update();
   EXPECT_NEAR(testDeadzone.getDoubleValue(), 0.0, 1e-14);
   input.set(-deadzone - verySmallStep);
   testDeadzone.update();
   EXPECT_NEAR(testDeadzone.getDoubleValue(), -verySmallStep, 1e-14);

   for (double valueToBeCorrected = -10.0; valueToBeCorrected < -deadzone; valueToBeCorrected += 0.01)
   {
      input.set(valueToBeCorrected);
      testDeadzone.update();
      EXPECT_NEAR(testDeadzone.getDoubleValue(), valueToBeCorrected + deadzone, 1e-14);
   }
   for (double valueToBeCorrected = -deadzone; valueToBeCorrected < deadzone; valueToBeCorrected += 0.01)
   {
      input.set(valueToBeCorrected);
      testDeadzone.update();
      EXPECT_NEAR(testDeadzone.getDoubleValue(), 0.0, 1e-14);
   }
   for (double valueToBeCorrected = deadzone; valueToBeCorrected < 10.0; valueToBeCorrected += 0.01)
   {
      input.set(valueToBeCorrected);
      testDeadzone.update();
      EXPECT_NEAR(testDeadzone.getDoubleValue(), valueToBeCorrected - deadzone, 1e-14);
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
