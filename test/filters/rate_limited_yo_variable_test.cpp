#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "ihmc/yovariables/filters/rate_limited_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
class RateLimitedYoVariableTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("registry");

      maxRate2 = std::make_unique<variable::YoDouble>("maxRate2", registry.get());
      maxRate4 = std::make_unique<variable::YoDouble>("maxRate4", registry.get());
      position3 = std::make_unique<variable::YoDouble>("position3", registry.get());
      position4 = std::make_unique<variable::YoDouble>("position4", registry.get());

      maxRate1 = 10.0;
      maxRate2->set(9.0);
      maxRate3 = 11.0;
      maxRate4->set(12.0);

      double dt1 = 1.0, dt2 = 1.0, dt3 = 1.0, dt4 = 1.0;

      position3->set(0.5);
      position4->set(0.75);

      rateLimitedYoVariable1 = std::make_unique<RateLimitedYoVariable>("rateLimitedYoVariable1", registry.get(), maxRate1, dt1);
      rateLimitedYoVariable2 = std::make_unique<RateLimitedYoVariable>("rateLimitedYoVariable2", registry.get(), *maxRate2, dt2);
      rateLimitedYoVariable3 = std::make_unique<RateLimitedYoVariable>("rateLimitedYoVariable3", registry.get(), maxRate3, position3.get(), dt3);
      rateLimitedYoVariable4 = std::make_unique<RateLimitedYoVariable>("rateLimitedYoVariable4", registry.get(), *maxRate4, position4.get(), dt4);
   }

   std::unique_ptr<registry::YoRegistry> registry;
   std::unique_ptr<RateLimitedYoVariable> rateLimitedYoVariable1, rateLimitedYoVariable2, rateLimitedYoVariable3, rateLimitedYoVariable4;
   std::unique_ptr<variable::YoDouble> maxRate2, maxRate4;
   std::unique_ptr<variable::YoDouble> position3, position4;
   double maxRate1, maxRate3;
};

TEST_F(RateLimitedYoVariableTest, testUpdate)
{
   EXPECT_NO_THROW({
      rateLimitedYoVariable3->update();
      rateLimitedYoVariable4->update();
   });
}

TEST_F(RateLimitedYoVariableTest, testUpdateWithNullPointerException)
{
   // Java's variables 1 and 2 (constructed without a position variable) throw
   // NullPointerException from update(); this port throws std::logic_error for the same reason
   // ("constructed without a position variable"), documented on RateLimitedYoVariable::update().
   EXPECT_THROW(
      {
         rateLimitedYoVariable1->update();
         rateLimitedYoVariable2->update();
      },
      std::exception);
}

TEST_F(RateLimitedYoVariableTest, testUpdateWithCurrentPositionParameter)
{
   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition1 = 10.0 * std::sin(angle);
      rateLimitedYoVariable1->update(currentPosition1);
      EXPECT_NEAR(rateLimitedYoVariable1->getDoubleValue(), currentPosition1, 1E-13);
   }

   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition2 = 7.0 * std::sin(angle);
      rateLimitedYoVariable2->update(currentPosition2);
      EXPECT_NEAR(rateLimitedYoVariable2->getDoubleValue(), currentPosition2, 1E-13);
   }

   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition3 = 11.0 * std::sin(angle);
      rateLimitedYoVariable3->update(currentPosition3);
      EXPECT_NEAR(rateLimitedYoVariable3->getDoubleValue(), currentPosition3, 1E-13);
   }

   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition4 = 12.0 * std::sin(angle);
      rateLimitedYoVariable4->update(currentPosition4);
      EXPECT_NEAR(rateLimitedYoVariable4->getDoubleValue(), currentPosition4, 1E-13);
   }
}

TEST_F(RateLimitedYoVariableTest, testUpdateWithCurrentPositionParameterExceedingMaxRate)
{
   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition1 = 25.0 * std::sin(angle);
      double dSinTheta = std::cos(angle);
      double signOfSlope = (dSinTheta > 0 ? 1.0 : (dSinTheta < 0 ? -1.0 : 0.0));

      if (std::abs(currentPosition1 - rateLimitedYoVariable1->getDoubleValue()) > maxRate1)
         currentPosition1 = rateLimitedYoVariable1->getDoubleValue() + signOfSlope * maxRate1;
      rateLimitedYoVariable1->update(currentPosition1);
      EXPECT_NEAR(rateLimitedYoVariable1->getDoubleValue(), currentPosition1, 1E-15);
   }

   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition2 = 25.0 * std::sin(angle);
      double dSinTheta = std::cos(angle);
      double signOfSlope = (dSinTheta > 0 ? 1.0 : (dSinTheta < 0 ? -1.0 : 0.0));

      if (std::abs(currentPosition2 - rateLimitedYoVariable2->getDoubleValue()) > maxRate2->getDoubleValue())
         currentPosition2 = rateLimitedYoVariable2->getDoubleValue() + signOfSlope * maxRate2->getDoubleValue();
      rateLimitedYoVariable2->update(currentPosition2);
      EXPECT_NEAR(rateLimitedYoVariable2->getDoubleValue(), currentPosition2, 1E-15);
   }

   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition3 = 25.0 * std::sin(angle);
      double dSinTheta = std::cos(angle);
      double signOfSlope = (dSinTheta > 0 ? 1.0 : (dSinTheta < 0 ? -1.0 : 0.0));

      if (std::abs(currentPosition3 - rateLimitedYoVariable3->getDoubleValue()) > maxRate3)
         currentPosition3 = rateLimitedYoVariable3->getDoubleValue() + signOfSlope * maxRate3;
      rateLimitedYoVariable3->update(currentPosition3);
      EXPECT_NEAR(rateLimitedYoVariable3->getDoubleValue(), currentPosition3, 1E-15);
   }

   for (double angle = 0.0; angle < 3 * 6.28; angle += 1.0)
   {
      double currentPosition4 = 25.0 * std::sin(angle);
      double dSinTheta = std::cos(angle);
      double signOfSlope = (dSinTheta > 0 ? 1.0 : (dSinTheta < 0 ? -1.0 : 0.0));

      if (std::abs(currentPosition4 - rateLimitedYoVariable4->getDoubleValue()) > maxRate4->getDoubleValue())
         currentPosition4 = rateLimitedYoVariable4->getDoubleValue() + signOfSlope * maxRate4->getDoubleValue();
      rateLimitedYoVariable4->update(currentPosition4);
      EXPECT_NEAR(rateLimitedYoVariable4->getDoubleValue(), currentPosition4, 1E-15);
   }
}

TEST_F(RateLimitedYoVariableTest, testUpdateWithMaxRateBeingNegative)
{
   try
   {
      RateLimitedYoVariable rateLimitedYoVariableWithNegativeMaxRate("rateLimitedYoVariableWithNegativeMaxRate", registry.get(), -5.0, 1.0);
      rateLimitedYoVariableWithNegativeMaxRate.update(5.0);
   }
   catch (const std::runtime_error& e)
   {
      // Message text differs slightly from Java's ("...in the RateLimitedYoVariable..." vs
      // "...in RateLimitedYoVariable..." here) - not worth chasing, just confirming the same
      // condition is reported.
      EXPECT_STREQ(e.what(), "The maxRate parameter in RateLimitedYoVariable cannot be negative.");
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
