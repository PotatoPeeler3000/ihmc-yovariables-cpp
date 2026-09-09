#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_integer.h"

using namespace ihmc::yovariables;

namespace
{
class YoIntegerTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("testRegistry");
      random = std::mt19937(1776U);
      yoInteger = std::make_unique<variable::YoInteger>("test", registry.get());
   }

   static constexpr double EPSILON = 1e-10;
   std::unique_ptr<registry::YoRegistry> registry;
   std::mt19937 random;
   std::unique_ptr<variable::YoInteger> yoInteger;
};

TEST_F(YoIntegerTest, testSetAndGet)
{
   std::uniform_int_distribution<std::int32_t> dist(0, std::numeric_limits<std::int32_t>::max());
   for (int i = 0; i < 100; i++)
   {
      std::int32_t value = dist(random);
      yoInteger->set(value);
      EXPECT_EQ(value, yoInteger->getIntegerValue());
   }
}

TEST_F(YoIntegerTest, testIncrementDecrementAddSubtract)
{
   std::int32_t value = 12345;
   yoInteger->set(value);

   yoInteger->increment();
   EXPECT_EQ(value + 1, yoInteger->getIntegerValue());

   yoInteger->decrement();
   EXPECT_EQ(value, yoInteger->getIntegerValue());

   yoInteger->add(value);
   EXPECT_EQ(value * 2, yoInteger->getIntegerValue());

   yoInteger->sub(value);
   EXPECT_EQ(value, yoInteger->getIntegerValue());
}

TEST_F(YoIntegerTest, testLargeValue)
{
   std::int32_t value = std::numeric_limits<std::int32_t>::max() - 2;
   yoInteger->set(value);
   EXPECT_EQ(value, yoInteger->getIntegerValue());
}

TEST_F(YoIntegerTest, testValueEquals)
{
   EXPECT_TRUE(yoInteger->valueEquals(0));
}

TEST_F(YoIntegerTest, testSetFinal)
{
   EXPECT_EQ(0, yoInteger->getIntegerValue());
   yoInteger->set(0);
   EXPECT_EQ(0, yoInteger->getIntegerValue());

   std::int32_t value = 7777;
   yoInteger->set(value);
   EXPECT_EQ(value, yoInteger->getIntegerValue());
}

TEST_F(YoIntegerTest, testSetValueFromDouble)
{
   double doubleValue = 4.2;
   std::int32_t intValue = static_cast<std::int32_t>(std::lround(doubleValue));
   yoInteger->setValueFromDouble(doubleValue, true);
   EXPECT_EQ(intValue, yoInteger->getIntegerValue());
}

TEST_F(YoIntegerTest, testGetValueAsDouble)
{
   std::int32_t value = 15;
   EXPECT_EQ(0, yoInteger->getIntegerValue());
   yoInteger->set(value);
   EXPECT_NEAR(yoInteger->getValueAsDouble(), 15.0, EPSILON);
}

TEST_F(YoIntegerTest, testToString)
{
   EXPECT_EQ(yoInteger->toString(), yoInteger->getName() + ": " + std::to_string(yoInteger->getIntegerValue()));
}

TEST_F(YoIntegerTest, testGetYoVariableType)
{
   EXPECT_EQ(variable::YoVariableType::INTEGER, yoInteger->getType());
}

TEST_F(YoIntegerTest, testGetAndSetValueAsLongBits)
{
   std::int32_t value = 57;
   yoInteger->set(value);
   std::int64_t result = yoInteger->getValueAsLongBits();
   EXPECT_EQ(value, result);

   std::int64_t longValue = 12345;
   yoInteger->setValueFromLongBits(longValue, true);
   EXPECT_EQ(longValue, yoInteger->getValueAsLongBits());
}

TEST_F(YoIntegerTest, testDuplicate)
{
   variable::YoInteger yoInteger2("var2", "descriptionTest", registry.get());
   registry::YoRegistry newRegistry("newRegistry");
   std::unique_ptr<variable::YoVariable> duplicate = yoInteger2.duplicate(&newRegistry);
   EXPECT_EQ(yoInteger2.getName(), duplicate->getName());
   EXPECT_EQ(yoInteger2.getDescription(), duplicate->getDescription());
   EXPECT_NEAR(yoInteger2.getLowerBound(), duplicate->getLowerBound(), EPSILON);
   EXPECT_NEAR(yoInteger2.getUpperBound(), duplicate->getUpperBound(), EPSILON);
}

TEST_F(YoIntegerTest, testProviderValue)
{
   yoInteger->set(1250948);
   EXPECT_EQ(yoInteger->getIntegerValue(), yoInteger->getValue());
   yoInteger->set(-521);
   EXPECT_EQ(yoInteger->getIntegerValue(), yoInteger->getValue());
}
} // namespace
