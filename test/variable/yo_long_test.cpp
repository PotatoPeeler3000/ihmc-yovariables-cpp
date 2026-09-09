#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_long.h"

using namespace ihmc::yovariables;

namespace
{
class YoLongTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("testRegistry");
      random = std::mt19937_64(1776U);
      yoLong = std::make_unique<variable::YoLong>("test", registry.get());
   }

   static constexpr double EPSILON = 1e-10;
   std::unique_ptr<registry::YoRegistry> registry;
   std::mt19937_64 random;
   std::unique_ptr<variable::YoLong> yoLong;
};

TEST_F(YoLongTest, testSetAndGet)
{
   std::uniform_int_distribution<std::int64_t> dist(std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::max());
   for (int i = 0; i < 100; i++)
   {
      std::int64_t value = dist(random);
      yoLong->set(value, i % 2 == 0);
      EXPECT_EQ(value, yoLong->getLongValue());
   }
}

TEST_F(YoLongTest, testIncrementDecrementAddSubtract)
{
   std::int64_t value = 123456789012LL;
   yoLong->set(value);

   yoLong->increment();
   EXPECT_EQ(value + 1, yoLong->getLongValue());

   yoLong->decrement();
   EXPECT_EQ(value, yoLong->getLongValue());

   yoLong->add(value);
   EXPECT_EQ(value * 2, yoLong->getLongValue());

   yoLong->subtract(value);
   EXPECT_EQ(value, yoLong->getLongValue());
}

TEST_F(YoLongTest, testLargeValue)
{
   std::int64_t value = std::numeric_limits<std::int64_t>::max() - 2;
   yoLong->set(value);
   EXPECT_EQ(value, yoLong->getLongValue());
}

TEST_F(YoLongTest, testValueEquals)
{
   EXPECT_TRUE(yoLong->valueEquals(0));

   std::int64_t number = 987654321LL;
   yoLong->set(number);
   EXPECT_TRUE(yoLong->valueEquals(number));
}

TEST_F(YoLongTest, testSetFinal)
{
   EXPECT_EQ(0, yoLong->getLongValue());
   yoLong->set(0);
   EXPECT_EQ(0, yoLong->getLongValue());

   std::int32_t value = 7777;
   yoLong->set(value);
   EXPECT_EQ(value, yoLong->getLongValue());
}

TEST_F(YoLongTest, testSetValueFromDouble)
{
   double doubleValue = 4.2;
   std::int64_t intValue = static_cast<std::int64_t>(std::lround(doubleValue));
   yoLong->setValueFromDouble(doubleValue, true);
   EXPECT_EQ(intValue, yoLong->getLongValue());
}

TEST_F(YoLongTest, testGetValueAsDouble)
{
   EXPECT_NEAR(yoLong->getValueAsDouble(), 0.0, EPSILON);
   std::int64_t value = 15;
   yoLong->set(value);
   EXPECT_NEAR(yoLong->getValueAsDouble(), 15.0, EPSILON);
}

TEST_F(YoLongTest, testToString)
{
   EXPECT_EQ(yoLong->toString(), yoLong->getName() + ": " + std::to_string(yoLong->getLongValue()));
}

TEST_F(YoLongTest, testGetYoVariableType)
{
   EXPECT_EQ(variable::YoVariableType::LONG, yoLong->getType());
}

TEST_F(YoLongTest, testGetAndSetValueAsLongBits)
{
   std::int64_t value = 57;
   yoLong->set(value);
   EXPECT_EQ(value, yoLong->getValueAsLongBits());

   std::int64_t longValue = 12345;
   yoLong->setValueFromLongBits(longValue, true);
   EXPECT_EQ(longValue, yoLong->getValueAsLongBits());
}

TEST_F(YoLongTest, testDuplicate)
{
   variable::YoLong yoLong2("var2", "descriptionTest", registry.get());
   registry::YoRegistry newRegistry("newRegistry");
   std::unique_ptr<variable::YoVariable> duplicate = yoLong2.duplicate(&newRegistry);
   EXPECT_EQ(yoLong2.getName(), duplicate->getName());
   EXPECT_EQ(yoLong2.getDescription(), duplicate->getDescription());
   EXPECT_NEAR(yoLong2.getLowerBound(), duplicate->getLowerBound(), EPSILON);
   EXPECT_NEAR(yoLong2.getUpperBound(), duplicate->getUpperBound(), EPSILON);
}

TEST_F(YoLongTest, testProviderValue)
{
   yoLong->set(10LL * std::numeric_limits<std::int32_t>::max());
   EXPECT_EQ(yoLong->getLongValue(), yoLong->getValue());
   yoLong->set(10LL * std::numeric_limits<std::int32_t>::min());
   EXPECT_EQ(yoLong->getLongValue(), yoLong->getValue());
}
} // namespace
