#include <gtest/gtest.h>

#include <random>

#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"

using namespace ihmc::yovariables;

namespace
{
class YoBooleanTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("testRegistry");
      yoBoolean = std::make_unique<variable::YoBoolean>("booleanVariable", registry.get());
   }

   static constexpr double EPSILON = 1e-10;
   std::unique_ptr<registry::YoRegistry> registry;
   std::unique_ptr<variable::YoBoolean> yoBoolean;
};

TEST_F(YoBooleanTest, testBooleanYoVariable)
{
   EXPECT_EQ(yoBoolean->getName(), "booleanVariable");
   EXPECT_EQ(yoBoolean->getRegistry()->getName(), "testRegistry");
   EXPECT_FALSE(yoBoolean->getBooleanValue());
}

TEST_F(YoBooleanTest, testValueEquals)
{
   yoBoolean->set(false);
   EXPECT_FALSE(yoBoolean->valueEquals(true));

   yoBoolean->set(true);
   EXPECT_TRUE(yoBoolean->valueEquals(true));

   yoBoolean->set(false);
   EXPECT_TRUE(yoBoolean->valueEquals(false));

   yoBoolean->set(true);
   EXPECT_FALSE(yoBoolean->valueEquals(false));
}

TEST_F(YoBooleanTest, testGetAndSetBooleanYoVariable)
{
   yoBoolean->set(false);
   EXPECT_FALSE(yoBoolean->getBooleanValue());

   yoBoolean->set(true);
   EXPECT_TRUE(yoBoolean->getBooleanValue());
}

TEST_F(YoBooleanTest, testSet_boolean_boolean)
{
   EXPECT_FALSE(yoBoolean->getBooleanValue());

   bool result = yoBoolean->set(true, true);
   EXPECT_TRUE(yoBoolean->getBooleanValue());
   EXPECT_TRUE(result);

   result = yoBoolean->set(true, true);
   EXPECT_FALSE(result);
}

TEST_F(YoBooleanTest, testGetAndSetAsDouble)
{
   std::mt19937 rng(std::random_device{}());
   std::uniform_real_distribution<double> unit(0.0, 1.0);

   yoBoolean->setValueFromDouble(0.0);
   EXPECT_FALSE(yoBoolean->getBooleanValue());

   yoBoolean->setValueFromDouble(10000);
   EXPECT_TRUE(yoBoolean->getBooleanValue());

   yoBoolean->setValueFromDouble(-0.4);
   EXPECT_FALSE(yoBoolean->getBooleanValue());

   for (int counter = 0; counter < 100; counter++)
   {
      double testRandom = unit(rng);
      yoBoolean->setValueFromDouble(testRandom);
      if (testRandom >= 0.5)
         EXPECT_TRUE(yoBoolean->getBooleanValue());
      else
         EXPECT_FALSE(yoBoolean->getBooleanValue());
   }
}

TEST_F(YoBooleanTest, testGetValueAsDouble)
{
   EXPECT_FALSE(yoBoolean->getBooleanValue());
   EXPECT_NEAR(yoBoolean->getValueAsDouble(), 0.0, EPSILON);

   yoBoolean->set(true);
   EXPECT_TRUE(yoBoolean->getBooleanValue());
   EXPECT_NEAR(yoBoolean->getValueAsDouble(), 1.0, EPSILON);
}

TEST_F(YoBooleanTest, testToString)
{
   yoBoolean->set(false);
   EXPECT_EQ(yoBoolean->toString(), "booleanVariable: false");
   yoBoolean->set(true);
   EXPECT_EQ(yoBoolean->toString(), "booleanVariable: true");
}

TEST_F(YoBooleanTest, testGetAndSetValueAsLongBits)
{
   EXPECT_FALSE(yoBoolean->getBooleanValue());
   EXPECT_EQ(yoBoolean->getValueAsLongBits(), 0);

   yoBoolean->set(true);
   EXPECT_TRUE(yoBoolean->getBooleanValue());
   std::int64_t value = yoBoolean->getValueAsLongBits();
   EXPECT_EQ(value, 1);

   yoBoolean->setValueFromLongBits(value, true);
}

TEST_F(YoBooleanTest, testYoVariableType)
{
   EXPECT_EQ(yoBoolean->getType(), variable::YoVariableType::BOOLEAN);
}

TEST_F(YoBooleanTest, testDuplicate)
{
   registry::YoRegistry newRegistry("newTestRegistry");

   yoBoolean->set(true);
   std::unique_ptr<variable::YoVariable> val = yoBoolean->duplicate(&newRegistry);
   variable::YoVariable* testVal = newRegistry.getVariables().at(0);

   EXPECT_EQ(yoBoolean->getBooleanValue(), dynamic_cast<variable::YoBoolean*>(val.get())->getBooleanValue());
   EXPECT_EQ(yoBoolean->getBooleanValue(), dynamic_cast<variable::YoBoolean*>(testVal)->getBooleanValue());
   EXPECT_EQ(val->getRegistry(), &newRegistry);
   EXPECT_EQ(val->getRegistry(), testVal->getRegistry());
}

TEST_F(YoBooleanTest, testProviderValue)
{
   EXPECT_EQ(yoBoolean->getBooleanValue(), yoBoolean->getValue());
   yoBoolean->set(true);
   EXPECT_EQ(yoBoolean->getBooleanValue(), yoBoolean->getValue());
}
} // namespace
