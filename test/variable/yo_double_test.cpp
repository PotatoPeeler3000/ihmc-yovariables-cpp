#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "ihmc/yovariables/listener/yo_variable_changed_listener.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

using namespace ihmc::yovariables;

namespace
{
class YoDoubleTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("testRegistry");
      yoDouble1 = std::make_unique<variable::YoDouble>("yoDouble1", registry.get());
      yoDouble2 = std::make_unique<variable::YoDouble>("yoDouble2", "description2", registry.get());
      yoDouble2->setVariableBounds(0.0, 10.0);
   }

   static constexpr double EPSILON = 1e-10;
   std::unique_ptr<registry::YoRegistry> registry;
   std::unique_ptr<variable::YoDouble> yoDouble1;
   std::unique_ptr<variable::YoDouble> yoDouble2;
   std::mt19937 random{345345U};
};

TEST_F(YoDoubleTest, testVariableChangeListenerNotification)
{
   bool valueChanged = false;
   struct Listener : listener::YoVariableChangedListener
   {
      bool* flag;
      void changed(variable::YoVariable&) override { *flag = true; }
   } listenerInstance;
   listenerInstance.flag = &valueChanged;
   yoDouble1->addListener(&listenerInstance);

   yoDouble1->set(yoDouble1->getValue() + 0.1);
   EXPECT_TRUE(valueChanged);
   valueChanged = false;

   yoDouble1->set(yoDouble1->getValue());
   EXPECT_FALSE(valueChanged);

   yoDouble1->set(std::numeric_limits<double>::infinity());
   EXPECT_TRUE(valueChanged);
   valueChanged = false;

   yoDouble1->set(std::numeric_limits<double>::infinity());
   EXPECT_FALSE(valueChanged);

   yoDouble1->set(-std::numeric_limits<double>::infinity());
   EXPECT_TRUE(valueChanged);
   valueChanged = false;

   yoDouble1->set(-std::numeric_limits<double>::infinity());
   EXPECT_FALSE(valueChanged);

   yoDouble1->set(std::numeric_limits<double>::quiet_NaN());
   EXPECT_TRUE(valueChanged);
   valueChanged = false;

   yoDouble1->set(std::numeric_limits<double>::quiet_NaN());
   EXPECT_FALSE(valueChanged);
}

TEST_F(YoDoubleTest, testDoubleYoVariableConstructorWithoutDescription)
{
   EXPECT_EQ(yoDouble1->getDoubleValue(), 0.0);
   EXPECT_EQ(yoDouble1->getName(), "yoDouble1");
}

TEST_F(YoDoubleTest, testDoubleYoVariableConstructorWithDescription)
{
   std::string testDescription = "This is a test description.";
   variable::YoDouble yoDoubleWithDescription("yoDoubleWithDescription", testDescription, registry.get());
   EXPECT_EQ(yoDoubleWithDescription.getDoubleValue(), 0.0);
   EXPECT_EQ(yoDoubleWithDescription.getName(), "yoDoubleWithDescription");
   EXPECT_EQ(yoDoubleWithDescription.getDescription(), testDescription);
}

TEST_F(YoDoubleTest, testToString)
{
   // Adapted rather than asserting an exact string: this port's double-to-string formatting is a
   // documented, deliberate divergence from Java's Double.toString() (see README.md), so the
   // Java test's exact-string assertion isn't meaningful here - what matters is that toString()
   // reports the name and the current value.
   yoDouble1->set(0.5);
   EXPECT_EQ(yoDouble1->toString(), "yoDouble1: " + std::to_string(0.5));
}

TEST_F(YoDoubleTest, testIsNaN)
{
   EXPECT_FALSE(yoDouble2->isNaN());
   yoDouble2->set(std::numeric_limits<double>::quiet_NaN());
   EXPECT_TRUE(yoDouble2->isNaN());
}

TEST_F(YoDoubleTest, testAdditionWithDoubles)
{
   double randomNumber1 = 0.37;
   double randomNumber2 = 0.61;

   yoDouble1->set(randomNumber1);
   yoDouble1->add(randomNumber2);
   EXPECT_EQ(yoDouble1->getDoubleValue(), randomNumber1 + randomNumber2);
}

TEST_F(YoDoubleTest, testSubtractionWithDoubles)
{
   double randomNumber1 = 0.83;
   double randomNumber2 = 0.12;

   yoDouble1->set(randomNumber1);
   yoDouble1->sub(randomNumber2);
   EXPECT_EQ(yoDouble1->getDoubleValue(), randomNumber1 - randomNumber2);
}

TEST_F(YoDoubleTest, testMultiplicationWithDoubles)
{
   double randomNumber1 = 0.44;
   double randomNumber2 = 0.9;

   yoDouble1->set(randomNumber1);
   yoDouble1->mul(randomNumber2);
   EXPECT_EQ(yoDouble1->getDoubleValue(), randomNumber1 * randomNumber2);
}

TEST_F(YoDoubleTest, testAdditionWithDoubleYoVariables)
{
   yoDouble1->set(0.3);
   yoDouble2->set(0.6);
   double expectedSum = yoDouble1->getDoubleValue() + yoDouble2->getDoubleValue();
   yoDouble1->add(*yoDouble2);
   EXPECT_EQ(yoDouble1->getDoubleValue(), expectedSum);
}

TEST_F(YoDoubleTest, testSubtractionWithDoubleYoVariables)
{
   yoDouble1->set(0.7);
   yoDouble2->set(0.2);
   double expectedDifference = yoDouble1->getDoubleValue() - yoDouble2->getDoubleValue();
   yoDouble1->sub(*yoDouble2);
   EXPECT_EQ(yoDouble1->getDoubleValue(), expectedDifference);
}

TEST_F(YoDoubleTest, testMultiplicationWithDoubleYoVariables)
{
   yoDouble1->set(0.5);
   yoDouble2->set(0.8);
   double expectedProduct = yoDouble1->getDoubleValue() * yoDouble2->getDoubleValue();
   yoDouble1->mul(*yoDouble2);
   EXPECT_EQ(yoDouble1->getDoubleValue(), expectedProduct);
}

TEST_F(YoDoubleTest, testValueEquals)
{
   yoDouble1->set(0.55);
   EXPECT_TRUE(yoDouble1->valueEquals(0.55));
}

TEST_F(YoDoubleTest, testGetAndSetMethods)
{
   yoDouble1->set(0.42);
   yoDouble2->set(yoDouble1->getDoubleValue());
   EXPECT_EQ(yoDouble1->getDoubleValue(), yoDouble2->getDoubleValue());
}

TEST_F(YoDoubleTest, testGetAndSetDoubleValue)
{
   yoDouble1->setValueFromDouble(0.42);
   yoDouble2->setValueFromDouble(yoDouble1->getValueAsDouble());
   EXPECT_EQ(yoDouble1->getValueAsDouble(), yoDouble2->getValueAsDouble());
}

TEST_F(YoDoubleTest, testSetFinal)
{
   yoDouble1->set(0.0);
   yoDouble1->set(0.0);

   yoDouble1->set(10.0);
   EXPECT_NEAR(yoDouble1->getDoubleValue(), 10.0, EPSILON);
}

TEST_F(YoDoubleTest, testGetYoVariableType)
{
   EXPECT_EQ(yoDouble1->getType(), variable::YoVariableType::DOUBLE);
}

TEST_F(YoDoubleTest, testDuplicate)
{
   registry::YoRegistry newRegistry("registry2000");
   double value = 0.31415;
   yoDouble2->set(value);
   std::unique_ptr<variable::YoVariable> duplicate = yoDouble2->duplicate(&newRegistry);
   EXPECT_NEAR(yoDouble2->getDoubleValue(), dynamic_cast<variable::YoDouble*>(duplicate.get())->getDoubleValue(), EPSILON);
}

TEST_F(YoDoubleTest, testProviderValue)
{
   yoDouble1->set(12509481.0);
   yoDouble2->set(2358);

   EXPECT_NEAR(yoDouble1->getDoubleValue(), yoDouble1->getValue(), 1e-9);
   EXPECT_NEAR(yoDouble2->getDoubleValue(), yoDouble2->getValue(), 1e-9);
}
} // namespace
