#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "ihmc/yovariables/exceptions/illegal_name_exception.h"
#include "ihmc/yovariables/exceptions/name_collision_exception.h"
#include "ihmc/yovariables/listener/yo_variable_changed_listener.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"
#include "ihmc/yovariables/variable/yo_integer.h"

using namespace ihmc::yovariables;

namespace
{
enum class FooEnum
{
   ONE,
   TWO,
   THREE
};

class TestVariableChangedListener : public listener::YoVariableChangedListener
{
public:
   void changed(variable::YoVariable& source) override { lastVariableChanged_ = &source; }
   variable::YoVariable* getLastVariableChanged() const { return lastVariableChanged_; }
   void reset() { lastVariableChanged_ = nullptr; }

private:
   variable::YoVariable* lastVariableChanged_ = nullptr;
};

class YoVariableTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      robotRegistry = std::make_unique<registry::YoRegistry>("robot");
      registry = std::make_unique<registry::YoRegistry>("testRegistry");
      robotRegistry->addChild(registry.get());

      yoVariable = std::make_unique<variable::YoDouble>("variableOne", registry.get());
   }

   std::unique_ptr<registry::YoRegistry> robotRegistry;
   std::unique_ptr<registry::YoRegistry> registry;
   std::unique_ptr<variable::YoDouble> yoVariable;
   std::vector<std::unique_ptr<TestVariableChangedListener>> variableChangedListeners;

   void createVariableChangeListeners(int numberOfListeners)
   {
      for (int i = 0; i < numberOfListeners; i++)
         variableChangedListeners.push_back(std::make_unique<TestVariableChangedListener>());
   }

   void addAllListenersToYoVariable()
   {
      for (auto& observer : variableChangedListeners)
         yoVariable->addListener(observer.get());
   }

   void resetAllObservers()
   {
      for (auto& observer : variableChangedListeners)
         observer->reset();
   }
};

TEST_F(YoVariableTest, testValidVariable)
{
   variable::YoDouble unused("foobar", "", nullptr);
}

TEST_F(YoVariableTest, testCantHaveADot)
{
   EXPECT_THROW((variable::YoDouble("foo.bar", "", nullptr)), exceptions::IllegalNameException);
}

TEST_F(YoVariableTest, testCantHaveAComma)
{
   EXPECT_THROW((variable::YoDouble("foo,bar", "", nullptr)), exceptions::IllegalNameException);
}

TEST_F(YoVariableTest, testCantHaveACarrot)
{
   EXPECT_THROW((variable::YoDouble("foo^bar", "", nullptr)), exceptions::IllegalNameException);
}

TEST_F(YoVariableTest, testCanHaveAClosingBracket)
{
   variable::YoDouble unused("foo]bar", "", nullptr);
}

TEST_F(YoVariableTest, testCanHaveAnOpeningBracket)
{
   variable::YoDouble unused("foo[bar", "", nullptr);
}

TEST_F(YoVariableTest, testCantHaveABackSlash)
{
   EXPECT_THROW((variable::YoDouble("foo\\bar", "", nullptr)), exceptions::IllegalNameException);
}

TEST_F(YoVariableTest, testCantHaveAQuote)
{
   EXPECT_THROW((variable::YoDouble("foo\"bar", "", nullptr)), exceptions::IllegalNameException);
}

TEST_F(YoVariableTest, testCantHaveASpace)
{
   EXPECT_THROW((variable::YoDouble("foo bar", "", nullptr)), exceptions::IllegalNameException);
}

TEST_F(YoVariableTest, testCantHaveASlash)
{
   EXPECT_THROW((variable::YoDouble("foo/bar", "", nullptr)), exceptions::IllegalNameException);
}

TEST_F(YoVariableTest, testGetBooleanValue)
{
   variable::YoBoolean booleanVariable("booleanVar", registry.get());
   booleanVariable.set(true);

   EXPECT_TRUE(booleanVariable.getBooleanValue());
   booleanVariable.set(false);
   EXPECT_FALSE(booleanVariable.getBooleanValue());
}

TEST_F(YoVariableTest, testGetDescription)
{
   variable::YoDouble descrVariable("booleanVar", "Description", registry.get());

   EXPECT_EQ(descrVariable.getDescription(), "Description");
   // Java's assertNotNull(yoVariable.getDescription()) has no meaningful C++ equivalent here:
   // std::string has no null state, so getDescription() is always a valid (if possibly empty) string.
}

TEST_F(YoVariableTest, testGetDoubleValue)
{
   variable::YoDouble doubleVariable("doubleVar", registry.get());
   doubleVariable.set(15.6);

   EXPECT_NEAR(doubleVariable.getDoubleValue(), 15.6, 1e-7);
}

TEST_F(YoVariableTest, testGetEnumValue)
{
   variable::YoEnum<FooEnum> enumVariable("booleanVar", registry.get(), false);
   enumVariable.set(FooEnum::TWO);

   EXPECT_EQ(enumVariable.getEnumValue(), FooEnum::TWO);
   EXPECT_FALSE(enumVariable.getEnumValue() == FooEnum::ONE);
}

TEST_F(YoVariableTest, testGetFullNameWithNamespace)
{
   EXPECT_EQ(yoVariable->getFullName(), yoVariable->getRegistry()->getNamespace().append(yoVariable->getName()));
   EXPECT_EQ(yoVariable->getFullNameString(), "robot.testRegistry.variableOne");

   registry::YoRegistry newReg("newReg");
   newReg.addChild(yoVariable->getRegistry()->getRoot());

   EXPECT_EQ(yoVariable->getFullName(), yoVariable->getRegistry()->getNamespace().append(yoVariable->getName()));
   EXPECT_EQ(yoVariable->getFullNameString(), "newReg.robot.testRegistry.variableOne");
}

TEST_F(YoVariableTest, testGetIntegerValue)
{
   variable::YoInteger integerVariable("integerVariable", registry.get());
   integerVariable.set(5);

   EXPECT_EQ(integerVariable.getIntegerValue(), 5);
}

TEST_F(YoVariableTest, testGetName)
{
   EXPECT_EQ(yoVariable->getName(), "variableOne");
}

TEST_F(YoVariableTest, testGetYoRegistry)
{
   registry::YoRegistry* variableRegistry = yoVariable->getRegistry();
   ASSERT_NE(variableRegistry, nullptr);
   EXPECT_EQ(variableRegistry, registry.get());
   EXPECT_EQ(variableRegistry->findVariable(yoVariable->getName()), yoVariable.get());
}

TEST_F(YoVariableTest, testToString)
{
   // Adapted: this port's double formatting is a documented divergence from Java's
   // Double.toString() (see README.md), so only the name-prefix and int/bool formatting -
   // unaffected by that divergence - are asserted exactly here.
   variable::YoEnum<FooEnum> enumYoVariable("enumYoVariable", registry.get(), false);
   enumYoVariable.set(FooEnum::THREE);
   EXPECT_EQ(enumYoVariable.toString(), "enumYoVariable: THREE");

   variable::YoInteger intYoVariable("intYoVariable", registry.get());
   intYoVariable.set(1);
   EXPECT_EQ(intYoVariable.toString(), "intYoVariable: 1");

   variable::YoBoolean booleanYoBoolean("booleanYoVariable", registry.get());
   booleanYoBoolean.set(false);
   EXPECT_EQ(booleanYoBoolean.toString(), "booleanYoVariable: false");
}

TEST_F(YoVariableTest, testValueEquals)
{
   variable::YoBoolean booleanVariable("booleanVar", registry.get());
   variable::YoDouble doubleVariable("doubleVariable", registry.get());
   variable::YoInteger intVariable("intVariable", registry.get());
   variable::YoEnum<FooEnum> enumVariable("enumVariable", registry.get(), false);

   booleanVariable.set(true);
   doubleVariable.set(1.4);
   intVariable.set(7);
   enumVariable.set(FooEnum::THREE);

   EXPECT_TRUE(booleanVariable.valueEquals(true));
   EXPECT_TRUE(doubleVariable.valueEquals(1.4));
   EXPECT_TRUE(intVariable.valueEquals(7));
   EXPECT_TRUE(enumVariable.valueEquals(FooEnum::THREE));

   booleanVariable.set(false);
   doubleVariable.set(4.5);
   intVariable.set(9);
   enumVariable.set(FooEnum::TWO);

   EXPECT_TRUE(booleanVariable.valueEquals(false));
   EXPECT_TRUE(doubleVariable.valueEquals(4.5));
   EXPECT_TRUE(intVariable.valueEquals(9));
   EXPECT_TRUE(enumVariable.valueEquals(FooEnum::TWO));

   EXPECT_FALSE(booleanVariable.valueEquals(true));
}

TEST_F(YoVariableTest, testNameCollision)
{
   variable::YoBoolean booleanVariable("booleanVar", registry.get());
   variable::YoDouble doubleVariable("doubleVariable", registry.get());
   variable::YoInteger intVariable("intVariable", registry.get());
   variable::YoEnum<FooEnum> enumVariable("enumVariable", registry.get(), false);

   EXPECT_THROW((variable::YoBoolean("booleanVar", registry.get())), exceptions::NameCollisionException);
   EXPECT_THROW((variable::YoDouble("doubleVariable", registry.get())), exceptions::NameCollisionException);
   EXPECT_THROW((variable::YoInteger("intVariable", registry.get())), exceptions::NameCollisionException);
   EXPECT_THROW((variable::YoEnum<FooEnum>("enumVariable", registry.get(), false)), exceptions::NameCollisionException);
}

TEST_F(YoVariableTest, testNotifyVaribaleChangeListeners)
{
   createVariableChangeListeners(5);

   yoVariable->removeListeners();
   addAllListenersToYoVariable();

   TestVariableChangedListener hearNoEvil;

   for (auto& listenerInstance : variableChangedListeners)
      EXPECT_EQ(listenerInstance->getLastVariableChanged(), nullptr);

   EXPECT_EQ(hearNoEvil.getLastVariableChanged(), nullptr);

   yoVariable->notifyListeners();

   for (auto& observer : variableChangedListeners)
      EXPECT_EQ(observer->getLastVariableChanged(), yoVariable.get());

   EXPECT_EQ(hearNoEvil.getLastVariableChanged(), nullptr);
}

TEST_F(YoVariableTest, testAddVariableChangeListener)
{
   yoVariable->removeListeners();
   TestVariableChangedListener listenerInstance;
   yoVariable->addListener(&listenerInstance);
   yoVariable->removeListener(&listenerInstance);
}

TEST_F(YoVariableTest, testRemoveAllVariableChangeListeners)
{
   createVariableChangeListeners(5);
   addAllListenersToYoVariable();

   yoVariable->notifyListeners();

   for (auto& observer : variableChangedListeners)
      EXPECT_EQ(observer->getLastVariableChanged(), yoVariable.get());

   yoVariable->removeListeners();
   resetAllObservers();
   yoVariable->notifyListeners();

   for (auto& observer : variableChangedListeners)
      EXPECT_EQ(observer->getLastVariableChanged(), nullptr);
}

TEST_F(YoVariableTest, testRemoveObserver)
{
   createVariableChangeListeners(5);
   addAllListenersToYoVariable();

   yoVariable->notifyListeners();

   for (auto& observer : variableChangedListeners)
      EXPECT_EQ(observer->getLastVariableChanged(), yoVariable.get());

   for (auto& observer : variableChangedListeners)
      yoVariable->removeListener(observer.get());

   resetAllObservers();
   yoVariable->notifyListeners();

   for (auto& observer : variableChangedListeners)
      EXPECT_EQ(observer->getLastVariableChanged(), nullptr);
}

TEST_F(YoVariableTest, testRemoveObserverNonExistent1)
{
   TestVariableChangedListener listenerInstance;
   EXPECT_FALSE(yoVariable->removeListener(&listenerInstance));
}

TEST_F(YoVariableTest, testRemoveObserverNonExistent2)
{
   createVariableChangeListeners(5);
   addAllListenersToYoVariable();
   TestVariableChangedListener listenerInstance;
   EXPECT_FALSE(yoVariable->removeListener(&listenerInstance));
}

// testRecursiveCompareYoVariables is not ported: Java's YoVariableComparer relies on
// ReflectionToStringBuilder (field-by-field reflection), which has no equivalent here.

TEST_F(YoVariableTest, testDestroy)
{
   EXPECT_EQ(yoVariable->getRegistry(), registry.get());
   const std::vector<variable::YoVariable*>& variables = registry->getVariables();
   EXPECT_NE(std::find(variables.begin(), variables.end(), yoVariable.get()), variables.end());

   yoVariable->destroy();
   EXPECT_EQ(yoVariable->getRegistry(), nullptr);

   const std::vector<variable::YoVariable*>& variablesAfter = registry->getVariables();
   EXPECT_EQ(std::find(variablesAfter.begin(), variablesAfter.end(), yoVariable.get()), variablesAfter.end());
   EXPECT_EQ(yoVariable->getFullNameString(), yoVariable->getName());
}
} // namespace
