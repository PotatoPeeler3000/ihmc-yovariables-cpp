#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "ihmc/yovariables/exceptions/illegal_operation_exception.h"
#include "ihmc/yovariables/exceptions/name_collision_exception.h"
#include "ihmc/yovariables/listener/yo_registry_changed_listener.h"
#include "ihmc/yovariables/parameters/double_parameter.h"
#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/registry/yo_registry_restriction_level.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables
{
namespace
{
class TestRegistryChangedListener : public listener::YoRegistryChangedListener
{
public:
   void changed(const Change& change) override
   {
      if (change.wasVariableAdded())
         lastRegisteredVariable = change.getTargetVariable();
      if (change.wasRegistryAdded())
         lastAddedRegistry = change.getTargetRegistry();
      if (change.wasRegistryRemoved())
         lastRemovedRegistry = change.getTargetRegistry();
      if (change.wasCleared())
         lastClearedRegistry = change.getSource();
   }

   variable::YoVariable* lastRegisteredVariable = nullptr;
   registry::YoRegistry* lastAddedRegistry = nullptr;
   registry::YoRegistry* lastRemovedRegistry = nullptr;
   registry::YoRegistry* lastClearedRegistry = nullptr;
};

class YoRegistryTest : public ::testing::Test
{
protected:
   static constexpr int kNVarsInRoot = 4;

   void SetUp() override
   {
      robotRegistry = std::make_unique<registry::YoRegistry>("robot");
      controllerRegistry = std::make_unique<registry::YoRegistry>("controller");
      testRegistry = std::make_unique<registry::YoRegistry>("testRegistry");

      robotRegistry->addChild(controllerRegistry.get());
      controllerRegistry->addChild(testRegistry.get());

      robotVariable = std::make_unique<variable::YoDouble>("robotVariable", robotRegistry.get());
      controlVariable = std::make_unique<variable::YoDouble>("controlVariable", controllerRegistry.get());

      createAndAddNYoVariables(kNVarsInRoot, testRegistry.get());

      listener = std::make_unique<TestRegistryChangedListener>();
   }

   void createAndAddNYoVariables(int numberVariablesToAdd, registry::YoRegistry* target)
   {
      if (numberVariablesToAdd >= 1)
         extraVariables.push_back(std::make_unique<variable::YoDouble>("variableOne", target));
      if (numberVariablesToAdd >= 2)
         extraVariables.push_back(std::make_unique<variable::YoDouble>("variableTwo", target));
      if (numberVariablesToAdd >= 3)
         extraVariables.push_back(std::make_unique<variable::YoDouble>("variableThree", target));
      if (numberVariablesToAdd >= 4)
         extraVariables.push_back(std::make_unique<variable::YoDouble>("variableFour", target));
   }

   std::unique_ptr<registry::YoRegistry> robotRegistry;
   std::unique_ptr<registry::YoRegistry> controllerRegistry;
   std::unique_ptr<registry::YoRegistry> testRegistry;

   std::unique_ptr<TestRegistryChangedListener> listener;

   std::unique_ptr<variable::YoDouble> robotVariable;
   std::unique_ptr<variable::YoDouble> controlVariable;

   // Owns every variable created via createAndAddNYoVariables, and any other loose YoDoubles a test
   // needs to keep alive for the duration of the test (registries only hold non-owning pointers).
   std::vector<std::unique_ptr<variable::YoVariable>> extraVariables;
};

TEST_F(YoRegistryTest, testCantAddChildWithSameName)
{
   EXPECT_THROW(
      {
         registry::YoRegistry child1("sameName");
         registry::YoRegistry child2("sameName");

         testRegistry->addChild(&child1);
         testRegistry->addChild(&child2);
      },
      std::exception);
}

TEST_F(YoRegistryTest, testGetName)
{
   EXPECT_EQ(robotRegistry->getName(), "robot");
   EXPECT_EQ(controllerRegistry->getName(), "controller");
   EXPECT_EQ(testRegistry->getName(), "testRegistry");
}

TEST_F(YoRegistryTest, testGetAllVariables)
{
   EXPECT_EQ(testRegistry->collectSubtreeVariables().size(), 4u);
}

TEST_F(YoRegistryTest, testGetNamespace)
{
   registry::YoNamespace expectedReturn("robot.controller.testRegistry");
   EXPECT_EQ(expectedReturn, testRegistry->getNamespace());
}

TEST_F(YoRegistryTest, testGetVariable)
{
   variable::YoVariable* variableOne = testRegistry->findVariable("variableOne");
   variable::YoVariable* variableTwo = testRegistry->findVariable("variableTwo");
   variable::YoVariable* variableThree = testRegistry->findVariable("variableThree");
   variable::YoVariable* variableFour = testRegistry->findVariable("variableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   EXPECT_EQ(testRegistry->findVariable("fooy"), nullptr);

   variableOne = testRegistry->findVariable("robot.controller.testRegistry.variableOne");
   variableTwo = testRegistry->findVariable("robot.controller.testRegistry.variableTwo");
   variableThree = testRegistry->findVariable("robot.controller.testRegistry.variableThree");
   variableFour = testRegistry->findVariable("robot.controller.testRegistry.variableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   variableOne = testRegistry->findVariable("testRegistry.variableOne");
   variableTwo = testRegistry->findVariable("controller.testRegistry.variableTwo");
   variableThree = testRegistry->findVariable("testRegistry.variableThree");
   variableFour = testRegistry->findVariable("controller.testRegistry.variableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   variableOne = testRegistry->findVariable("robot.controller.variableOne");
   variableTwo = testRegistry->findVariable("robot.testRegistry.variableTwo");
   variableThree = testRegistry->findVariable("bot.controller.testRegistry.variableThree");
   variableFour = testRegistry->findVariable("robot.controller.testRegis.variableFour");

   EXPECT_EQ(variableOne, nullptr);
   EXPECT_EQ(variableTwo, nullptr);
   EXPECT_EQ(variableThree, nullptr);
   EXPECT_EQ(variableFour, nullptr);
}

TEST_F(YoRegistryTest, testCaseInsensitivityToNameButNotNamespace)
{
   variable::YoVariable* variableOne = testRegistry->findVariable("variableone");
   variable::YoVariable* variableTwo = testRegistry->findVariable("variableTWO");
   variable::YoVariable* variableThree = testRegistry->findVariable("VAriableThree");
   variable::YoVariable* variableFour = testRegistry->findVariable("variableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   variableOne = testRegistry->findVariable("robot.controller.testRegistry.variableONE");
   variableTwo = testRegistry->findVariable("robot.controller.testRegistry.variableTwo");
   variableThree = testRegistry->findVariable("robot.controller.testRegistry.variableTHREe");
   variableFour = testRegistry->findVariable("robot.controller.testRegistry.VAriableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   variableOne = testRegistry->findVariable("testRegistry.variableONE");
   variableTwo = testRegistry->findVariable("controller.testRegistry.variableTWO");
   variableThree = testRegistry->findVariable("testRegistry.variableThREE");
   variableFour = testRegistry->findVariable("controller.testRegistry.VAriableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   variableOne = testRegistry->findVariable("Robot.controller.testRegistry.variableOne");
   variableTwo = testRegistry->findVariable("robot.coNtroller.testRegistry.variableTwo");
   variableThree = testRegistry->findVariable("robot.controller.TestRegistry.variableThree");
   variableFour = testRegistry->findVariable("robot.controller.testRegistrY.variableFour");

   EXPECT_EQ(variableOne, nullptr);
   EXPECT_EQ(variableTwo, nullptr);
   EXPECT_EQ(variableThree, nullptr);
   EXPECT_EQ(variableFour, nullptr);
}

TEST_F(YoRegistryTest, testGetVariable1)
{
   std::string ns = "robot.controller.testRegistry";

   variable::YoVariable* variableOne = testRegistry->findVariable(ns, "variableOne");
   variable::YoVariable* variableTwo = testRegistry->findVariable(ns, "variableTwo");
   variable::YoVariable* variableThree = testRegistry->findVariable(ns, "variableThree");
   variable::YoVariable* variableFour = testRegistry->findVariable(ns, "variableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   ns = "controller.testRegistry";

   variableOne = testRegistry->findVariable(ns, "variableOne");
   variableTwo = testRegistry->findVariable(ns, "variableTwo");
   variableThree = testRegistry->findVariable(ns, "variableThree");
   variableFour = testRegistry->findVariable(ns, "variableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   ns = "testRegistry";

   variableOne = testRegistry->findVariable(ns, "variableOne");
   variableTwo = testRegistry->findVariable(ns, "variableTwo");
   variableThree = testRegistry->findVariable(ns, "variableThree");
   variableFour = testRegistry->findVariable(ns, "variableFour");

   EXPECT_EQ(variableOne->getName(), "variableOne");
   EXPECT_EQ(variableTwo->getName(), "variableTwo");
   EXPECT_EQ(variableThree->getName(), "variableThree");
   EXPECT_EQ(variableFour->getName(), "variableFour");

   ns = ".testRegistry";

   variableOne = testRegistry->findVariable(ns, "variableOne");
   variableTwo = testRegistry->findVariable(ns, "variableTwo");
   variableThree = testRegistry->findVariable(ns, "variableThree");
   variableFour = testRegistry->findVariable(ns, "variableFour");

   EXPECT_EQ(variableOne, nullptr);
   EXPECT_EQ(variableTwo, nullptr);
   EXPECT_EQ(variableThree, nullptr);
   EXPECT_EQ(variableFour, nullptr);

   EXPECT_THROW(testRegistry->findVariable(ns, "foo.variableOne"), std::exception);
}

TEST_F(YoRegistryTest, testGetVariables1)
{
   EXPECT_EQ(testRegistry->findVariables("variableOne").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("variableTwo").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("variableThree").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("variableFour").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("variable").size(), 0u);
   EXPECT_EQ(testRegistry->findVariables("robot.controller.testRegistry.variableOne").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("controller.testRegistry.variableOne").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("testRegistry.variableOne").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("estRegistry.variableOne").size(), 0u);
   EXPECT_EQ(testRegistry->findVariables("foo.robot.controller.testRegistry.variableOne").size(), 0u);
}

TEST_F(YoRegistryTest, testGetVariables2)
{
   EXPECT_EQ(testRegistry->findVariables("robot.controller.testRegistry", "variableOne").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("robot.controller.testRegistry", "variableTwo").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("robot.controller.testRegistry", "variableThree").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("robot.controller.testRegistry", "variableFour").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("robot.controller.testRegistry", "variable").size(), 0u);
   EXPECT_EQ(testRegistry->findVariables("controller.testRegistry", "variableOne").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("testRegistry", "variableOne").size(), 1u);
   EXPECT_EQ(testRegistry->findVariables("estRegistry", "variableOne").size(), 0u);
   EXPECT_EQ(testRegistry->findVariables("foo.robot.controller.testRegistry", "variableOne").size(), 0u);

   EXPECT_THROW(testRegistry->findVariables("robot.controller.testRegistry", "robot.controller.testRegistry.variableOne"), std::exception);
}

TEST_F(YoRegistryTest, testHasUniqueVariable)
{
   EXPECT_FALSE(testRegistry->hasUniqueVariable(""));

   EXPECT_TRUE(testRegistry->hasUniqueVariable("variableOne"));
   EXPECT_FALSE(testRegistry->hasUniqueVariable("dontHaveMeVariable"));

   EXPECT_TRUE(testRegistry->hasUniqueVariable("robot.controller.testRegistry", "variableTwo"));
   EXPECT_TRUE(testRegistry->hasUniqueVariable("controller.testRegistry", "variableTwo"));
   EXPECT_FALSE(testRegistry->hasUniqueVariable("robot.controller", "variableTwo"));
}

TEST_F(YoRegistryTest, testHasUniqueVariable1)
{
   EXPECT_FALSE(testRegistry->hasUniqueVariable("", ""));
}

TEST_F(YoRegistryTest, testRegisterVariable)
{
   // Java's addVariable(null) throwing NullPointerException is not ported: it relies on a null
   // reference being a catchable exception, whereas dereferencing a null YoVariable* here is
   // undefined behavior, not a recoverable error - there's no meaningful C++ equivalent to test.

   variable::YoDouble variableFive("variableFive", nullptr);
   testRegistry->addVariable(&variableFive);

   EXPECT_TRUE(testRegistry->hasUniqueVariable("variableFive"));
}

TEST_F(YoRegistryTest, testCannotRegisterSameVariableName)
{
   EXPECT_THROW(
      {
         variable::YoDouble variableFiveOnce("variableFive", nullptr);
         variable::YoDouble variableFiveTwice("variableFive", nullptr);

         testRegistry->addVariable(&variableFiveOnce);
         EXPECT_TRUE(testRegistry->hasUniqueVariable("variableFive"));
         testRegistry->addVariable(&variableFiveTwice);
      },
      exceptions::NameCollisionException);
}

TEST_F(YoRegistryTest, testGetYoVariables)
{
   EXPECT_EQ(robotRegistry->getVariables().size(), 1u);
   EXPECT_EQ(controllerRegistry->getVariables().size(), 1u);
   EXPECT_EQ(testRegistry->getVariables().size(), 4u);
}

TEST_F(YoRegistryTest, testAddChildAndGetParentAndGetChildren)
{
   EXPECT_EQ(robotRegistry->getParent(), nullptr);

   registry::YoRegistry childOne("childOne");
   EXPECT_EQ(childOne.getParent(), nullptr);

   testRegistry->addChild(&childOne);

   EXPECT_THROW(
      {
         registry::YoRegistry childOneRepeat("childOne");
         testRegistry->addChild(&childOneRepeat);
      },
      std::exception);
}

TEST_F(YoRegistryTest, testDontLetAChildGetAddedToTwoRegistries)
{
   EXPECT_THROW(
      {
         registry::YoRegistry root1("root1");
         root1.setRestrictionLevel(registry::YoRegistryRestrictionLevel::RESTRICTED);
         registry::YoRegistry root2("root2");

         registry::YoRegistry child("child");
         root1.addChild(&child);
         root2.addChild(&child);
      },
      std::exception);
}

TEST_F(YoRegistryTest, testIllegalName1)
{
   std::string illegalName = "foo..foo";
   bool runtimeExceptionThrown = false;
   try
   {
      testRegistry = std::make_unique<registry::YoRegistry>(illegalName);
   }
   catch (const std::exception& e)
   {
      EXPECT_NE(std::string(e.what()).find(illegalName), std::string::npos);
      runtimeExceptionThrown = true;
   }

   EXPECT_TRUE(runtimeExceptionThrown);
}

TEST_F(YoRegistryTest, testIllegalName2)
{
   EXPECT_THROW(testRegistry = std::make_unique<registry::YoRegistry>("foo."), std::exception);
}

TEST_F(YoRegistryTest, testNoDotsAllowed)
{
   EXPECT_THROW(testRegistry = std::make_unique<registry::YoRegistry>("foo.bar"), std::exception);
}

TEST_F(YoRegistryTest, testIllegalAddChild)
{
   EXPECT_THROW(
      {
         registry::YoRegistry childOne("childOne");
         childOne.addChild(&childOne);
      },
      exceptions::IllegalOperationException);
}

TEST_F(YoRegistryTest, testGetAllVariablesIncludingDescendants)
{
   registry::YoRegistry childOne("childOne");
   int nVarsChildOne = 3;
   createAndAddNYoVariables(nVarsChildOne, &childOne);

   registry::YoRegistry childTwo("childTwo");
   int nVarsChildTwo = 2;
   createAndAddNYoVariables(nVarsChildTwo, &childTwo);

   testRegistry->addChild(&childOne);
   testRegistry->addChild(&childTwo);

   int nVarsExpected = nVarsChildOne + nVarsChildTwo + kNVarsInRoot;

   EXPECT_EQ(testRegistry->collectSubtreeVariables().size(), static_cast<std::size_t>(nVarsExpected));
   EXPECT_EQ(childTwo.collectSubtreeVariables().size(), static_cast<std::size_t>(nVarsChildTwo));
}

TEST_F(YoRegistryTest, testFamilyRelations)
{
   registry::YoRegistry childOne("childOne");
   registry::YoRegistry childTwo("childTwo");

   testRegistry->addChild(&childOne);
   testRegistry->addChild(&childTwo);

   EXPECT_EQ(childOne.getParent(), testRegistry.get());
   EXPECT_EQ(childTwo.getParent(), testRegistry.get());

   const std::vector<registry::YoRegistry*>& children = testRegistry->getChildRegistries();

   EXPECT_EQ(children.size(), 2u);
   EXPECT_NE(std::find(children.begin(), children.end(), &childOne), children.end());
   EXPECT_NE(std::find(children.begin(), children.end(), &childTwo), children.end());
}

TEST_F(YoRegistryTest, testCantAddDuplicateSubnames)
{
   EXPECT_THROW(
      {
         registry::YoRegistry childOne("childOne");
         testRegistry->addChild(&childOne);

         registry::YoRegistry grandChildOne(childOne.getParent()->getNamespace().getRootName());
         childOne.addChild(&grandChildOne);
      },
      std::exception);
}

TEST_F(YoRegistryTest, testNullChild)
{
   registry::YoRegistry testNullChild("TestNullChild");

   testNullChild.addChild(nullptr);
   EXPECT_EQ(testNullChild.getChildRegistries().size(), 0u);
}

TEST_F(YoRegistryTest, testRegistryTree)
{
   registry::YoRegistry root("root");

   registry::YoRegistry registry0("registry0");
   EXPECT_EQ(registry0.getNamespace().getName(), "registry0");
   registry::YoRegistry registry1("registry1");
   registry::YoRegistry registry2("registry2");
   root.addChild(&registry0);
   EXPECT_EQ(registry0.getNamespace().getName(), "root.registry0");
   EXPECT_EQ(registry0.getNamespace().getShortName(), "registry0");
   root.addChild(&registry2);

   registry::YoRegistry registry00("registry00");
   registry::YoRegistry registry01("registry01");
   registry0.addChild(&registry00);
   registry0.addChild(&registry01);

   registry::YoRegistry registry10("registry10");
   registry1.addChild(&registry10);

   registry::YoRegistry registry010("registry010");
   registry::YoRegistry registry011("registry011");
   registry01.addChild(&registry010);

   variable::YoDouble variable0_A("variable0_A", &registry0);
   variable::YoDouble variable0_B("variable0_B", &registry0);
   variable::YoDouble variable10_A("variable10_A", &registry10);
   variable::YoDouble variable011_A("variable011_A", &registry011);

   variable::YoDouble repeatedVariable_root("repeatedVariable", &root);
   variable::YoDouble repeatedVariable_registry0("repeatedVariable", &registry0);
   variable::YoDouble repeatedVariable_registry01("repeatedVariable", &registry01);
   variable::YoDouble repeatedVariable_registry010("repeatedVariable", &registry010);

   // Do some of the addChilds out of order to make sure they work correctly when done out of order.
   root.addChild(&registry1);
   registry01.addChild(&registry011);
   EXPECT_EQ(registry011.getNamespace().getName(), "root.registry0.registry01.registry011");

   EXPECT_EQ(variable0_A.getFullNameString(), "root.registry0.variable0_A");
   EXPECT_EQ(variable0_B.getFullNameString(), "root.registry0.variable0_B");
   EXPECT_EQ(variable10_A.getFullNameString(), "root.registry1.registry10.variable10_A");
   EXPECT_EQ(variable011_A.getFullNameString(), "root.registry0.registry01.registry011.variable011_A");

   EXPECT_EQ(root.collectSubtreeVariables().size(), 8u);

   EXPECT_TRUE(registry10.hasUniqueVariable("root.registry1.registry10.variable10_A"));

   EXPECT_EQ(&variable10_A, registry10.findVariable("root.registry1.registry10.variable10_A"));
   EXPECT_EQ(&variable10_A, registry10.findVariable("registry10.variable10_A"));
   EXPECT_EQ(&variable10_A, registry10.findVariable("variable10_A"));

   EXPECT_TRUE(root.hasUniqueVariable("root.registry1.registry10.variable10_A"));
   EXPECT_TRUE(root.hasUniqueVariable("registry1.registry10.variable10_A"));
   EXPECT_TRUE(root.hasUniqueVariable("registry10.variable10_A"));
   EXPECT_TRUE(root.hasUniqueVariable("variable10_A"));

   EXPECT_FALSE(root.hasUniqueVariable("repeatedVariable"));
   EXPECT_FALSE(registry0.hasUniqueVariable("repeatedVariable"));
   EXPECT_FALSE(registry01.hasUniqueVariable("repeatedVariable"));
   EXPECT_TRUE(registry010.hasUniqueVariable("repeatedVariable"));

   EXPECT_TRUE(root.hasUniqueVariable("registry0.repeatedVariable"));
   EXPECT_TRUE(root.hasUniqueVariable("registry0", "repeatedVariable"));
   EXPECT_FALSE(root.hasUniqueVariable("registry0.noWay"));
   EXPECT_FALSE(root.hasUniqueVariable("noWay.repeatedVariable"));
   EXPECT_FALSE(root.hasUniqueVariable("noWay", "repeatedVariable"));

   EXPECT_EQ(&variable10_A, registry1.findVariable("variable10_A"));
   EXPECT_EQ(&variable10_A, root.findVariable("variable10_A"));
   EXPECT_EQ(&variable011_A, root.findVariable("variable011_A"));
   EXPECT_EQ(&variable011_A, registry0.findVariable("variable011_A"));
   EXPECT_EQ(&variable011_A, registry01.findVariable("variable011_A"));
   EXPECT_EQ(&variable011_A, registry011.findVariable("variable011_A"));

   EXPECT_EQ(&repeatedVariable_root, root.findVariable("repeatedVariable"));
   EXPECT_EQ(&repeatedVariable_registry0, registry0.findVariable("repeatedVariable"));
   EXPECT_EQ(&repeatedVariable_registry01, registry01.findVariable("repeatedVariable"));
   EXPECT_EQ(&repeatedVariable_registry010, registry010.findVariable("repeatedVariable"));

   EXPECT_EQ(root.findVariables("repeatedVariable").size(), 4u);
}

TEST_F(YoRegistryTest, testDontAllowRepeatRegistryNames)
{
   EXPECT_THROW(
      {
         registry::YoRegistry root("root");
         registry::YoRegistry levelOne("levelOne");
         registry::YoRegistry registryOne("registryOne");
         registry::YoRegistry registryOneRepeat("registryOne");

         root.addChild(&levelOne);

         levelOne.addChild(&registryOne);
         levelOne.addChild(&registryOneRepeat);
      },
      std::exception);
}

TEST_F(YoRegistryTest, testCantAddAChildWithANullNamespace)
{
   EXPECT_THROW(
      {
         registry::YoRegistry root("root");
         registry::YoRegistry child("");
         root.addChild(&child);
      },
      std::exception);
}

TEST_F(YoRegistryTest, testGetAllRegistriesIncludingChildren)
{
   std::vector<registry::YoRegistry*> registries = robotRegistry->collectSubtreeRegistries();

   EXPECT_EQ(registries.size(), 3u);
   EXPECT_NE(std::find(registries.begin(), registries.end(), robotRegistry.get()), registries.end());
   EXPECT_NE(std::find(registries.begin(), registries.end(), controllerRegistry.get()), registries.end());
   EXPECT_NE(std::find(registries.begin(), registries.end(), testRegistry.get()), registries.end());
}

TEST_F(YoRegistryTest, testGetRegistry)
{
   EXPECT_EQ(robotRegistry.get(), robotRegistry->findRegistry(registry::YoNamespace("robot")));
   EXPECT_EQ(controllerRegistry.get(), robotRegistry->findRegistry(registry::YoNamespace("robot.controller")));
   EXPECT_EQ(testRegistry.get(), robotRegistry->findRegistry(registry::YoNamespace("robot.controller.testRegistry")));

   EXPECT_EQ(controllerRegistry.get(), controllerRegistry->findRegistry(registry::YoNamespace("robot.controller")));
   EXPECT_EQ(testRegistry.get(), controllerRegistry->findRegistry(registry::YoNamespace("robot.controller.testRegistry")));

   EXPECT_EQ(testRegistry.get(), testRegistry->findRegistry(registry::YoNamespace("robot.controller.testRegistry")));

   EXPECT_EQ(testRegistry.get(), robotRegistry->findRegistry(registry::YoNamespace("testRegistry")));
   EXPECT_EQ(testRegistry.get(), robotRegistry->findRegistry(registry::YoNamespace("controller.testRegistry")));

   EXPECT_NE(robotRegistry.get(), controllerRegistry->findRegistry(registry::YoNamespace("robot")));
}

TEST_F(YoRegistryTest, testGetNumberOfVariables)
{
   EXPECT_EQ(robotRegistry->getNumberOfVariables(), 1u);
   EXPECT_EQ(controllerRegistry->getNumberOfVariables(), 1u);
   EXPECT_EQ(testRegistry->getNumberOfVariables(), 4u);
}

TEST_F(YoRegistryTest, testListenersOne)
{
   robotRegistry->addListener(listener.get());

   EXPECT_EQ(listener->lastRegisteredVariable, nullptr);
   variable::YoDouble addedYoVariable("addedLater", controllerRegistry.get());
   EXPECT_EQ(listener->lastRegisteredVariable, &addedYoVariable);

   EXPECT_EQ(listener->lastAddedRegistry, nullptr);
   registry::YoRegistry addedRegistry("addedRegistry");
   testRegistry->addChild(&addedRegistry);
   EXPECT_EQ(listener->lastAddedRegistry, &addedRegistry);

   testRegistry->addListener(listener.get());
   EXPECT_EQ(listener->lastClearedRegistry, nullptr);
   EXPECT_EQ(listener->lastRemovedRegistry, nullptr);
   testRegistry->destroy();
   EXPECT_EQ(listener->lastRemovedRegistry, testRegistry.get());
   EXPECT_EQ(listener->lastClearedRegistry, testRegistry.get());
}

TEST_F(YoRegistryTest, testAreEqual)
{
   registry::YoRegistry robotRegistryClone("robot");
   registry::YoRegistry controllerRegistryClone("controller");
   registry::YoRegistry testRegistryClone("testRegistry");

   robotRegistryClone.addChild(&controllerRegistryClone);
   controllerRegistryClone.addChild(&testRegistryClone);

   variable::YoDouble robotVariableClone("robotVariable", &robotRegistryClone);
   variable::YoDouble controlVariableClone("controlVariable", &controllerRegistryClone);

   createAndAddNYoVariables(kNVarsInRoot, &testRegistryClone);

   EXPECT_TRUE(*robotRegistry == robotRegistryClone);
}

TEST_F(YoRegistryTest, testClear)
{
   EXPECT_FALSE(robotRegistry->collectSubtreeVariables().empty());
   EXPECT_FALSE(robotRegistry->getChildRegistries().empty());

   robotRegistry->destroy();

   EXPECT_TRUE(robotRegistry->collectSubtreeVariables().empty());
   EXPECT_TRUE(robotRegistry->getChildRegistries().empty());
}

TEST_F(YoRegistryTest, testGetVariables)
{
   registry::YoNamespace robotNamespace("robot");
   std::vector<variable::YoVariable*> robotVariables = robotRegistry->findVariables(robotNamespace);

   EXPECT_EQ(robotVariables.size(), 1u);
   EXPECT_NE(std::find(robotVariables.begin(), robotVariables.end(), robotVariable.get()), robotVariables.end());
   EXPECT_EQ(std::find(robotVariables.begin(), robotVariables.end(), controlVariable.get()), robotVariables.end());

   registry::YoNamespace controllerNamespace("robot.controller");
   std::vector<variable::YoVariable*> controllerVariables = robotRegistry->findVariables(controllerNamespace);

   EXPECT_EQ(controllerVariables.size(), 1u);
   EXPECT_EQ(std::find(controllerVariables.begin(), controllerVariables.end(), robotVariable.get()), controllerVariables.end());
   EXPECT_NE(std::find(controllerVariables.begin(), controllerVariables.end(), controlVariable.get()), controllerVariables.end());
}

TEST_F(YoRegistryTest, testParameters)
{
   registry::YoRegistry a("a");
   registry::YoRegistry aa("aa");
   registry::YoRegistry ab("ab");
   registry::YoRegistry aaa("aaa");
   registry::YoRegistry aab("aab");

   parameters::DoubleParameter parameter1("parameter1", &aa);
   parameters::DoubleParameter parameter2("parameter2", &aaa);
   parameters::DoubleParameter parameter3("parameter3", &aaa);

   variable::YoDouble double1("double1", &a);
   variable::YoDouble double2("double2", &aa);
   variable::YoDouble double3("double3", &ab);
   variable::YoDouble double4("double4", &aaa);
   variable::YoDouble double5("double5", &aab);

   a.addChild(&aa);
   a.addChild(&ab);
   aa.addChild(&aaa);
   aa.addChild(&aab);

   EXPECT_FALSE(a.hasParameters());
   EXPECT_TRUE(a.hasParametersDeep());
   EXPECT_TRUE(aa.hasParameters());
   EXPECT_TRUE(aaa.hasParameters());
   EXPECT_FALSE(ab.hasParameters());
   EXPECT_FALSE(aab.hasParameters());

   EXPECT_EQ(a.collectSubtreeParameters().size(), 3u);
   EXPECT_EQ(aa.collectSubtreeParameters().size(), 3u);
   EXPECT_EQ(aaa.collectSubtreeParameters().size(), 2u);
   EXPECT_EQ(ab.collectSubtreeParameters().size(), 0u);
   EXPECT_EQ(aab.collectSubtreeParameters().size(), 0u);

   EXPECT_EQ(aa.getParameters().size(), 1u);
   EXPECT_EQ(aaa.getParameters().size(), 2u);
   EXPECT_EQ(a.getParameters().size(), 0u);
   EXPECT_EQ(ab.getParameters().size(), 0u);
   EXPECT_EQ(aab.getParameters().size(), 0u);

   std::vector<parameters::YoParameter*> aParams = a.collectSubtreeParameters();
   EXPECT_NE(std::find(aParams.begin(), aParams.end(), &parameter1), aParams.end());
   EXPECT_NE(std::find(aParams.begin(), aParams.end(), &parameter2), aParams.end());
   EXPECT_NE(std::find(aParams.begin(), aParams.end(), &parameter3), aParams.end());

   const std::vector<parameters::YoParameter*>& aaParams = aa.getParameters();
   EXPECT_NE(std::find(aaParams.begin(), aaParams.end(), &parameter1), aaParams.end());
   const std::vector<parameters::YoParameter*>& aaaParams = aaa.getParameters();
   EXPECT_NE(std::find(aaaParams.begin(), aaaParams.end(), &parameter2), aaaParams.end());
   EXPECT_NE(std::find(aaaParams.begin(), aaaParams.end(), &parameter3), aaaParams.end());
}

TEST_F(YoRegistryTest, testRemoveVariable)
{
   EXPECT_TRUE(robotVariable->getRegistry() == robotRegistry.get());
   std::vector<variable::YoVariable*> robotVars = robotRegistry->getVariables();
   EXPECT_NE(std::find(robotVars.begin(), robotVars.end(), robotVariable.get()), robotVars.end());

   controllerRegistry->removeVariable(robotVariable.get());
   EXPECT_TRUE(robotVariable->getRegistry() == robotRegistry.get());
   robotVars = robotRegistry->getVariables();
   EXPECT_NE(std::find(robotVars.begin(), robotVars.end(), robotVariable.get()), robotVars.end());

   robotRegistry->removeVariable(robotVariable.get());
   EXPECT_EQ(robotVariable->getRegistry(), nullptr);
   robotVars = robotRegistry->getVariables();
   EXPECT_EQ(std::find(robotVars.begin(), robotVars.end(), robotVariable.get()), robotVars.end());
   EXPECT_EQ(robotVariable->getFullNameString(), robotVariable->getName());
}

// Guards the assumption that YoRegistry::getVariables()/getChildRegistries() return their contents
// in insertion order rather than in whatever order an internal lookup structure (e.g. a
// name-to-instance map) happens to iterate. Names are added in an order that is neither
// alphabetical nor likely to match typical hash bucket ordering, so a regression backed by an
// unordered map would very likely be caught.
TEST_F(YoRegistryTest, testGetVariablesAndGetChildrenPreserveInsertionOrder)
{
   registry::YoRegistry orderRoot("orderRoot");

   std::vector<std::string> variableNames{"zebra", "mango", "apple", "banana", "kiwi", "fig"};
   std::vector<std::unique_ptr<variable::YoDouble>> variables;
   for (const std::string& name : variableNames)
      variables.push_back(std::make_unique<variable::YoDouble>(name, &orderRoot));

   std::vector<variable::YoVariable*> actualVariables = orderRoot.getVariables();
   ASSERT_EQ(actualVariables.size(), variableNames.size());
   for (std::size_t i = 0; i < variableNames.size(); i++)
      EXPECT_EQ(actualVariables[i], variables[i].get()) << "Variable at index " << i << " was out of insertion order.";

   std::vector<std::string> childNames{"delta", "alpha", "charlie", "echo", "bravo"};
   std::vector<std::unique_ptr<registry::YoRegistry>> children;
   for (const std::string& name : childNames)
   {
      children.push_back(std::make_unique<registry::YoRegistry>(name));
      orderRoot.addChild(children.back().get());
   }

   const std::vector<registry::YoRegistry*>& actualChildren = orderRoot.getChildRegistries();
   ASSERT_EQ(actualChildren.size(), childNames.size());
   for (std::size_t i = 0; i < childNames.size(); i++)
      EXPECT_EQ(actualChildren[i], children[i].get()) << "Child at index " << i << " was out of insertion order.";

   // Removing from the middle should not reshuffle the remaining entries.
   orderRoot.removeVariable(variables[2].get()); // "apple"
   std::vector<variable::YoVariable*> afterRemoval = orderRoot.getVariables();
   ASSERT_EQ(afterRemoval.size(), variableNames.size() - 1);
   EXPECT_EQ(afterRemoval[0], variables[0].get());
   EXPECT_EQ(afterRemoval[1], variables[1].get());
   EXPECT_EQ(afterRemoval[2], variables[3].get());
   EXPECT_EQ(afterRemoval[3], variables[4].get());
   EXPECT_EQ(afterRemoval[4], variables[5].get());

   // A variable added after a removal should land at the end, not in the vacated slot.
   variable::YoDouble reAdded("apple", &orderRoot);
   std::vector<variable::YoVariable*> afterReAdd = orderRoot.getVariables();
   ASSERT_EQ(afterReAdd.size(), variableNames.size());
   EXPECT_EQ(afterReAdd.back(), &reAdded);
}
} // namespace
} // namespace ihmc::yovariables
