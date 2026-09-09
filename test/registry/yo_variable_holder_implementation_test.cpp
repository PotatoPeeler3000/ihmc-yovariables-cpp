#include <gtest/gtest.h>

#include <vector>

#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/registry/yo_variable_list.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::registry
{
namespace
{
class YoVariableHolderImplementationTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      yoVariableHolderImplementation = std::make_unique<YoVariableList>("Blop");

      robotRegistry = std::make_unique<YoRegistry>("robot");
      robot2Registry = std::make_unique<YoRegistry>("robot2");

      registryA = std::make_unique<YoRegistry>("registryA");
      registryB = std::make_unique<YoRegistry>("registryB");
      registryC = std::make_unique<YoRegistry>("registryC");
      robotRegistry->addChild(registryA.get());
      robotRegistry->addChild(registryB.get());
      robotRegistry->addChild(registryC.get());

      registryC2 = std::make_unique<YoRegistry>("registryC");
      robot2Registry->addChild(registryC2.get());

      addVariable("variableOne", registryA.get());
      addVariable("variableOne", registryB.get());
      addVariable("variableOne", registryC.get());
      addVariable("variableOne", registryC2.get());

      addVariable("variableTwo", registryA.get());
      addVariable("variableTwo", registryB.get());
      addVariable("variableTwo", registryC.get());
      addVariable("variableTwo", registryC2.get());

      addVariable("variableThree", registryA.get());
      addVariable("variableThree", registryB.get());
      addVariable("variableThree", registryC.get());
      addVariable("variableThree", registryC2.get());
   }

   void addVariable(const std::string& name, YoRegistry* registry)
   {
      variables.push_back(std::make_unique<variable::YoDouble>(name, registry));
      yoVariableHolderImplementation->add(variables.back().get());
   }

   std::unique_ptr<YoVariableList> yoVariableHolderImplementation;
   std::unique_ptr<YoRegistry> robotRegistry;
   std::unique_ptr<YoRegistry> robot2Registry;
   std::unique_ptr<YoRegistry> registryA;
   std::unique_ptr<YoRegistry> registryB;
   std::unique_ptr<YoRegistry> registryC;
   std::unique_ptr<YoRegistry> registryC2;
   std::vector<std::unique_ptr<variable::YoDouble>> variables;
};

TEST_F(YoVariableHolderImplementationTest, testGetVariable)
{
   variable::YoVariable* variable = yoVariableHolderImplementation->findVariable("robot.registryA.variableOne");
   EXPECT_EQ(variable->getName(), "variableOne");

   variable = yoVariableHolderImplementation->findVariable("registryA.variableOne");
   EXPECT_EQ(variable->getName(), "variableOne");

   variable = yoVariableHolderImplementation->findVariable("robot.registryA.variableOne");
   EXPECT_EQ(variable->getName(), "variableOne");

   variable = yoVariableHolderImplementation->findVariable("istryA.variableOne");
   EXPECT_EQ(variable, nullptr);

   variable = yoVariableHolderImplementation->findVariable("robot.registryA.variableTwo");
   EXPECT_EQ(variable->getName(), "variableTwo");
}

TEST_F(YoVariableHolderImplementationTest, testGetVariable1)
{
   variable::YoVariable* variable = yoVariableHolderImplementation->findVariable("robot.registryA", "variableOne");
   EXPECT_EQ(variable->getName(), "variableOne");
   EXPECT_EQ(variable->getFullNameString(), "robot.registryA.variableOne");

   variable = yoVariableHolderImplementation->findVariable("robot.registryB", "variableOne");
   EXPECT_EQ(variable->getName(), "variableOne");
   EXPECT_EQ(variable->getFullNameString(), "robot.registryB.variableOne");

   variable = yoVariableHolderImplementation->findVariable("robot.registryC", "variableOne");
   EXPECT_EQ(variable->getName(), "variableOne");
   EXPECT_EQ(variable->getFullNameString(), "robot.registryC.variableOne");

   variable = yoVariableHolderImplementation->findVariable("registryA", "variableOne");
   EXPECT_EQ(variable->getName(), "variableOne");
   EXPECT_EQ(variable->getFullNameString(), "robot.registryA.variableOne");

   variable = yoVariableHolderImplementation->findVariable("registryB", "variableTwo");
   EXPECT_EQ(variable->getName(), "variableTwo");
   EXPECT_EQ(variable->getFullNameString(), "robot.registryB.variableTwo");

   variable = yoVariableHolderImplementation->findVariable("registryC", "variableOne");
   EXPECT_EQ(variable->getFullNameString(), "robot.registryC.variableOne");

   variable = yoVariableHolderImplementation->findVariable("registryC", "variableTwo");
   EXPECT_EQ(variable->getFullNameString(), "robot.registryC.variableTwo");
}

TEST_F(YoVariableHolderImplementationTest, testGetVariables)
{
   EXPECT_EQ(yoVariableHolderImplementation->findVariables(YoNamespace("robot.registryA")).size(), 3u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables(YoNamespace("robot.registryB")).size(), 3u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables(YoNamespace("robot.registryC")).size(), 3u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables(YoNamespace("robot2.registryC")).size(), 3u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables(YoNamespace("robot")).size(), 0u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables(YoNamespace("registryA")).size(), 0u);
}

TEST_F(YoVariableHolderImplementationTest, testGetVariables1)
{
   std::vector<variable::YoVariable*> variablesFound = yoVariableHolderImplementation->findVariables("variableOne");
   bool aFound = false, bFound = false, cFound = false, c2Found = false;

   for (variable::YoVariable* variable : variablesFound)
   {
      if (variable->getFullNameString() == "robot.registryA.variableOne")
         aFound = true;
      if (variable->getFullNameString() == "robot.registryB.variableOne")
         bFound = true;
      if (variable->getFullNameString() == "robot.registryC.variableOne")
         cFound = true;
      if (variable->getFullNameString() == "robot2.registryC.variableOne")
         c2Found = true;
   }

   EXPECT_TRUE(aFound && bFound && cFound && c2Found);
   EXPECT_EQ(variablesFound.size(), 4u);

   EXPECT_EQ(yoVariableHolderImplementation->findVariables("variableTwo").size(), 4u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables("variableThree").size(), 4u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables("var").size(), 0u);
}

TEST_F(YoVariableHolderImplementationTest, testGetVariables2)
{
   EXPECT_EQ(yoVariableHolderImplementation->findVariables("robot.registryA", "variableOne").size(), 1u);
   EXPECT_EQ(yoVariableHolderImplementation->findVariables("robot", "variableOne").size(), 0u);

   std::vector<variable::YoVariable*> variablesFound = yoVariableHolderImplementation->findVariables("registryC", "variableOne");
   EXPECT_EQ(variablesFound.size(), 2u);
   bool cFound = false, c2Found = false;

   for (variable::YoVariable* variable : variablesFound)
   {
      if (variable->getFullNameString() == "robot.registryC.variableOne")
         cFound = true;
      if (variable->getFullNameString() == "robot2.registryC.variableOne")
         c2Found = true;
   }

   EXPECT_TRUE(cFound && c2Found);

   EXPECT_THROW(yoVariableHolderImplementation->findVariables("robot", "registryC.variableOne"), std::exception);
}

TEST_F(YoVariableHolderImplementationTest, testHasUniqueVariable)
{
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("variableOne"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("robot.registryA.variableOne"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("registryA.variableOne"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("robot.registryA.variableOne"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("istryA.variableOne"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("robot.registryA.variableTwo"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("registryC.variableTwo"));
}

TEST_F(YoVariableHolderImplementationTest, testHasUniqueVariable1)
{
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("robot.registryA", "variableOne"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("registryA", "variableOne"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("registryC", "variableTwo"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("istryA", "variableOne"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("robot.registryA", "variableTwo"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("robot", "variableOne"));

   EXPECT_THROW(yoVariableHolderImplementation->hasUniqueVariable("robot", "registryC.variableOne"), std::exception);
}
} // namespace
} // namespace ihmc::yovariables::registry
