#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/registry/yo_variable_list.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"
#include "ihmc/yovariables/variable/yo_integer.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::registry
{
namespace
{
enum class EnumYoVariableTestEnums
{
   ONE,
   TWO
};

class YoVariableHolderImplementationNewTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      yoVariableHolderImplementation = std::make_unique<YoVariableList>("Blop");

      testVariables.push_back(std::make_unique<variable::YoDouble>("yoDouble", nullptr));
      testVariables.push_back(std::make_unique<variable::YoBoolean>("yoBoolean", nullptr));
      testVariables.push_back(std::make_unique<variable::YoInteger>("yoInteger", nullptr));
      testVariables.push_back(std::make_unique<variable::YoEnum<EnumYoVariableTestEnums>>("yoEnum", nullptr));
   }

   std::vector<variable::YoVariable*> testVariablePointers() const
   {
      std::vector<variable::YoVariable*> pointers;
      for (const std::unique_ptr<variable::YoVariable>& variable : testVariables)
         pointers.push_back(variable.get());
      return pointers;
   }

   std::unique_ptr<YoVariableList> yoVariableHolderImplementation;
   std::vector<std::unique_ptr<variable::YoVariable>> testVariables;
};

TEST_F(YoVariableHolderImplementationNewTest, testAddSingleYoVariableToHolderAndGetVariableByName)
{
   variable::YoDouble* yoDoubleFromArrayList = dynamic_cast<variable::YoDouble*>(testVariables[0].get());
   yoVariableHolderImplementation->add(yoDoubleFromArrayList);
   EXPECT_EQ(yoDoubleFromArrayList, yoVariableHolderImplementation->findVariable("yoDouble"));
}

TEST_F(YoVariableHolderImplementationNewTest, testAddMultipleYoVariablesToHolderAndGetAllVariables)
{
   std::vector<variable::YoVariable*> pointers = testVariablePointers();
   yoVariableHolderImplementation->addAll(pointers);

   for (variable::YoVariable* var : yoVariableHolderImplementation->getVariables())
      EXPECT_NE(std::find(pointers.begin(), pointers.end(), var), pointers.end());
}

TEST_F(YoVariableHolderImplementationNewTest, testGetVariableUsingFullNamespace)
{
   yoVariableHolderImplementation->addAll(testVariablePointers());
   EXPECT_TRUE(testVariables[0].get() == yoVariableHolderImplementation->findVariable("yoDouble"));
}

TEST_F(YoVariableHolderImplementationNewTest, testGetVariable)
{
   yoVariableHolderImplementation->addAll(testVariablePointers());
   EXPECT_TRUE(testVariables[0].get() == yoVariableHolderImplementation->findVariable("yoDouble"));
}

TEST_F(YoVariableHolderImplementationNewTest, testGetVariableCaseInsensitive)
{
   yoVariableHolderImplementation->addAll(testVariablePointers());
   variable::YoVariable* variable = yoVariableHolderImplementation->findVariable("YODouble");
   EXPECT_TRUE(testVariables[0].get() == variable);
}

TEST_F(YoVariableHolderImplementationNewTest, testGetVariableWithNamespace)
{
   YoRegistry testRegistry("testRegistry");
   variable::YoDouble yoDoubleWithNamespace("yoDoubleWithNamespace", &testRegistry);
   yoVariableHolderImplementation->add(&yoDoubleWithNamespace);
   EXPECT_EQ(&yoDoubleWithNamespace, yoVariableHolderImplementation->findVariable("testRegistry", "yoDoubleWithNamespace"));
}

TEST_F(YoVariableHolderImplementationNewTest, testGetVariableWithNamespaceCaseInsensitiveExceptNamespace)
{
   YoRegistry testRegistry("testRegistry");
   variable::YoDouble yoDoubleWithNamespace("yoDoubleWithNamespace", &testRegistry);
   yoVariableHolderImplementation->add(&yoDoubleWithNamespace);
   EXPECT_EQ(&yoDoubleWithNamespace, yoVariableHolderImplementation->findVariable("testRegistry", "yoDOUBLEWithNamespace"));
   EXPECT_EQ(yoVariableHolderImplementation->findVariable("TESTRegistry", "yoDoubleWithNamespace"), nullptr);
}

TEST_F(YoVariableHolderImplementationNewTest, testHasUniqueVariable)
{
   yoVariableHolderImplementation->addAll(testVariablePointers());
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("yoDouble"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("yoDoubleNotPresent"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("yoBoolean"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("yoInteger"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("yoIntegerNotPresent"));
}

TEST_F(YoVariableHolderImplementationNewTest, testHasUniqueVariableWithNamespace)
{
   YoRegistry testRegistry1("testRegistry1");
   YoRegistry testRegistry2("testRegistry2");
   variable::YoDouble yoDoubleWithNamespace1("yoDoubleWithNamespace1", &testRegistry1);
   variable::YoDouble yoDoubleWithNamespace2("yoDoubleWithNamespace2", &testRegistry2);
   yoVariableHolderImplementation->add(&yoDoubleWithNamespace1);
   yoVariableHolderImplementation->add(&yoDoubleWithNamespace2);
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("testRegistry1", "yoDoubleWithNamespace1"));
   EXPECT_TRUE(yoVariableHolderImplementation->hasUniqueVariable("testRegistry2", "yoDoubleWithNamespace2"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("testRegistry1", "yoDoubleWithNamespace2"));
   EXPECT_FALSE(yoVariableHolderImplementation->hasUniqueVariable("testRegistry2", "yoDoubleWithNamespace1"));
}

TEST_F(YoVariableHolderImplementationNewTest, testGetVariablesInNamespace)
{
   YoRegistry testRegistry1("testRegistry1");
   YoRegistry testRegistry2("testRegistry2");
   variable::YoDouble yoDoubleWithNamespace1("yoDoubleWithNamespace1", &testRegistry1);
   variable::YoDouble yoDoubleWithNamespace2("yoDoubleWithNamespace2", &testRegistry2);
   variable::YoBoolean yoBooleanWithNamespace1("yoBooleanWithNamespace1", &testRegistry1);
   variable::YoBoolean yoBooleanWithNamespace2("yoBooleanWithNamespace2", &testRegistry2);
   variable::YoInteger yoIntegerWithNamespace1("yoIntegerWithNamespace1", &testRegistry1);
   variable::YoInteger yoIntegerWithNamespace2("yoIntegerWithNamespace2", &testRegistry2);
   yoVariableHolderImplementation->add(&yoDoubleWithNamespace1);
   yoVariableHolderImplementation->add(&yoDoubleWithNamespace2);
   yoVariableHolderImplementation->add(&yoBooleanWithNamespace1);
   yoVariableHolderImplementation->add(&yoBooleanWithNamespace2);
   yoVariableHolderImplementation->add(&yoIntegerWithNamespace1);
   yoVariableHolderImplementation->add(&yoIntegerWithNamespace2);

   std::vector<variable::YoVariable*> expectedFromNamespaceTestRegistry1{&yoDoubleWithNamespace1, &yoBooleanWithNamespace1, &yoIntegerWithNamespace1};

   // Doesn't necessarily return the results in a specific order, so containment is checked instead.
   std::vector<variable::YoVariable*> actual = yoVariableHolderImplementation->findVariables(testRegistry1.getNamespace());
   for (variable::YoVariable* expected : expectedFromNamespaceTestRegistry1)
      EXPECT_NE(std::find(actual.begin(), actual.end(), expected), actual.end());
}
} // namespace
} // namespace ihmc::yovariables::registry
