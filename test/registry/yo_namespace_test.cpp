#include <gtest/gtest.h>

#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/tools/yo_tools.h"

namespace ihmc::yovariables::registry
{
namespace
{
TEST(YoNamespaceTest, testConstructors)
{
   YoNamespace("robot1.controller1.module1");

   EXPECT_THROW(YoNamespace(""), std::exception);
   EXPECT_THROW(YoNamespace("foo.bar."), std::exception);
   EXPECT_THROW(YoNamespace(".foo.bar"), std::exception);
   EXPECT_THROW(YoNamespace("foo.foo").checkSanity(), std::exception);
   EXPECT_THROW(YoNamespace("foo.bar.foo").checkSanity(), std::exception);
   EXPECT_THROW(YoNamespace("foo..bar"), std::exception);
}

TEST(YoNamespaceTest, testEquals)
{
   YoNamespace namespace1("robot1.controller1.module1");
   YoNamespace namespace2("robot1.controller1.module1");
   YoNamespace namespace3("robot1.module1");
   YoNamespace namespace4("robot1.controller1.mod");
   YoNamespace namespace5("bot1.controller1.module1");

   EXPECT_TRUE(namespace1 == namespace1);
   EXPECT_TRUE(namespace2 == namespace2);
   EXPECT_TRUE(namespace3 == namespace3);
   EXPECT_TRUE(namespace4 == namespace4);
   EXPECT_TRUE(namespace5 == namespace5);

   EXPECT_TRUE(namespace1 == namespace2);
   EXPECT_TRUE(namespace2 == namespace1);
   EXPECT_TRUE(namespace1 != namespace3);
   EXPECT_TRUE(namespace1 != namespace4);
   EXPECT_TRUE(namespace1 != namespace5);
   EXPECT_TRUE(namespace3 != namespace1);
   EXPECT_TRUE(namespace4 != namespace1);
   EXPECT_TRUE(namespace5 != namespace1);

   // Java's namespace1.equals("A string") and namespace1.equals(null) are not ported: operator==
   // here is strongly typed to YoNamespace, so there is no equivalent "compare against an unrelated
   // type" or "compare against null" case to test.
}

TEST(YoNamespaceTest, testStartsWith)
{
   YoNamespace namespaceValue("robot1.controller1.module1");
   EXPECT_TRUE(namespaceValue.startsWith("robot1"));
   EXPECT_TRUE(namespaceValue.startsWith("robot1.controller1"));
   EXPECT_TRUE(namespaceValue.startsWith("robot1.controller1.module1"));

   EXPECT_FALSE(namespaceValue.startsWith("robot1.controller1.mod"));
   EXPECT_FALSE(namespaceValue.startsWith("robot1.controll"));
   EXPECT_FALSE(namespaceValue.startsWith("rob"));

   EXPECT_FALSE(namespaceValue.startsWith(".robot1"));
   EXPECT_FALSE(namespaceValue.startsWith("robot1."));

   EXPECT_FALSE(namespaceValue.startsWith(""));
}

TEST(YoNamespaceTest, testEndsWith)
{
   YoNamespace namespaceValue("robot1.controller1.module1");
   EXPECT_TRUE(namespaceValue.endsWith("module1"));
   EXPECT_TRUE(namespaceValue.endsWith("controller1.module1"));
   EXPECT_TRUE(namespaceValue.endsWith("robot1.controller1.module1"));

   EXPECT_FALSE(namespaceValue.endsWith("odule1"));
   EXPECT_FALSE(namespaceValue.endsWith("ontroller1.module1"));
   EXPECT_FALSE(namespaceValue.endsWith("obot1.controller1.module1"));

   EXPECT_FALSE(namespaceValue.endsWith("module1."));
   EXPECT_FALSE(namespaceValue.endsWith(".module1"));

   EXPECT_FALSE(namespaceValue.endsWith(""));
}

TEST(YoNamespaceTest, testGetShortName)
{
   YoNamespace namespaceValue("robot1.controller1.module1");
   EXPECT_EQ(namespaceValue.getShortName(), "module1");
   EXPECT_EQ(namespaceValue.getName(), "robot1.controller1.module1");

   YoNamespace namespaceValue2("module2");
   EXPECT_EQ(namespaceValue2.getName(), "module2");
   EXPECT_EQ(namespaceValue2.getShortName(), "module2");
}

TEST(YoNamespaceTest, testContains)
{
   YoNamespace namespaceValue("robot1.controller1.module1");

   EXPECT_TRUE(namespaceValue.contains("robot1"));
   EXPECT_FALSE(namespaceValue.contains("notMe"));
   EXPECT_FALSE(namespaceValue.contains("notMe.nope.noway.nada.unhunh"));
   EXPECT_TRUE(namespaceValue.contains("robot1.controller1"));
   EXPECT_TRUE(namespaceValue.contains("robot1.controller1.module1"));
   EXPECT_FALSE(namespaceValue.contains("robot1.notMe"));

   EXPECT_FALSE(namespaceValue.contains("robot1.controller1.mod"));
   EXPECT_FALSE(namespaceValue.contains("robot1.controll"));
   EXPECT_FALSE(namespaceValue.contains("rob"));

   EXPECT_TRUE(namespaceValue.contains("module1"));
   EXPECT_TRUE(namespaceValue.contains("controller1.module1"));
   EXPECT_TRUE(namespaceValue.contains("robot1.controller1.module1"));

   EXPECT_FALSE(namespaceValue.contains("odule1"));
   EXPECT_FALSE(namespaceValue.contains("ontroller1.module1"));
   EXPECT_FALSE(namespaceValue.contains("obot1.controller1.module1"));

   EXPECT_TRUE(namespaceValue.contains("controller1"));
   EXPECT_FALSE(namespaceValue.contains("controller1.mod"));
   EXPECT_FALSE(namespaceValue.contains("bot1.controller1"));
   EXPECT_FALSE(namespaceValue.contains(".controller1"));
   EXPECT_FALSE(namespaceValue.contains("controller1."));

   EXPECT_FALSE(namespaceValue.contains(""));
}

TEST(YoNamespaceTest, testStripOffFromBeginning)
{
   YoNamespace namespaceValue("root.name1.name2");

   YoNamespace namespaceToRemove("root");
   std::optional<YoNamespace> newNamespace = namespaceValue.removeStart(namespaceToRemove);
   ASSERT_TRUE(newNamespace.has_value());
   EXPECT_EQ(YoNamespace("name1.name2"), *newNamespace);

   namespaceToRemove = YoNamespace("root.name1");
   newNamespace = namespaceValue.removeStart(namespaceToRemove);
   ASSERT_TRUE(newNamespace.has_value());
   EXPECT_EQ(YoNamespace("name2"), *newNamespace);

   namespaceToRemove = YoNamespace("root.name1.name2");
   newNamespace = namespaceValue.removeStart(namespaceToRemove);
   EXPECT_FALSE(newNamespace.has_value());

   namespaceToRemove = YoNamespace("name1");
   newNamespace = namespaceValue.removeStart(namespaceToRemove);
   EXPECT_FALSE(newNamespace.has_value());
}

TEST(YoNamespaceTest, testStripOffNamespaceToGetVariableName)
{
   EXPECT_EQ(tools::toShortName("root.level1.level2.variable"), "variable");
   EXPECT_EQ(tools::toShortName("root.variable"), "variable");
   EXPECT_EQ(tools::toShortName("variable"), "variable");
}
} // namespace
} // namespace ihmc::yovariables::registry
