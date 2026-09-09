#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/registry/yo_variable_list.h"
#include "ihmc/yovariables/tools/yo_search_tools.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::registry
{
namespace
{
TEST(YoVariableListTest, testCommonUsage)
{
   YoVariableList varList("listOne");
   YoRegistry registryOne("registryOne");
   YoRegistry registryTwo("registryTwo");

   variable::YoBoolean booleanOne("booleanOne", &registryOne);
   variable::YoDouble doubleOne("doubleOne", &registryOne);
   varList.add(&booleanOne);
   varList.add(&doubleOne);

   variable::YoBoolean booleanTwo("booleanTwo", &registryTwo);
   variable::YoDouble doubleTwo("doubleTwo", &registryTwo);
   varList.add(&booleanTwo);
   varList.add(&doubleTwo);

   // Here's the tricky case. Same name variable booleanOne but with a different name space.
   // Design question is should varList.findVariable("booleanOne") return the first one, both, or
   // throw an exception? Right now it returns the first one. This has problems with graphGroups and
   // such and should be fixed later.
   variable::YoBoolean repeatBooleanOne("booleanOne", &registryTwo);
   varList.add(&repeatBooleanOne);

   variable::YoBoolean notIncluded("notIncluded", &registryTwo);

   EXPECT_EQ(varList.getName(), "listOne");
   EXPECT_EQ(varList.indexOf(&booleanOne), 0);
   EXPECT_EQ(varList.indexOf(&doubleOne), 1);
   EXPECT_EQ(varList.indexOf(&booleanTwo), 2);
   EXPECT_EQ(varList.indexOf(&doubleTwo), 3);
   EXPECT_EQ(varList.indexOf(&repeatBooleanOne), 4);

   EXPECT_EQ(varList.indexOf(&notIncluded), -1);

   EXPECT_TRUE(varList.contains(&booleanOne));
   EXPECT_TRUE(varList.contains(&doubleOne));
   EXPECT_TRUE(varList.contains(&booleanTwo));
   EXPECT_TRUE(varList.contains(&doubleTwo));
   EXPECT_TRUE(varList.contains(&repeatBooleanOne));
   EXPECT_FALSE(varList.contains(&notIncluded));

   EXPECT_THROW(varList.get(-1), std::out_of_range);
   EXPECT_TRUE(&booleanOne == varList.get(0));
   EXPECT_TRUE(&doubleOne == varList.get(1));
   EXPECT_TRUE(&booleanTwo == varList.get(2));
   EXPECT_TRUE(&doubleTwo == varList.get(3));
   EXPECT_TRUE(&repeatBooleanOne == varList.get(4));
   EXPECT_THROW(varList.get(5), std::out_of_range);

   EXPECT_TRUE(varList.hasVariable("booleanOne"));
   EXPECT_FALSE(varList.hasUniqueVariable("booleanOne"));
   EXPECT_TRUE(varList.hasUniqueVariable("registryOne.booleanOne"));
   EXPECT_TRUE(varList.hasUniqueVariable("doubleOne"));
   EXPECT_TRUE(varList.hasUniqueVariable("registryOne.doubleOne"));
   EXPECT_TRUE(varList.hasUniqueVariable("booleanTwo"));
   EXPECT_TRUE(varList.hasUniqueVariable("registryTwo.booleanTwo"));
   EXPECT_TRUE(varList.hasUniqueVariable("doubleTwo"));
   EXPECT_TRUE(varList.hasUniqueVariable("registryTwo.doubleTwo"));
   EXPECT_TRUE(varList.hasUniqueVariable("registryTwo.booleanOne"));

   EXPECT_FALSE(varList.hasUniqueVariable("notIncluded"));
   EXPECT_FALSE(varList.hasUniqueVariable("registryOne.doubleTwo"));

   EXPECT_TRUE(&booleanOne == varList.findVariable("booleanOne"));
   EXPECT_TRUE(&doubleOne == varList.findVariable("doubleOne"));
   EXPECT_TRUE(&booleanTwo == varList.findVariable("booleanTwo"));
   EXPECT_TRUE(&doubleTwo == varList.findVariable("doubleTwo"));
   EXPECT_TRUE(&booleanOne == varList.findVariable("registryOne.booleanOne"));
   EXPECT_TRUE(&doubleOne == varList.findVariable("registryOne.doubleOne"));
   EXPECT_TRUE(&booleanTwo == varList.findVariable("registryTwo.booleanTwo"));
   EXPECT_TRUE(&doubleTwo == varList.findVariable("registryTwo.doubleTwo"));

   EXPECT_TRUE(&repeatBooleanOne == varList.findVariable("registryTwo.booleanOne"));

   EXPECT_EQ(varList.findVariable("registryOne.doubleTwo"), nullptr);
   EXPECT_EQ(varList.findVariable("notIncluded"), nullptr);
}

TEST(YoVariableListTest, testGetPerformanceInLargeList)
{
   // Test should take O(n) or O(n lg n) approximately.
   YoRegistry rootRegistry("rootRegistry");
   YoRegistry registryOne("registryOne");
   YoRegistry registryTwo("registryTwo");
   YoRegistry registryThree("registryThree");

   rootRegistry.addChild(&registryOne);
   registryOne.addChild(&registryTwo);
   registryTwo.addChild(&registryThree);

   variable::YoDouble t("t", &registryThree);
   variable::YoDouble time("time", &registryThree);
   t.set(1.1);
   time.set(2.2);

   const int numberOfVariables = 4000;
   std::vector<std::unique_ptr<variable::YoDouble>> variables;
   YoVariableList varList("test");

   for (int i = 0; i < numberOfVariables; i++)
   {
      std::string name = "variable" + std::to_string(i);
      auto variableA = std::make_unique<variable::YoDouble>(name, &registryThree);
      auto variableB = std::make_unique<variable::YoDouble>(name, &registryTwo);
      variableA->set(0.5);
      variableB->set(0.25);

      varList.add(variableA.get());
      varList.add(variableB.get());
      variables.push_back(std::move(variableA));
      variables.push_back(std::move(variableB));
   }

   EXPECT_EQ(varList.size(), variables.size());

   for (const std::unique_ptr<variable::YoDouble>& yoVariable : variables)
   {
      EXPECT_TRUE(varList.hasUniqueVariable(yoVariable->getFullNameString()));
      EXPECT_TRUE(yoVariable.get() == varList.findVariable(yoVariable->getFullNameString()));
   }
}

TEST(YoVariableListTest, testToString)
{
   YoRegistry registry("registry");
   YoVariableList list("list");

   variable::YoDouble a("a", &registry);
   variable::YoDouble b("b", &registry);
   variable::YoDouble c("c", &registry);

   list.add(&a);
   list.add(&b);
   list.add(&c);

   EXPECT_EQ(list.toString(), list.getName() + ", variables:\n" + a.toString() + "\n" + b.toString() + "\n" + c.toString());
}

TEST(YoVariableListTest, testAddVariables)
{
   YoRegistry registry("registry");
   YoVariableList list("list");
   YoVariableList listTwo("listTwo");
   YoVariableList listThree("listThree");
   YoVariableList listFour("listFour");

   variable::YoDouble a("a", &registry);
   variable::YoDouble b("b", &registry);
   variable::YoDouble c("c", &registry);

   list.add(&a);
   EXPECT_EQ(list.size(), 1u);
   list.add(&a); // Ignores if added twice.
   EXPECT_EQ(list.size(), 1u);

   listTwo.add(&b);
   listTwo.add(&c);
   EXPECT_EQ(listTwo.size(), 2u);

   list.addAll(listTwo);
   EXPECT_EQ(list.size(), 3u);

   listThree.add(&a);
   listThree.addAll(std::vector<variable::YoVariable*>{&b, &c});

   listFour.add(&a);
   listFour.addAll(std::vector<variable::YoVariable*>{&b, &c});

   for (std::size_t i = 0; i < list.size(); i++)
   {
      EXPECT_EQ(list.get(static_cast<int>(i)), listThree.get(static_cast<int>(i)));
      EXPECT_EQ(list.get(static_cast<int>(i)), listFour.get(static_cast<int>(i)));
   }
}

TEST(YoVariableListTest, testCommonUsageTwo)
{
   YoRegistry registry("registry");
   YoVariableList list("list");
   YoVariableList listTwo("list");
   YoVariableList listThree("list");

   EXPECT_TRUE(list.isEmpty());

   variable::YoDouble a("a", &registry);
   variable::YoDouble b("b", &registry);
   variable::YoDouble c("c", &registry);
   variable::YoDouble f("f", &registry); // variable will not be added to list

   list.add(&a);
   list.add(&b);
   list.add(&c);

   list.remove(&b);
   list.remove(&f); // attempt to remove variable not in list

   listTwo.add(&a);
   listTwo.add(&c);

   EXPECT_EQ(list.toString(), listTwo.toString());

   EXPECT_FALSE(list.isEmpty());

   list.clear();
   EXPECT_EQ(list.toString(), listThree.toString());

   std::vector<variable::YoVariable*> allVariables = listTwo.getVariables();

   for (std::size_t i = 0; i < listTwo.size(); i++)
      EXPECT_EQ(allVariables[i]->toString(), listTwo.get(static_cast<int>(i))->toString());
}

TEST(YoVariableListTest, testGetMatchingVariables)
{
   YoRegistry registry("registry");
   YoVariableList list("list");

   EXPECT_TRUE(list.isEmpty());

   variable::YoDouble a("a_arm", &registry);
   variable::YoDouble b("b_arm", &registry);
   variable::YoDouble c("c_arm", &registry);
   variable::YoDouble f("f_arm", &registry); // variable will not be added to list

   list.add(&a);
   list.add(&b);
   list.add(&c);

   std::string regularExpression = ".*";

   std::vector<variable::YoVariable*> matchedAll = tools::filterVariables(tools::regularExpressionFilter({regularExpression}), list);

   EXPECT_TRUE(std::find(matchedAll.begin(), matchedAll.end(), &a) != matchedAll.end());
   EXPECT_TRUE(std::find(matchedAll.begin(), matchedAll.end(), &b) != matchedAll.end());
   EXPECT_TRUE(std::find(matchedAll.begin(), matchedAll.end(), &c) != matchedAll.end());
   EXPECT_FALSE(std::find(matchedAll.begin(), matchedAll.end(), &f) != matchedAll.end());

   std::string regexpStartWithC = "c.*";
   std::vector<variable::YoVariable*> matchedStartWithC = tools::filterVariables(tools::regularExpressionFilter({regexpStartWithC}), list);

   EXPECT_FALSE(std::find(matchedStartWithC.begin(), matchedStartWithC.end(), &a) != matchedStartWithC.end());
   EXPECT_FALSE(std::find(matchedStartWithC.begin(), matchedStartWithC.end(), &b) != matchedStartWithC.end());
   EXPECT_TRUE(std::find(matchedStartWithC.begin(), matchedStartWithC.end(), &c) != matchedStartWithC.end());
   EXPECT_FALSE(std::find(matchedStartWithC.begin(), matchedStartWithC.end(), &f) != matchedStartWithC.end());

   std::vector<variable::YoVariable*> namesOrStartWithC = tools::filterVariables(tools::regularExpressionFilter({regularExpression}), list);

   EXPECT_TRUE(std::find(namesOrStartWithC.begin(), namesOrStartWithC.end(), &a) != namesOrStartWithC.end());
   EXPECT_TRUE(std::find(namesOrStartWithC.begin(), namesOrStartWithC.end(), &b) != namesOrStartWithC.end());
   EXPECT_TRUE(std::find(namesOrStartWithC.begin(), namesOrStartWithC.end(), &c) != namesOrStartWithC.end());
   EXPECT_FALSE(std::find(namesOrStartWithC.begin(), namesOrStartWithC.end(), &f) != namesOrStartWithC.end());

   // Return empty list when none match.
   std::vector<variable::YoVariable*> matchedNameShouldBeEmpty = tools::filterVariables(tools::regularExpressionFilter({"foo"}), list);
   EXPECT_TRUE(matchedNameShouldBeEmpty.empty());
   matchedNameShouldBeEmpty = tools::filterVariables(tools::regularExpressionFilter({"bar"}), list);
   EXPECT_TRUE(matchedNameShouldBeEmpty.empty());
}

// Guards the assumption that YoVariableList preserves insertion order (via get(), indexOf(), and
// getVariables()) rather than whatever order an internal lookup structure (e.g. a name-to-variable
// map) happens to iterate. Names are added in an order that is neither alphabetical nor likely to
// match typical hash bucket ordering, so a regression backed by an unordered map would very likely
// be caught.
TEST(YoVariableListTest, testGetAndIndexOfPreserveInsertionOrder)
{
   YoRegistry registryA("registryA");
   YoVariableList list("orderList");

   std::vector<std::string> variableNames{"zebra", "mango", "apple", "banana", "kiwi", "fig"};
   std::vector<std::unique_ptr<variable::YoDouble>> variables;
   for (const std::string& name : variableNames)
   {
      variables.push_back(std::make_unique<variable::YoDouble>(name, &registryA));
      list.add(variables.back().get());
   }

   ASSERT_EQ(list.size(), variableNames.size());
   for (std::size_t i = 0; i < variableNames.size(); i++)
   {
      EXPECT_EQ(list.get(static_cast<int>(i)), variables[i].get()) << "Variable at index " << i << " was out of insertion order.";
      EXPECT_EQ(list.indexOf(variables[i].get()), static_cast<int>(i)) << "indexOf did not match insertion order for index " << i << ".";
   }

   std::vector<variable::YoVariable*> actualVariables = list.getVariables();
   for (std::size_t i = 0; i < variableNames.size(); i++)
      EXPECT_EQ(actualVariables[i], variables[i].get()) << "getVariables() index " << i << " was out of insertion order.";

   // Removing from the middle should not reshuffle the remaining entries.
   list.remove(variables[2].get()); // "apple"
   ASSERT_EQ(list.size(), variableNames.size() - 1);
   EXPECT_EQ(list.get(0), variables[0].get());
   EXPECT_EQ(list.get(1), variables[1].get());
   EXPECT_EQ(list.get(2), variables[3].get());
   EXPECT_EQ(list.get(3), variables[4].get());
   EXPECT_EQ(list.get(4), variables[5].get());

   // A variable added after a removal should land at the end, not in the vacated slot.
   YoRegistry registryB("registryB");
   variable::YoDouble reAdded("apple", &registryB);
   list.add(&reAdded);
   ASSERT_EQ(list.size(), variableNames.size());
   EXPECT_EQ(list.get(static_cast<int>(list.size()) - 1), &reAdded);
}
} // namespace
} // namespace ihmc::yovariables::registry
