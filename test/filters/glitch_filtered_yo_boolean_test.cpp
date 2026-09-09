#include <gtest/gtest.h>

#include <memory>

#include "ihmc/yovariables/filters/glitch_filtered_yo_boolean.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"

namespace ihmc::yovariables::filters
{
namespace
{
constexpr int kWindowSize = 10;

class GlitchFilteredYoBooleanTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("testRegistry");
      yoVariableToFilter = std::make_unique<variable::YoBoolean>("variableToFilter", registry.get());
      filteredVariable = std::make_unique<GlitchFilteredYoBoolean>("filteredVariable", "", registry.get(), yoVariableToFilter.get(), kWindowSize);
   }

   std::unique_ptr<registry::YoRegistry> registry;
   std::unique_ptr<variable::YoBoolean> yoVariableToFilter;
   std::unique_ptr<GlitchFilteredYoBoolean> filteredVariable;
};

TEST_F(GlitchFilteredYoBooleanTest, testConstructors_Set_Get)
{
   registry::YoRegistry testRegistry1("test");
   registry::YoRegistry testRegistry2("test");

   GlitchFilteredYoBoolean number1("stringInt", &testRegistry1, kWindowSize);
   GlitchFilteredYoBoolean number2("stringYoVariableRegistryInt", registry.get(), kWindowSize);
   GlitchFilteredYoBoolean number3("stringBooleanYoVariableInt", "", &testRegistry2, yoVariableToFilter.get(), kWindowSize);
   GlitchFilteredYoBoolean number4("stringYoVariableRegistryBooleanYoVariableInt", "", registry.get(), yoVariableToFilter.get(), kWindowSize);

   GlitchFilteredYoBoolean* array[] = {&number1, &number2, &number3, &number4};

   for (GlitchFilteredYoBoolean* variable : array)
   {
      EXPECT_FALSE(variable->getBooleanValue());

      variable->set(true);
      EXPECT_TRUE(variable->getBooleanValue());

      variable->set(false);
      EXPECT_FALSE(variable->getBooleanValue());
   }
}

TEST_F(GlitchFilteredYoBooleanTest, testUpdate)
{
   int windowSize = 3;
   filteredVariable->set(true);
   filteredVariable->setWindowSize(windowSize);

   int counter = 0;
   for (int i = 0; i < windowSize + 10; i++)
   {
      filteredVariable->update(false);
      counter++;

      if (counter < windowSize)
         EXPECT_TRUE(filteredVariable->getBooleanValue());
      else
         EXPECT_FALSE(filteredVariable->getBooleanValue());
   }

   // Java's final block sets filteredVariable to null and expects update() to throw a
   // NullPointerException. Dereferencing a null pointer in C++ is undefined behavior, not a
   // catchable exception, so there is no equivalent to port here.
}

// Java's testCounter reaches into GlitchFilteredYoBoolean's package-private `counter` field
// directly. This port's equivalent member (counter_) is private with no accessor exposed - the
// Java source's access level was already tighter than "public API" and this port makes it fully
// private rather than approximating package-private - so there's nothing to observe this through
// and the test isn't ported.

TEST_F(GlitchFilteredYoBooleanTest, testFiltering)
{
   yoVariableToFilter->set(true);

   for (int i = 0; i < kWindowSize / 2; i++)
      filteredVariable->update();

   EXPECT_NE(yoVariableToFilter->getBooleanValue(), filteredVariable->getBooleanValue());

   for (int i = 0; i < kWindowSize / 2; i++)
      filteredVariable->update();

   EXPECT_EQ(yoVariableToFilter->getBooleanValue(), filteredVariable->getBooleanValue());
}
} // namespace
} // namespace ihmc::yovariables::filters
