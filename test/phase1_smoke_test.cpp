#include <gtest/gtest.h>

#include "ihmc/yovariables/listener/yo_variable_changed_listener.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"
#include "ihmc/yovariables/variable/yo_integer.h"
#include "ihmc/yovariables/variable/yo_long.h"

using namespace ihmc::yovariables;

enum class Color
{
   RED,
   GREEN,
   BLUE
};

TEST(Phase1Smoke, RegistryTreeAndVariableLookup)
{
   registry::YoRegistry root("root");
   registry::YoRegistry child("child");
   root.addChild(&child);

   variable::YoDouble x("x", &child);
   x.set(3.5);

   EXPECT_EQ(root.getNumberOfVariablesDeep(), 1u);
   EXPECT_EQ(child.getVariable("x"), &x);
   EXPECT_EQ(x.getFullNameString(), "root.child.x");
   EXPECT_DOUBLE_EQ(x.getValueAsDouble(), 3.5);

   variable::YoVariable* found = root.findVariable("x");
   EXPECT_EQ(found, &x);
}

TEST(Phase1Smoke, YoDoubleNotifiesListeners)
{
   registry::YoRegistry root("root");
   variable::YoDouble x("x", &root);

   struct RecordingListener : listener::YoVariableChangedListener
   {
      int callCount = 0;
      void changed(variable::YoVariable&) override { callCount++; }
   } recordingListener;

   x.addListener(&recordingListener);
   EXPECT_TRUE(x.set(1.0));
   EXPECT_FALSE(x.set(1.0));
   EXPECT_TRUE(x.set(2.0));
   EXPECT_EQ(recordingListener.callCount, 2);
}

TEST(Phase1Smoke, YoBooleanYoIntegerYoLongRoundTrip)
{
   registry::YoRegistry root("root");
   variable::YoBoolean flag("flag", &root);
   variable::YoInteger count("count", &root);
   variable::YoLong bigCount("bigCount", &root);

   flag.set(true);
   EXPECT_EQ(flag.getValueAsString(std::nullopt), "true");

   count.set(42);
   EXPECT_EQ(count.getValueAsDouble(), 42.0);

   bigCount.set(static_cast<std::int64_t>(1) << 40);
   EXPECT_EQ(bigCount.getValue(), static_cast<std::int64_t>(1) << 40);
}

TEST(Phase1Smoke, YoEnumBackedByRealEnum)
{
   registry::YoRegistry root("root");
   variable::YoEnum<Color> color("color", &root, false);

   EXPECT_TRUE(color.isBackedByEnum());
   EXPECT_EQ(color.getEnumSize(), 3);
   EXPECT_EQ(color.getValue(), Color::RED);

   color.set(Color::BLUE);
   EXPECT_EQ(color.getValue(), Color::BLUE);
   EXPECT_EQ(color.getStringValue(), "BLUE");

   EXPECT_TRUE(color.parseValue("GREEN", true));
   EXPECT_EQ(color.getValue(), Color::GREEN);
}

TEST(Phase1Smoke, YoEnumNullAllowed)
{
   registry::YoRegistry root("root");
   variable::YoEnum<Color> color("color", &root, true);

   EXPECT_EQ(color.getOrdinal(), variable::YoEnum<Color>::NULL_VALUE);
   EXPECT_FALSE(color.getEnumValueOrNull().has_value());

   color.set(Color::RED);
   EXPECT_TRUE(color.getEnumValueOrNull().has_value());

   color.set(std::optional<Color>(std::nullopt), true);
   EXPECT_FALSE(color.getEnumValueOrNull().has_value());
}

TEST(Phase1Smoke, RegistryDestroyDetachesVariables)
{
   registry::YoRegistry root("root");
   variable::YoDouble x("x", &root);

   EXPECT_EQ(x.getRegistry(), &root);
   root.destroy();
   EXPECT_EQ(x.getRegistry(), nullptr);
   EXPECT_EQ(root.getNumberOfVariables(), 0u);
}
