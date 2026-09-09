#include <gtest/gtest.h>

#include <vector>

#include "ihmc/yovariables/listener/yo_variable_changed_listener.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_enum.h"

using namespace ihmc::yovariables;

namespace
{
enum class EnumYoVariableTestEnums
{
   ONE,
   TWO
};

enum class EmptyEnum
{
};

class YoEnumTest : public ::testing::Test
{
protected:
   void SetUp() override { registry = std::make_unique<registry::YoRegistry>("testRegistry"); }

   static constexpr double EPSILON = 1e-10;
   std::unique_ptr<registry::YoRegistry> registry;
};

TEST_F(YoEnumTest, testConstructorNoDescription)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   EXPECT_EQ(registry->getVariables().size(), 1u);
   EXPECT_EQ(yoEnum.getName(), "yoEnum");
   EXPECT_EQ(registry->findVariable("yoEnum"), &yoEnum);
}

TEST_F(YoEnumTest, testConstructorWithDescription)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", "yoEnum with description", registry.get(), false);
   EXPECT_EQ(registry->getVariables().size(), 1u);
   EXPECT_EQ(yoEnum.getName(), "yoEnum");
   EXPECT_EQ(registry->findVariable("yoEnum"), &yoEnum);
   EXPECT_EQ(yoEnum.getDescription(), "yoEnum with description");
}

TEST_F(YoEnumTest, testSetAndValueEquals)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), true);
   yoEnum.set(EnumYoVariableTestEnums::ONE);
   EXPECT_TRUE(yoEnum.valueEquals(EnumYoVariableTestEnums::ONE));
   yoEnum.set(EnumYoVariableTestEnums::TWO);
   EXPECT_TRUE(yoEnum.valueEquals(EnumYoVariableTestEnums::TWO));

   yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true);
   EXPECT_TRUE(yoEnum.valueEquals(std::nullopt));
}

TEST_F(YoEnumTest, testSetAndGet)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   yoEnum.set(EnumYoVariableTestEnums::ONE);
   EXPECT_EQ(EnumYoVariableTestEnums::ONE, yoEnum.getEnumValue());
   yoEnum.set(EnumYoVariableTestEnums::TWO);
   EXPECT_EQ(EnumYoVariableTestEnums::TWO, yoEnum.getEnumValue());
   yoEnum.set(EnumYoVariableTestEnums::ONE, false);
   EXPECT_EQ(EnumYoVariableTestEnums::ONE, yoEnum.getEnumValue());
   yoEnum.set(EnumYoVariableTestEnums::TWO, false);
   EXPECT_EQ(EnumYoVariableTestEnums::TWO, yoEnum.getEnumValue());
}

TEST_F(YoEnumTest, testGetValues)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   const std::vector<EnumYoVariableTestEnums>& enumTypeArray = yoEnum.getEnumValues();
   EXPECT_EQ(enumTypeArray.size(), 2u);
   EXPECT_EQ(EnumYoVariableTestEnums::ONE, enumTypeArray[0]);
   EXPECT_EQ(EnumYoVariableTestEnums::TWO, enumTypeArray[1]);
}

TEST_F(YoEnumTest, testSetValueAsDoublePositiveNumber)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   yoEnum.setValueFromDouble(0.0, true);
   EXPECT_EQ(EnumYoVariableTestEnums::ONE, yoEnum.getEnumValue());
   yoEnum.setValueFromDouble(0.25, true);
   EXPECT_EQ(EnumYoVariableTestEnums::ONE, yoEnum.getEnumValue());
   yoEnum.setValueFromDouble(0.5, true);
   EXPECT_EQ(EnumYoVariableTestEnums::TWO, yoEnum.getEnumValue());
   yoEnum.setValueFromDouble(1.0, true);
   EXPECT_EQ(EnumYoVariableTestEnums::TWO, yoEnum.getEnumValue());
}

TEST_F(YoEnumTest, testSetValueAsDoubleOutOfBoundsJustIgnoresIt)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);

   yoEnum.setValueFromDouble(2.0, true);
   EXPECT_EQ(EnumYoVariableTestEnums::TWO, yoEnum.getEnumValue());

   yoEnum.setValueFromDouble(-100.0, true);
   EXPECT_EQ(EnumYoVariableTestEnums::ONE, yoEnum.getEnumValue());
}

TEST_F(YoEnumTest, testForNull)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", "", registry.get(), true);
   yoEnum.setValueFromDouble(static_cast<double>(variable::YoEnum<EnumYoVariableTestEnums>::NULL_VALUE), true);
   EXPECT_FALSE(yoEnum.getEnumValueOrNull().has_value());
}

TEST_F(YoEnumTest, testNotAllowNull)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   EXPECT_THROW(yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true), std::invalid_argument);
}

TEST_F(YoEnumTest, testAllowNull)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", "", registry.get(), true);
   yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true);
   EXPECT_FALSE(yoEnum.getEnumValueOrNull().has_value());
}

TEST_F(YoEnumTest, testGetValueAsDouble)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), true);
   yoEnum.set(EnumYoVariableTestEnums::ONE);
   EXPECT_NEAR(yoEnum.getValueAsDouble(), 0.0, EPSILON);
   yoEnum.set(EnumYoVariableTestEnums::TWO);
   EXPECT_NEAR(yoEnum.getValueAsDouble(), 1.0, EPSILON);
   yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true);
   EXPECT_NEAR(yoEnum.getValueAsDouble(), -1.0, EPSILON);
}

TEST_F(YoEnumTest, testToString)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   yoEnum.set(EnumYoVariableTestEnums::ONE);
   EXPECT_EQ(yoEnum.toString(), "yoEnum: ONE");
   yoEnum.set(EnumYoVariableTestEnums::TWO);
   EXPECT_EQ(yoEnum.toString(), "yoEnum: TWO");
}

TEST_F(YoEnumTest, testGetYoVariableType)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   EXPECT_EQ(variable::YoVariableType::ENUM, yoEnum.getType());
}

TEST_F(YoEnumTest, testGetValueAsLongBitsAndSetValueFromLongBits)
{
   std::int64_t longValue = 1;
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), true);
   yoEnum.setValueFromLongBits(longValue, true);
   EXPECT_EQ(longValue, yoEnum.getValueAsLongBits());

   EXPECT_THROW(yoEnum.setValueFromLongBits(2, true), std::runtime_error);

   yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true);
   EXPECT_EQ(-1, yoEnum.getValueAsLongBits());
}

// testGetEnumType is not ported: it exercises Java's Class<E> reflection
// (yoEnum.getEnumType()), which has no equivalent in this port (see the Phase 1 decision to
// replace enum reflection with magic_enum rather than expose a runtime type object).

TEST_F(YoEnumTest, testDuplicate)
{
   registry::YoRegistry newRegistry("newRegistry");
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   std::unique_ptr<variable::YoVariable> duplicate = yoEnum.duplicate(&newRegistry);

   EXPECT_EQ(duplicate->getName(), yoEnum.getName());
   EXPECT_EQ(duplicate->getDescription(), yoEnum.getDescription());
   auto* duplicateEnum = dynamic_cast<variable::YoEnum<EnumYoVariableTestEnums>*>(duplicate.get());
   ASSERT_NE(duplicateEnum, nullptr);
   EXPECT_EQ(duplicateEnum->isNullAllowed(), yoEnum.isNullAllowed());
}

TEST_F(YoEnumTest, testProviderValue)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("yoEnum", registry.get(), false);
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum2("yoEnum2", registry.get(), false);

   yoEnum.set(EnumYoVariableTestEnums::ONE);
   yoEnum2.set(EnumYoVariableTestEnums::TWO);

   EXPECT_EQ(yoEnum.getEnumValue(), yoEnum.getValue());
   EXPECT_EQ(yoEnum2.getEnumValue(), yoEnum2.getValue());
}

// testStringBasedAccessNullConstant is not ported: it checks that a literal null entry in the
// constants varargs throws. std::string (used for the constants vector here) has no null state -
// every element is a valid, if possibly empty, string - so this specific case doesn't have a
// meaningful C++ equivalent to test.

// The enum-backed half of Java's testEmptyConstantList/testEmptyConstantListNotNull (constructing
// a YoEnum<EmptyEnum> for a genuinely zero-constant enum) is not ported: magic_enum cannot reflect
// an enum with no enumerators at all - it needs at least one to calibrate the value range it
// probes - so YoEnum<E>'s enum-backed constructor cannot support this case, a real capability gap
// against Java (which allows a zero-constant enum fine) surfaced by writing this test, not
// previously exercised by Phase 1's own tests. The string-backed constructor path (tested below)
// has no such limitation, since it never touches magic_enum.

TEST_F(YoEnumTest, testEmptyConstantList)
{
   registry::YoRegistry localRegistry("test");

   variable::YoEnum<EnumYoVariableTestEnums> stringConstructor("stringConstructor", "", &localRegistry, true, std::vector<std::string>{});

   EXPECT_EQ(stringConstructor.getStringValue(), "null");
}

TEST_F(YoEnumTest, testEmptyStringConstantListNotNull)
{
   registry::YoRegistry localRegistry("test");
   EXPECT_THROW((variable::YoEnum<EnumYoVariableTestEnums>("stringConstructor", "", &localRegistry, false, std::vector<std::string>{})),
                std::invalid_argument);
}

TEST_F(YoEnumTest, testListener)
{
   variable::YoEnum<EnumYoVariableTestEnums> yoEnum("anEnum", "", registry.get(), true);
   yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true);

   bool hasChanged = false;
   struct Listener : listener::YoVariableChangedListener
   {
      bool* flag;
      void changed(variable::YoVariable&) override { *flag = true; }
   } listenerInstance;
   listenerInstance.flag = &hasChanged;
   yoEnum.addListener(&listenerInstance);

   for (EnumYoVariableTestEnums value : {EnumYoVariableTestEnums::ONE, EnumYoVariableTestEnums::TWO})
   {
      yoEnum.set(value);
      EXPECT_TRUE(hasChanged);
      hasChanged = false;
      yoEnum.set(value);
      EXPECT_FALSE(hasChanged);
      hasChanged = false;
   }

   yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true);
   EXPECT_TRUE(hasChanged);
   hasChanged = false;
   yoEnum.set(std::optional<EnumYoVariableTestEnums>(std::nullopt), true);
   EXPECT_FALSE(hasChanged);
}
} // namespace
