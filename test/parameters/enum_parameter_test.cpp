#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ihmc/yovariables/listener/yo_parameter_changed_listener.h"
#include "ihmc/yovariables/parameters/enum_parameter.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_enum.h"

namespace ihmc::yovariables::parameters
{
namespace
{
enum class TestEnum
{
   A,
   B,
   C,
   D,
   E,
   F,
   G,
   H,
   I,
   J,
   K,
   L,
   M
};

constexpr TestEnum kInitialValue = TestEnum::D;

struct ParamWithNamespace
{
   std::unique_ptr<registry::YoRegistry> root, a, b, c;
   std::unique_ptr<EnumParameter<TestEnum>> param;
};

ParamWithNamespace createParameterWithNamespace()
{
   ParamWithNamespace result;
   result.root = std::make_unique<registry::YoRegistry>("root");
   result.a = std::make_unique<registry::YoRegistry>("a");
   result.b = std::make_unique<registry::YoRegistry>("b");
   result.c = std::make_unique<registry::YoRegistry>("c");

   result.root->addChild(result.a.get());
   result.a->addChild(result.b.get());
   result.b->addChild(result.c.get());

   result.param =
      std::make_unique<EnumParameter<TestEnum>>("param", "paramDescription", result.c.get(), true, std::optional<TestEnum>(kInitialValue));

   return result;
}

variable::YoEnum<TestEnum>& asYoEnum(variable::YoVariable& variable)
{
   auto* cast = dynamic_cast<variable::YoEnum<TestEnum>*>(&variable);
   if (cast == nullptr)
      throw std::logic_error("Expected a YoEnum<TestEnum>.");
   return *cast;
}

class CallbackTest : public listener::YoParameterChangedListener
{
public:
   void changed(YoParameter&) override
   {
      set = true;
   }

   bool set = false;
};

TEST(EnumParameterTest, testLoadNullValue)
{
   registry::YoRegistry dummy("dummy");
   EnumParameter<TestEnum> yesnull("yesnull", &dummy, true, std::optional<TestEnum>(kInitialValue));

   yesnull.load("NULL");

   // Java's EnumParameter<E>.getValue() returns the nullable enum value directly (null here); this
   // port's getValue() forwards to YoEnum<E>::getEnumValue(), which throws on a null current value
   // (see the Phase 1 divergence on YoEnum's nullable-enum handling) rather than exposing an
   // optional/nullable result on EnumParameter itself. Reaching into the backing YoEnum's
   // getEnumValueOrNull() is the closest equivalent check.
   EXPECT_FALSE(asYoEnum(yesnull.getVariable()).getEnumValueOrNull().has_value());
}

TEST(EnumParameterTest, testDuplicate)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   variable::YoEnum<TestEnum>& var = asYoEnum(bundle.param->getVariable());
   bundle.param->loadDefault();

   var.set(TestEnum::E);

   registry::YoRegistry newRegistry("newRegistry");
   std::unique_ptr<variable::YoVariable> newVarBase = var.duplicate(&newRegistry);
   variable::YoEnum<TestEnum>& newVar = asYoEnum(*newVarBase);
   auto* newParam = dynamic_cast<EnumParameter<TestEnum>*>(newVar.getParameter());
   ASSERT_NE(newParam, nullptr);

   EXPECT_EQ(bundle.param->getName(), newParam->getName());
   EXPECT_EQ(bundle.param->getDescription(), newParam->getDescription());
   EXPECT_EQ(bundle.param->getValue(), newParam->getValue());

   EXPECT_EQ(var.isNullAllowed(), newVar.isNullAllowed());
   EXPECT_EQ(var.getEnumValues(), newVar.getEnumValues());
}

TEST(EnumParameterTest, testStringDuplicate)
{
   registry::YoRegistry root("root");

   EnumParameter<TestEnum> param("testString", "stringDescription", &root, true, std::vector<std::string>{"A", "B", "C", "D", "E", "F"});
   variable::YoEnum<TestEnum>& var = asYoEnum(param.getVariable());
   param.loadDefault();

   var.set(4);

   registry::YoRegistry newRegistry("newRegistry");
   std::unique_ptr<variable::YoVariable> newVarBase = var.duplicate(&newRegistry);
   variable::YoEnum<TestEnum>& newVar = asYoEnum(*newVarBase);
   auto* newParam = dynamic_cast<EnumParameter<TestEnum>*>(newVar.getParameter());
   ASSERT_NE(newParam, nullptr);

   EXPECT_EQ(param.getName(), newParam->getName());
   EXPECT_EQ(param.getDescription(), newParam->getDescription());
   EXPECT_EQ(param.getValueAsString(), newParam->getValueAsString());

   EXPECT_EQ(var.isBackedByEnum(), newVar.isBackedByEnum());
   EXPECT_EQ(var.isNullAllowed(), newVar.isNullAllowed());
   EXPECT_EQ(var.getEnumValuesAsString(), newVar.getEnumValuesAsString());
}

TEST(EnumParameterTest, testDisallowNullLoadValue)
{
   EXPECT_THROW(
      {
         registry::YoRegistry dummy("dummy");
         EnumParameter<TestEnum> nonull("nonull", &dummy, false, std::optional<TestEnum>(kInitialValue));
         nonull.load("null");
      },
      std::exception);
}

TEST(EnumParameterTest, testDisallowNullConstructValue)
{
   EXPECT_THROW(
      {
         registry::YoRegistry dummy("dummy");
         EnumParameter<TestEnum> nonull("nonull", &dummy, false, std::optional<TestEnum>(std::nullopt));
      },
      std::exception);
}

TEST(EnumParameterTest, testConstructDefaultValue)
{
   registry::YoRegistry dummy("dummy");
   EnumParameter<TestEnum> nonull("nonull", &dummy, false);
   EnumParameter<TestEnum> yesnull("yesnull", &dummy, true);

   nonull.loadDefault();
   yesnull.loadDefault();

   EXPECT_EQ(TestEnum::A, nonull.getValue());
   // See testLoadNullValue's comment: yesnull.getValue() would throw here (null current value), so
   // check via the backing YoEnum's nullable accessor instead.
   EXPECT_FALSE(asYoEnum(yesnull.getVariable()).getEnumValueOrNull().has_value());
}

TEST(EnumParameterTest, testGetNamespace)
{
   ParamWithNamespace bundle = createParameterWithNamespace();

   EXPECT_EQ(bundle.param->getNamespace()->toString(), "root.a.b.c");
   EXPECT_EQ(bundle.param->getName(), "param");
}

TEST(EnumParameterTest, testLoadFromString)
{
   for (TestEnum element : magic_enum::enum_values<TestEnum>())
   {
      registry::YoRegistry dummy("dummy");
      EnumParameter<TestEnum> param("test", &dummy, true, std::optional<TestEnum>(std::nullopt));
      std::string stringValue(magic_enum::enum_name(element));
      param.load(stringValue);

      EXPECT_EQ(element, param.getValue());
      EXPECT_EQ(stringValue, param.getValueAsString());
   }
}

TEST(EnumParameterTest, testGetBeforeLoad)
{
   EXPECT_THROW(
      {
         ParamWithNamespace bundle = createParameterWithNamespace();
         bundle.param->getValue();
      },
      std::exception);
}

TEST(EnumParameterTest, testDefault)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   bundle.param->loadDefault();
   EXPECT_EQ(kInitialValue, bundle.param->getValue());
}

TEST(EnumParameterTest, testListener)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   CallbackTest callback;
   bundle.param->addListener(&callback);

   EXPECT_FALSE(callback.set);

   bundle.param->loadDefault();

   callback.set = false;

   // No change
   bundle.param->getVariable().setValueFromDouble(bundle.param->getVariable().getValueAsDouble());
   EXPECT_FALSE(callback.set);

   bundle.param->getVariable().setValueFromDouble(static_cast<double>(static_cast<int>(TestEnum::K)));

   EXPECT_TRUE(callback.set);
}

TEST(EnumParameterTest, testStringBased)
{
   std::vector<std::string> constants{"A", "B", "C", "D", "E", "F", "G", "H"};

   registry::YoRegistry registry("test");

   EnumParameter<TestEnum> nullDefault("nullDefault", "", &registry, true, constants);
   nullDefault.loadDefault();
   EXPECT_EQ(nullDefault.getValueAsString(), "null");

   EnumParameter<TestEnum> constantDefault("constantDefault", "", &registry, false, constants);
   constantDefault.loadDefault();
   EXPECT_EQ(constantDefault.getValueAsString(), "A");

   for (const std::string& c : constants)
   {
      constantDefault.setToString(c);
      EXPECT_EQ(constantDefault.getValueAsString(), c);
   }

   nullDefault.setToString("A");
   EXPECT_EQ(nullDefault.getValueAsString(), "A");

   nullDefault.setToString("NULL");
   EXPECT_EQ(nullDefault.getValueAsString(), "null");
}

TEST(EnumParameterTest, testStringBasedAccess)
{
   // The brace-init-list below can't appear directly inside EXPECT_THROW's macro argument (the
   // preprocessor splits on its top-level commas, since {} isn't () for that purpose), so it's
   // built ahead of the statement instead.
   std::vector<std::string> constants{"A", "B", "C", "D", "E", "F", "G", "H"};
   EXPECT_THROW(
      {
         registry::YoRegistry registry("test");
         EnumParameter<TestEnum> nullDefault("nullDefault", "", &registry, true, constants);
         nullDefault.loadDefault();
         nullDefault.getValue();
      },
      std::exception);
}

TEST(EnumParameterTest, testStringBasedAccessSetNull)
{
   std::vector<std::string> constants{"A", "B", "C", "D", "E", "F", "G", "H"};
   EXPECT_THROW(
      {
         registry::YoRegistry registry("test");

         EnumParameter<TestEnum> constantDefault("constantDefault", "", &registry, false, constants);
         constantDefault.loadDefault();

         constantDefault.setToString("null");
      },
      std::exception);
}

TEST(EnumParameterTest, testStringBasedAccessSetNonExistant)
{
   std::vector<std::string> constants{"A", "B", "C", "D", "E", "F", "G", "H"};
   EXPECT_THROW(
      {
         registry::YoRegistry registry("test");

         EnumParameter<TestEnum> constantDefault("constantDefault", "", &registry, false, constants);
         constantDefault.loadDefault();

         constantDefault.setToString("NONEXISTANT");
      },
      std::exception);
}

TEST(EnumParameterTest, testStringBasedAccessNullValueConstant)
{
   std::vector<std::string> constants{"A", "B", "C", "NuLl", "E", "F", "G", "H"};
   EXPECT_THROW(
      {
         registry::YoRegistry registry("test");

         EnumParameter<TestEnum> constantDefault("constantDefault", "", &registry, false, constants);
      },
      std::exception);
}

// Java's testStringBasedAccessNullConstant (a null element in the constants array) is not ported:
// std::string has no null state - every element is a valid, if possibly empty, string - so this
// specific case doesn't have a meaningful C++ equivalent to test (same divergence documented in
// YoEnumTest).

TEST(EnumParameterTest, testStringBasedAccessEmptyConstantListNotNull)
{
   EXPECT_THROW(
      {
         registry::YoRegistry registry("test");
         EnumParameter<TestEnum> constantDefault("constantDefault", "", &registry, false, std::vector<std::string>{});
      },
      std::exception);
}

TEST(EnumParameterTest, testStringBasedAccessEmptyConstantList)
{
   registry::YoRegistry registry("test");

   EnumParameter<TestEnum> nullDefault("nullDefault", "", &registry, true, std::vector<std::string>{});
   nullDefault.loadDefault();
   EXPECT_EQ(nullDefault.getValueAsString(), "null");
}
} // namespace
} // namespace ihmc::yovariables::parameters
