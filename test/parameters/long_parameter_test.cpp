#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "ihmc/yovariables/exceptions/illegal_operation_exception.h"
#include "ihmc/yovariables/listener/yo_parameter_changed_listener.h"
#include "ihmc/yovariables/parameters/long_parameter.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_long.h"

namespace ihmc::yovariables::parameters
{
namespace
{
constexpr std::int64_t kInitialValue = 42;

struct ParamWithNamespace
{
   std::unique_ptr<registry::YoRegistry> root, a, b, c;
   std::unique_ptr<LongParameter> param;
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

   result.param = std::make_unique<LongParameter>("param", "paramDescription", result.c.get(), kInitialValue, -102024, 294);

   return result;
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

TEST(LongParameterTest, testConstructDefaultValue)
{
   registry::YoRegistry dummy("dummy");
   LongParameter test("test", &dummy);
   test.loadDefault();

   EXPECT_EQ(0, test.getValue());
}

TEST(LongParameterTest, testDuplicate)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   variable::YoLong& var = bundle.param->getVariable();
   bundle.param->loadDefault();

   var.set(632);

   registry::YoRegistry newRegistry("newRegistry");
   std::unique_ptr<variable::YoVariable> newVarBase = var.duplicate(&newRegistry);
   auto* newVar = dynamic_cast<variable::YoLong*>(newVarBase.get());
   ASSERT_NE(newVar, nullptr);
   auto* newParam = dynamic_cast<LongParameter*>(newVar->getParameter());
   ASSERT_NE(newParam, nullptr);

   EXPECT_EQ(bundle.param->getName(), newParam->getName());
   EXPECT_EQ(bundle.param->getDescription(), newParam->getDescription());
   EXPECT_NEAR(static_cast<double>(bundle.param->getValue()), static_cast<double>(newParam->getValue()), 1e-9);
   EXPECT_NEAR(var.getLowerBound(), newVar->getLowerBound(), 1e-9);
   EXPECT_NEAR(var.getUpperBound(), newVar->getUpperBound(), 1e-9);
}

TEST(LongParameterTest, testGetNamespace)
{
   ParamWithNamespace bundle = createParameterWithNamespace();

   EXPECT_EQ(bundle.param->getNamespace()->toString(), "root.a.b.c");
   EXPECT_EQ(bundle.param->getName(), "param");
}

TEST(LongParameterTest, testLoadFromString)
{
   const std::int64_t step = std::numeric_limits<std::int32_t>::max();
   for (std::int64_t s = 100LL * std::numeric_limits<std::int32_t>::min(); s < 100LL * std::numeric_limits<std::int32_t>::max(); s += step)
   {
      registry::YoRegistry dummy("dummy");
      LongParameter param("test", &dummy);
      param.load(std::to_string(s));

      EXPECT_EQ(s, param.getValue());
      EXPECT_EQ(std::to_string(s), param.getValueAsString());
   }
}

TEST(LongParameterTest, testGetBeforeLoad)
{
   EXPECT_THROW(
      {
         ParamWithNamespace bundle = createParameterWithNamespace();
         bundle.param->getValue();
      },
      exceptions::IllegalOperationException);
}

TEST(LongParameterTest, testDefault)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   bundle.param->loadDefault();
   EXPECT_EQ(kInitialValue, bundle.param->getValue());
}

TEST(LongParameterTest, testListener)
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

   bundle.param->getVariable().setValueFromDouble(1.0);

   EXPECT_TRUE(callback.set);
}
} // namespace
} // namespace ihmc::yovariables::parameters
