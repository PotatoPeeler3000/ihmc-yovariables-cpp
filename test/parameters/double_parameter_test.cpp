#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "ihmc/yovariables/exceptions/illegal_operation_exception.h"
#include "ihmc/yovariables/listener/yo_parameter_changed_listener.h"
#include "ihmc/yovariables/parameters/double_parameter.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::parameters
{
namespace
{
constexpr double kInitialValue = 42.0;

// Bundles a DoubleParameter with the registries it needs to stay alive alongside it. Every field is
// a unique_ptr, so the struct itself is safely movable (the DoubleParameter object's address never
// changes - only pointers to it move) even though DoubleParameter's own BackingVariable holds a
// reference back to it that would go stale if the parameter itself were relocated.
struct ParamWithNamespace
{
   std::unique_ptr<registry::YoRegistry> root, a, b, c;
   std::unique_ptr<DoubleParameter> param;
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

   result.param = std::make_unique<DoubleParameter>("param", "paramDescription", result.c.get(), kInitialValue, -20.3, 24.5);

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

TEST(DoubleParameterTest, testConstructDefaultValue)
{
   registry::YoRegistry dummy("dummy");
   DoubleParameter test("test", &dummy);
   test.loadDefault();

   EXPECT_TRUE(std::isnan(test.getValue()));
}

TEST(DoubleParameterTest, testDuplicate)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   variable::YoDouble& var = bundle.param->getVariable();
   bundle.param->loadDefault();

   var.set(632.0);

   registry::YoRegistry newRegistry("newRegistry");
   std::unique_ptr<variable::YoVariable> newVarBase = var.duplicate(&newRegistry);
   auto* newVar = dynamic_cast<variable::YoDouble*>(newVarBase.get());
   ASSERT_NE(newVar, nullptr);
   auto* newParam = dynamic_cast<DoubleParameter*>(newVar->getParameter());
   ASSERT_NE(newParam, nullptr);

   EXPECT_EQ(bundle.param->getName(), newParam->getName());
   EXPECT_EQ(bundle.param->getDescription(), newParam->getDescription());
   EXPECT_NEAR(bundle.param->getValue(), newParam->getValue(), 1e-9);
   EXPECT_NEAR(var.getLowerBound(), newVar->getLowerBound(), 1e-9);
   EXPECT_NEAR(var.getUpperBound(), newVar->getUpperBound(), 1e-9);
}

TEST(DoubleParameterTest, testGetNamespace)
{
   ParamWithNamespace bundle = createParameterWithNamespace();

   EXPECT_EQ(bundle.param->getNamespace()->toString(), "root.a.b.c");
   EXPECT_EQ(bundle.param->getName(), "param");
}

TEST(DoubleParameterTest, testLoadFromString)
{
   for (double s = 0; s < 100.0; s += 1.153165)
   {
      registry::YoRegistry dummy("dummy");
      DoubleParameter param("test", &dummy);
      param.load(std::to_string(s));

      EXPECT_NEAR(s, param.getValue(), 1e-9);
      EXPECT_EQ(std::to_string(s), param.getValueAsString());
   }
}

TEST(DoubleParameterTest, testGetBeforeLoad)
{
   EXPECT_THROW(
      {
         ParamWithNamespace bundle = createParameterWithNamespace();
         bundle.param->getValue();
      },
      exceptions::IllegalOperationException);
}

TEST(DoubleParameterTest, testDefault)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   bundle.param->loadDefault();
   EXPECT_NEAR(kInitialValue, bundle.param->getValue(), 1e-9);
}

TEST(DoubleParameterTest, testListener)
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
