#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include "ihmc/yovariables/exceptions/illegal_operation_exception.h"
#include "ihmc/yovariables/listener/yo_parameter_changed_listener.h"
#include "ihmc/yovariables/parameters/boolean_parameter.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"

namespace ihmc::yovariables::parameters
{
namespace
{
constexpr bool kInitialValue = true;

struct ParamWithNamespace
{
   std::unique_ptr<registry::YoRegistry> root, a, b, c;
   std::unique_ptr<BooleanParameter> param;
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

   result.param = std::make_unique<BooleanParameter>("param", "paramDescription", result.c.get(), kInitialValue);

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

TEST(BooleanParameterTest, testGetNamespace)
{
   ParamWithNamespace bundle = createParameterWithNamespace();

   EXPECT_EQ(bundle.param->getNamespace()->toString(), "root.a.b.c");
   EXPECT_EQ(bundle.param->getName(), "param");
}

TEST(BooleanParameterTest, testGetBeforeLoad)
{
   EXPECT_THROW(
      {
         ParamWithNamespace bundle = createParameterWithNamespace();
         bundle.param->getValue();
      },
      exceptions::IllegalOperationException);
}

TEST(BooleanParameterTest, testLoadFromString)
{
   std::vector<std::string> options{"false", "true", "FALSE", "TRUE", "False", "True"};

   for (std::size_t i = 0; i < options.size(); i++)
   {
      registry::YoRegistry dummy("dummy");
      BooleanParameter param("test", &dummy);
      param.load(options[i]);

      EXPECT_EQ(i % 2 == 1, param.getValue());
      std::string lower = options[i];
      std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) { return std::tolower(ch); });
      EXPECT_EQ(lower, param.getValueAsString());
   }
}

TEST(BooleanParameterTest, testDefault)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   bundle.param->loadDefault();
   EXPECT_EQ(kInitialValue, bundle.param->getValue());
}

TEST(BooleanParameterTest, testDuplicate)
{
   ParamWithNamespace bundle = createParameterWithNamespace();
   variable::YoBoolean& var = bundle.param->getVariable();
   bundle.param->loadDefault();

   var.set(true);

   registry::YoRegistry newRegistry("newRegistry");
   std::unique_ptr<variable::YoVariable> newVarBase = var.duplicate(&newRegistry);
   auto* newVar = dynamic_cast<variable::YoBoolean*>(newVarBase.get());
   ASSERT_NE(newVar, nullptr);
   auto* newParam = dynamic_cast<BooleanParameter*>(newVar->getParameter());
   ASSERT_NE(newParam, nullptr);

   EXPECT_EQ(bundle.param->getName(), newParam->getName());
   EXPECT_EQ(bundle.param->getDescription(), newParam->getDescription());
   EXPECT_EQ(bundle.param->getValue(), newParam->getValue());
}

TEST(BooleanParameterTest, testListener)
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

   bundle.param->getVariable().setValueFromDouble(0.0);

   EXPECT_TRUE(callback.set);
}
} // namespace
} // namespace ihmc::yovariables::parameters
