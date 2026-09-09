#include <gtest/gtest.h>

#include "ihmc/yovariables/parameters/default_parameter_reader.h"
#include "ihmc/yovariables/parameters/double_parameter.h"
#include "ihmc/yovariables/registry/yo_registry.h"

namespace ihmc::yovariables::parameters
{
namespace
{
constexpr double kInitialValue = 42.0;

TEST(DefaultParameterReaderTest, testReadDefault)
{
   registry::YoRegistry root("root");
   registry::YoRegistry a("a");

   root.addChild(&a);
   DoubleParameter param("param", &a, kInitialValue);

   DefaultParameterReader reader;
   reader.readParametersInRegistry(root);

   EXPECT_NEAR(kInitialValue, param.getValue(), 1e-9);
}
} // namespace
} // namespace ihmc::yovariables::parameters
