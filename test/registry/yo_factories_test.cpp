#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/tools/yo_factories.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::registry
{
namespace
{
TEST(YoFactoriesTest, testGetOrCreateAndAddRegistry)
{
   YoRegistry root("root");
   std::vector<std::unique_ptr<YoRegistry>> ownershipSink;

   YoRegistry* registry000 = tools::findOrCreateRegistry(root, YoNamespace("root.registry0.registry00.registry000"), ownershipSink);
   const YoNamespace& namespaceCheck = registry000->getNamespace();
   EXPECT_EQ(YoNamespace("root.registry0.registry00.registry000"), namespaceCheck);

   variable::YoDouble foo("foo", registry000);
   EXPECT_EQ(foo.getFullNameString(), "root.registry0.registry00.registry000.foo");

   YoRegistry* registry010 = tools::findOrCreateRegistry(root, YoNamespace("root.registry0.registry01.registry010"), ownershipSink);
   variable::YoDouble bar("bar", registry010);
   EXPECT_EQ(bar.getFullNameString(), "root.registry0.registry01.registry010.bar");

   EXPECT_EQ(&foo, root.findVariable("foo"));
   EXPECT_EQ(&bar, root.findVariable("bar"));

   EXPECT_EQ(registry000, tools::findOrCreateRegistry(root, YoNamespace("root.registry0.registry00.registry000"), ownershipSink));
   EXPECT_EQ(registry010, tools::findOrCreateRegistry(root, YoNamespace("root.registry0.registry01.registry010"), ownershipSink));
}
} // namespace
} // namespace ihmc::yovariables::registry
