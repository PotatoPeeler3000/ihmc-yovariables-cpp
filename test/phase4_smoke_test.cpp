#include <gtest/gtest.h>

#include <sstream>

#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/tools/yo_factories.h"
#include "ihmc/yovariables/tools/yo_geometry_name_tools.h"
#include "ihmc/yovariables/tools/yo_tools.h"
#include "ihmc/yovariables/variable/yo_double.h"

using namespace ihmc::yovariables;

TEST(Phase4Smoke, GeometryNameToolsAssembly)
{
   EXPECT_EQ(tools::createXName("foot", "Position"), "footXPosition");
   EXPECT_EQ(tools::appendSuffix("", "Position"), "position");
   EXPECT_EQ(tools::appendSuffix("foot_", "Position"), "foot_position");
   EXPECT_EQ(tools::getCommonPrefix({"footLeft", "footRight"}), "foot");
   EXPECT_EQ(tools::getCommonSuffix({"leftFoot", "rightFoot"}), "tFoot");
}

TEST(Phase4Smoke, ToShortName)
{
   EXPECT_EQ(tools::toShortName("root.child.leaf"), "leaf");
   EXPECT_EQ(tools::toShortName("noNamespace"), "noNamespace");
}

TEST(Phase4Smoke, GetRegistryInfoAndPrintStatistics)
{
   registry::YoRegistry root("root");
   registry::YoRegistry child("child");
   root.addChild(&child);
   variable::YoDouble x("x", &child);
   variable::YoDouble y("y", &child);

   std::string info = tools::getRegistryInfo(child);
   EXPECT_NE(info.find("Variables: 2"), std::string::npos);
   EXPECT_NE(info.find("child"), std::string::npos);

   std::ostringstream out;
   tools::printStatistics([](const registry::YoRegistry&) { return true; }, root, [](const registry::YoRegistry& r) { return tools::getRegistryInfo(r); },
                           out);

   std::string printed = out.str();
   EXPECT_NE(printed.find("Total number of variables: 2"), std::string::npos);
   EXPECT_NE(printed.find("child"), std::string::npos);
}

TEST(Phase4Smoke, FindOrCreateRegistryCreatesMissingChain)
{
   registry::YoRegistry root("root");
   std::vector<std::unique_ptr<registry::YoRegistry>> ownershipSink;

   registry::YoNamespace target("root.a.b.c");
   registry::YoRegistry* found = tools::findOrCreateRegistry(root, target, ownershipSink);

   ASSERT_NE(found, nullptr);
   EXPECT_EQ(found->getNamespace().getName(), "root.a.b.c");
   EXPECT_EQ(ownershipSink.size(), 3u);

   // Finding it again should not create anything new.
   registry::YoRegistry* foundAgain = tools::findOrCreateRegistry(root, target, ownershipSink);
   EXPECT_EQ(foundAgain, found);
   EXPECT_EQ(ownershipSink.size(), 3u);
}

TEST(Phase4Smoke, FindOrCreateRegistryRejectsIncompatibleNamespace)
{
   registry::YoRegistry root("root");
   std::vector<std::unique_ptr<registry::YoRegistry>> ownershipSink;

   registry::YoNamespace incompatible("somethingElse.a");
   EXPECT_EQ(tools::findOrCreateRegistry(root, incompatible, ownershipSink), nullptr);
   EXPECT_TRUE(ownershipSink.empty());
}
