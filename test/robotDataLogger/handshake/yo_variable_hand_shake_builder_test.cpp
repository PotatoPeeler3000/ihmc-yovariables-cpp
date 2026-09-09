#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "ihmc/robotDataLogger/handshake/json_serialization.h"
#include "ihmc/robotDataLogger/handshake/yo_variable_hand_shake_builder.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"

using namespace ihmc::yovariables;

namespace ihmc::robotDataLogger::handshake
{
namespace
{
enum class HandshakeTestEnum
{
   ONE,
   TWO
};

TEST(YoVariableHandShakeBuilderTest, testRegistryIdsAndPreOrderVariableOrdering)
{
   registry::YoRegistry main("Main");
   variable::YoDouble d1("d1", &main);

   registry::YoRegistry child("Child");
   main.addChild(&child);
   variable::YoBoolean b1("b1", &child);
   variable::YoEnum<HandshakeTestEnum> e1("e1", &child, false);

   YoVariableHandShakeBuilder builder("ServerRoot", 0.001);
   builder.build(main);

   const Handshake& handshake = builder.getHandshake();

   // registries[0] is the synthetic root from the constructor, registries[1] is "Main" (the
   // registry passed to build()), registries[2] is "Child" - exactly Java's numbering.
   ASSERT_EQ(3u, handshake.registries.size());
   EXPECT_EQ("ServerRoot", handshake.registries[0].name);
   EXPECT_EQ(0, handshake.registries[0].parent);
   EXPECT_EQ("Main", handshake.registries[1].name);
   EXPECT_EQ(0, handshake.registries[1].parent);
   EXPECT_EQ("Child", handshake.registries[2].name);
   EXPECT_EQ(1, handshake.registries[2].parent);

   // Pre-order: Main's own variable (d1) comes before Child's variables (b1, e1).
   ASSERT_EQ(3u, handshake.variables.size());
   EXPECT_EQ("d1", handshake.variables[0].name);
   EXPECT_EQ(YoType::DoubleYoVariable, handshake.variables[0].type);
   EXPECT_EQ(1, handshake.variables[0].registry);

   EXPECT_EQ("b1", handshake.variables[1].name);
   EXPECT_EQ(YoType::BooleanYoVariable, handshake.variables[1].type);
   EXPECT_EQ(2, handshake.variables[1].registry);

   EXPECT_EQ("e1", handshake.variables[2].name);
   EXPECT_EQ(YoType::EnumYoVariable, handshake.variables[2].type);
   EXPECT_EQ(2, handshake.variables[2].registry);
   EXPECT_FALSE(handshake.variables[2].allowNullValues);

   ASSERT_EQ(1u, handshake.enumTypes.size());
   EXPECT_EQ(std::vector<std::string>({"ONE", "TWO"}), handshake.enumTypes[0].enumValues);
   EXPECT_EQ(0, handshake.variables[2].enumType);

   // The wire-value order must match the handshake's variable order exactly, by identity.
   const std::vector<variable::YoVariable*>& wireOrder = builder.getVariablesInWireOrder();
   ASSERT_EQ(3u, wireOrder.size());
   EXPECT_EQ(static_cast<variable::YoVariable*>(&d1), wireOrder[0]);
   EXPECT_EQ(static_cast<variable::YoVariable*>(&b1), wireOrder[1]);
   EXPECT_EQ(static_cast<variable::YoVariable*>(&e1), wireOrder[2]);
}

TEST(YoVariableHandShakeBuilderTest, testEnumTypeDedupSharesOneEntryAcrossVariables)
{
   registry::YoRegistry root("Root");
   variable::YoEnum<HandshakeTestEnum> e1("e1", &root, false);
   variable::YoEnum<HandshakeTestEnum> e2("e2", &root, false);

   YoVariableHandShakeBuilder builder("ServerRoot", 0.001);
   builder.build(root);

   const Handshake& handshake = builder.getHandshake();
   ASSERT_EQ(1u, handshake.enumTypes.size()) << "both variables are backed by the same enum type, so should dedupe to one EnumType entry";
   EXPECT_EQ(handshake.variables[0].enumType, handshake.variables[1].enumType);
}

TEST(YoVariableHandShakeBuilderTest, testNonBackedEnumNeverDedupes)
{
   registry::YoRegistry root("Root");
   variable::YoEnum<HandshakeTestEnum> e1("e1", "", &root, false, std::vector<std::string>{"A", "B"});
   variable::YoEnum<HandshakeTestEnum> e2("e2", "", &root, false, std::vector<std::string>{"A", "B"});

   YoVariableHandShakeBuilder builder("ServerRoot", 0.001);
   builder.build(root);

   const Handshake& handshake = builder.getHandshake();
   // Mirrors Java: non-backed (string-constant) enums key off the variable's own full name, so
   // identical constant lists on different variables never share an EnumType entry.
   ASSERT_EQ(2u, handshake.enumTypes.size());
   EXPECT_NE(handshake.variables[0].enumType, handshake.variables[1].enumType);
}

TEST(YoVariableHandShakeBuilderTest, testJsonShapeMatchesJavaWireFormat)
{
   registry::YoRegistry root("Main");
   variable::YoDouble d1("d1", "a description", &root);
   d1.setVariableBounds(-1.0, 1.0);

   YoVariableHandShakeBuilder builder("ServerRoot", 0.002);
   builder.build(root);

   nlohmann::json parsed = nlohmann::json::parse(toJsonString(builder.getHandshake()));

   ASSERT_TRUE(parsed.contains("us::ihmc::robotDataLogger::Handshake"));
   const nlohmann::json& body = parsed["us::ihmc::robotDataLogger::Handshake"];

   EXPECT_DOUBLE_EQ(0.002, body["dt"].get<double>());
   ASSERT_TRUE(body["registries"].is_array());
   EXPECT_EQ(2u, body["registries"].size());
   EXPECT_EQ("Main", body["registries"][1]["name"].get<std::string>());

   ASSERT_TRUE(body["variables"].is_array());
   ASSERT_EQ(1u, body["variables"].size());
   const nlohmann::json& variableJson = body["variables"][0];
   EXPECT_EQ("d1", variableJson["name"].get<std::string>());
   EXPECT_EQ("a description", variableJson["description"].get<std::string>());
   EXPECT_EQ("DoubleYoVariable", variableJson["type"].get<std::string>());
   EXPECT_EQ("NoParameter", variableJson["loadStatus"].get<std::string>());
   EXPECT_DOUBLE_EQ(-1.0, variableJson["min"].get<double>());
   EXPECT_DOUBLE_EQ(1.0, variableJson["max"].get<double>());

   // Java always emits these keys even when empty (every sequence getter is serialized).
   EXPECT_TRUE(body["joints"].is_array());
   EXPECT_TRUE(body["graphicObjects"].is_array());
   EXPECT_TRUE(body["artifacts"].is_array());
   EXPECT_TRUE(body["scs2YoGraphicDefinitions"].is_array());
   EXPECT_TRUE(body["enumTypes"].is_array());
   EXPECT_TRUE(body.contains("referenceFrameInformation"));
   EXPECT_TRUE(body.contains("summary"));
}

TEST(YoVariableHandShakeBuilderTest, testAnnouncementJsonShape)
{
   Announcement announcement;
   announcement.identifier = "test-id";
   announcement.name = "TestServer";
   announcement.hostName = "test-host";
   announcement.reconnectKey = "test-key";
   announcement.log = false;
   announcement.modelFileDescription.hasModel = false;

   nlohmann::json parsed = nlohmann::json::parse(toJsonString(announcement));
   ASSERT_TRUE(parsed.contains("us::ihmc::robotDataLogger::Announcement"));
   const nlohmann::json& body = parsed["us::ihmc::robotDataLogger::Announcement"];
   EXPECT_EQ("TestServer", body["name"].get<std::string>());
   EXPECT_EQ("test-host", body["hostName"].get<std::string>());
   EXPECT_FALSE(body["modelFileDescription"]["hasModel"].get<bool>());
}
} // namespace
}
