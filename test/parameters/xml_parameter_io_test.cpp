#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>

#include "ihmc/yovariables/parameters/default_parameter_reader.h"
#include "ihmc/yovariables/parameters/double_parameter.h"
#include "ihmc/yovariables/parameters/parameter_load_status.h"
#include "ihmc/yovariables/parameters/xml_parameter_reader.h"
#include "ihmc/yovariables/parameters/xml_parameter_writer.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::parameters
{
namespace
{
struct RegistryBundle
{
   std::unique_ptr<registry::YoRegistry> root, a, b, c;
   std::unique_ptr<DoubleParameter> paramA, paramB, paramC, paramD, paramE, paramF, paramG;
};

RegistryBundle createRegistries()
{
   RegistryBundle result;
   result.root = std::make_unique<registry::YoRegistry>("root");
   result.a = std::make_unique<registry::YoRegistry>("a");
   result.b = std::make_unique<registry::YoRegistry>("b");
   result.c = std::make_unique<registry::YoRegistry>("c");

   result.root->addChild(result.a.get());
   result.a->addChild(result.b.get());
   result.b->addChild(result.c.get());

   result.paramA = std::make_unique<DoubleParameter>("paramA", "parameter A description", result.c.get(), 0.0);
   result.paramB = std::make_unique<DoubleParameter>("paramB", result.c.get(), 0.0);
   result.paramC = std::make_unique<DoubleParameter>("paramC", result.c.get(), 0.0);
   result.paramD = std::make_unique<DoubleParameter>("paramD", result.c.get(), 0.0);
   result.paramE = std::make_unique<DoubleParameter>("paramE", result.b.get(), 0.0);
   result.paramF = std::make_unique<DoubleParameter>("paramF", result.a.get(), 0.0);
   result.paramG = std::make_unique<DoubleParameter>("paramG", result.root.get(), 0.0);

   return result;
}

TEST(XmlParameterIoTest, testEmptyFile)
{
   RegistryBundle target = createRegistries();
   std::string data = "<parameters/>";
   std::istringstream stream(data);

   XmlParameterReader parameterReader({&stream});
   parameterReader.readParametersInRegistry(*target.root);
}

TEST(XmlParameterIoTest, testWritingAndReading)
{
   RegistryBundle source = createRegistries();
   RegistryBundle target = createRegistries();

   DefaultParameterReader defaultReader;
   defaultReader.readParametersInRegistry(*source.root);

   source.root->findVariable("root.a.b.c.paramA")->setValueFromDouble(1.0);
   source.root->findVariable("root.a.b.c.paramB")->setValueFromDouble(2.0);
   source.root->findVariable("root.a.b.c.paramC")->setValueFromDouble(3.0);
   source.root->findVariable("root.a.b.c.paramD")->setValueFromDouble(4.0);
   source.root->findVariable("root.a.b.paramE")->setValueFromDouble(5.0);
   source.root->findVariable("root.a.paramF")->setValueFromDouble(6.0);
   source.root->findVariable("root.paramG")->setValueFromDouble(7.0);

   std::ostringstream os;

   XmlParameterWriter writer;
   writer.addParameters(*source.root);
   writer.write(os);

   std::istringstream is(os.str());
   XmlParameterReader reader({&is});
   reader.readParametersInRegistry(*target.root);

   EXPECT_NEAR(source.root->findVariable("root.a.b.c.paramA")->getValueAsDouble(), target.root->findVariable("root.a.b.c.paramA")->getValueAsDouble(),
               1e-9);
   EXPECT_NEAR(source.root->findVariable("root.a.b.c.paramB")->getValueAsDouble(), target.root->findVariable("root.a.b.c.paramB")->getValueAsDouble(),
               1e-9);
   EXPECT_NEAR(source.root->findVariable("root.a.b.c.paramC")->getValueAsDouble(), target.root->findVariable("root.a.b.c.paramC")->getValueAsDouble(),
               1e-9);
   EXPECT_NEAR(source.root->findVariable("root.a.b.c.paramD")->getValueAsDouble(), target.root->findVariable("root.a.b.c.paramD")->getValueAsDouble(),
               1e-9);
   EXPECT_NEAR(source.root->findVariable("root.a.b.paramE")->getValueAsDouble(), target.root->findVariable("root.a.b.paramE")->getValueAsDouble(), 1e-9);
   EXPECT_NEAR(source.root->findVariable("root.a.paramF")->getValueAsDouble(), target.root->findVariable("root.a.paramF")->getValueAsDouble(), 1e-9);
   EXPECT_NEAR(source.root->findVariable("root.paramG")->getValueAsDouble(), target.root->findVariable("root.paramG")->getValueAsDouble(), 1e-9);
}

TEST(XmlParameterIoTest, testOverwritingDuringContruction)
{
   registry::YoRegistry target("TestRegistry");
   DoubleParameter parameter1("TestParameter1", &target);
   DoubleParameter parameter2("TestParameter2", &target);

   std::string data1 = "<parameters><registry name=\"TestRegistry\">"
                        "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>"
                        "<parameter name=\"TestParameter2\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/></registry></parameters>";
   std::istringstream stream1(data1);

   std::string data2 = "<parameters><registry name=\"TestRegistry\">"
                        "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.0\"/></registry></parameters>";
   std::istringstream stream2(data2);

   XmlParameterReader parameterReader({&stream1, &stream2});
   parameterReader.readParametersInRegistry(target);

   EXPECT_EQ(0.0, parameter1.getValue());
   EXPECT_EQ(0.5, parameter2.getValue());
}

TEST(XmlParameterIoTest, testOverwriting)
{
   registry::YoRegistry target("TestRegistry");
   DoubleParameter parameter1("TestParameter1", &target);
   DoubleParameter parameter2("TestParameter2", &target);

   std::string data1 = "<parameters><registry name=\"TestRegistry\">"
                        "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>"
                        "<parameter name=\"TestParameter2\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/></registry></parameters>";
   std::istringstream stream1(data1);

   std::string data2 = "<parameters><registry name=\"TestRegistry\">"
                        "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.0\"/></registry></parameters>";
   std::istringstream stream2(data2);

   XmlParameterReader parameterReader({&stream1});
   parameterReader.overwrite({&stream2});
   parameterReader.readParametersInRegistry(target);

   EXPECT_EQ(0.0, parameter1.getValue());
   EXPECT_EQ(0.5, parameter2.getValue());
}

TEST(XmlParameterIoTest, testOverwritingAvoidingException)
{
   registry::YoRegistry target("TestRegistry");
   DoubleParameter parameter1("TestParameter1", &target);
   DoubleParameter parameter2("TestParameter2", &target);
   DoubleParameter parameter3("TestParameter3", &target);

   std::string data1 = "<parameters><registry name=\"TestRegistry\">"
                        "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>"
                        "<parameter name=\"TestParameter2\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/></registry></parameters>";
   std::istringstream stream1(data1);

   std::string data2 = "<parameters><registry name=\"TestRegistry\">"
                        "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.0\"/>"
                        "<parameter name=\"TestParameter3\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.3\"/></registry></parameters>";
   std::istringstream stream2(data2);

   XmlParameterReader parameterReader({&stream1});
   parameterReader.readAndOverwrite({&stream2});
   parameterReader.readParametersInRegistry(target);

   EXPECT_EQ(0.0, parameter1.getValue());
   EXPECT_EQ(0.5, parameter2.getValue());
   EXPECT_EQ(0.3, parameter3.getValue());
}

TEST(XmlParameterIoTest, testOverwritingFails)
{
   EXPECT_THROW(
      {
         registry::YoRegistry target("TestRegistry");
         DoubleParameter parameter1("TestParameter1", &target);
         DoubleParameter parameter2("TestParameter2", &target);

         std::string data1 = "<parameters><registry name=\"TestRegistry\">"
                              "<parameter name=\"TestParameter2\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/></registry></parameters>";
         std::istringstream stream1(data1);

         std::string data2 = "<parameters><registry name=\"TestRegistry\">"
                              "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.0\"/></registry></parameters>";
         std::istringstream stream2(data2);

         XmlParameterReader parameterReader({&stream1});
         parameterReader.overwrite({&stream2});
         parameterReader.readParametersInRegistry(target);
      },
      std::runtime_error);
}

TEST(XmlParameterIoTest, testRootNamespaceDoesNotMatch)
{
   registry::YoRegistry target("Root");
   DoubleParameter parameter("TestParameter", &target);

   std::string data1 = "<parameters><registry name=\"" + target.getName() + "\">" + "<parameter name=\"" + parameter.getName()
                        + "\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/></registry></parameters>";
   std::istringstream stream1(data1);

   XmlParameterReader parameterReader({&stream1}, std::string("SomeOtherRoot"));
   parameterReader.readParametersInRegistry(target);
   EXPECT_EQ(parameter.getLoadStatus(), ParameterLoadStatus::DEFAULT);
}

TEST(XmlParameterIoTest, testRootNamespaceMatches)
{
   registry::YoRegistry target("Root");
   DoubleParameter parameter("TestParameter", &target);

   std::string data1 = "<parameters><registry name=\"" + target.getName() + "\">" + "<parameter name=\"" + parameter.getName()
                        + "\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/></registry></parameters>";
   std::istringstream stream1(data1);

   XmlParameterReader parameterReader({&stream1}, target.getName());
   parameterReader.readParametersInRegistry(target);
   EXPECT_EQ(parameter.getLoadStatus(), ParameterLoadStatus::LOADED);
}

TEST(XmlParameterIoTest, testReadingWithMinMax)
{
   registry::YoRegistry target("Root");
   DoubleParameter parameter("TestParameter", &target);

   double min = -0.4536;
   double max = 9509.3;

   std::string data1 = "<parameters><registry name=\"" + target.getName() + "\">" + "<parameter name=\"" + parameter.getName() + "\" type=\"DoubleParameter\" min=\""
                        + std::to_string(min) + "\" max=\"" + std::to_string(max) + "\" value=\"0.5\"/></registry></parameters>";
   std::istringstream stream1(data1);

   XmlParameterReader parameterReader({&stream1}, target.getName());
   parameterReader.readParametersInRegistry(target);
   EXPECT_EQ(parameter.getLoadStatus(), ParameterLoadStatus::LOADED);
   EXPECT_NEAR(min, parameter.getLowerBound(), 1e-6);
   EXPECT_NEAR(max, parameter.getUpperBound(), 1e-6);
}

TEST(XmlParameterIoTest, testReadingWithoutMinMax)
{
   registry::YoRegistry target("Root");
   DoubleParameter parameter("TestParameter", &target);

   std::string data1 = "<parameters><registry name=\"" + target.getName() + "\">" + "<parameter name=\"" + parameter.getName()
                        + "\" type=\"BooleanParameter\" value=\"0.5\"/></registry></parameters>";
   std::istringstream stream1(data1);

   XmlParameterReader parameterReader({&stream1}, target.getName());
   parameterReader.readParametersInRegistry(target);
   EXPECT_EQ(parameter.getLoadStatus(), ParameterLoadStatus::LOADED);
}
} // namespace
} // namespace ihmc::yovariables::parameters
