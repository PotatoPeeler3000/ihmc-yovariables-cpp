#include <gtest/gtest.h>

#include <sstream>

#include "ihmc/yovariables/parameters/boolean_parameter.h"
#include "ihmc/yovariables/parameters/default_parameter_reader.h"
#include "ihmc/yovariables/parameters/double_parameter.h"
#include "ihmc/yovariables/parameters/enum_parameter.h"
#include "ihmc/yovariables/parameters/integer_parameter.h"
#include "ihmc/yovariables/parameters/long_parameter.h"
#include "ihmc/yovariables/parameters/xml_parameter_reader.h"
#include "ihmc/yovariables/parameters/xml_parameter_writer.h"
#include "ihmc/yovariables/registry/yo_registry.h"

using namespace ihmc::yovariables;

enum class Gain
{
   LOW,
   MEDIUM,
   HIGH
};

TEST(Phase3Smoke, ParameterRequiresLoadBeforeUse)
{
   registry::YoRegistry root("root");
   parameters::DoubleParameter kp("kp", &root, 2.5);

   EXPECT_FALSE(kp.isLoaded());
   EXPECT_THROW(kp.getValue(), std::exception);

   kp.loadDefault();
   EXPECT_TRUE(kp.isLoaded());
   EXPECT_DOUBLE_EQ(kp.getValue(), 2.5);
}

TEST(Phase3Smoke, DefaultParameterReaderLoadsAllToDefault)
{
   registry::YoRegistry root("root");
   parameters::BooleanParameter flag("flag", &root, true);
   parameters::IntegerParameter count("count", &root, 7);
   parameters::LongParameter big("big", &root, static_cast<std::int64_t>(123456789012));

   parameters::DefaultParameterReader reader;
   reader.readParametersInRegistry(root);

   EXPECT_TRUE(flag.getValue());
   EXPECT_EQ(count.getValue(), 7);
   EXPECT_EQ(big.getValue(), 123456789012);
}

TEST(Phase3Smoke, EnumParameterBackedByRealEnum)
{
   registry::YoRegistry root("root");
   parameters::EnumParameter<Gain> gain("gain", &root, false);
   gain.loadDefault();

   EXPECT_EQ(gain.getValue(), Gain::LOW);
   EXPECT_EQ(gain.getEnumSize(), 3);
}

TEST(Phase3Smoke, DuplicateProducesIndependentParameter)
{
   registry::YoRegistry root("root");
   registry::YoRegistry clonedRoot("clonedRoot");

   parameters::DoubleParameter kp("kp", &root, 1.0, 0.0, 10.0);
   kp.loadDefault();
   kp.getVariable().set(4.0);

   std::unique_ptr<variable::YoVariable> duplicate = kp.getVariable().duplicate(&clonedRoot);

   auto* duplicateParameter = dynamic_cast<parameters::DoubleParameter*>(duplicate->getParameter());
   ASSERT_NE(duplicateParameter, nullptr);
   EXPECT_TRUE(duplicateParameter->isLoaded());
   EXPECT_DOUBLE_EQ(duplicateParameter->getValue(), 4.0);

   // Independent: mutating the original doesn't affect the clone.
   kp.getVariable().set(9.0);
   EXPECT_DOUBLE_EQ(duplicateParameter->getValue(), 4.0);
}

TEST(Phase3Smoke, XmlWriterReaderRoundTrip)
{
   registry::YoRegistry root("robot");
   parameters::DoubleParameter kp("kp", "proportional gain", &root, 1.0, 0.0, 10.0);
   parameters::BooleanParameter enabled("enabled", &root, false);

   parameters::DefaultParameterReader defaultReader;
   defaultReader.readParametersInRegistry(root);
   kp.getVariable().set(3.75);
   enabled.getVariable().set(true);

   parameters::XmlParameterWriter writer;
   writer.addParameters(root);

   std::ostringstream xmlOut;
   writer.write(xmlOut);

   std::istringstream xmlIn(xmlOut.str());
   parameters::XmlParameterReader reader({&xmlIn});

   registry::YoRegistry freshRoot("robot");
   parameters::DoubleParameter freshKp("kp", "proportional gain", &freshRoot, 1.0, 0.0, 10.0);
   parameters::BooleanParameter freshEnabled("enabled", &freshRoot, false);

   reader.readParametersInRegistry(freshRoot);

   EXPECT_DOUBLE_EQ(freshKp.getValue(), 3.75);
   EXPECT_TRUE(freshEnabled.getValue());
}
