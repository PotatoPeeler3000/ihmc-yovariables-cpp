#include <gtest/gtest.h>

#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ihmc/yovariables/parameters/abstract_parameter_reader.h"
#include "ihmc/yovariables/parameters/double_parameter.h"
#include "ihmc/yovariables/parameters/parameter_data.h"
#include "ihmc/yovariables/registry/yo_registry.h"

namespace ihmc::yovariables::parameters
{
namespace
{
// Java's Double.toString() produces the shortest decimal string that round-trips exactly back to
// the original double, which this test relies on (it compares values with a delta of
// Double.MIN_VALUE - effectively exact equality). std::to_string() only emits 6 decimal digits and
// would lose precision here; 17 significant digits is the standard bound that guarantees an exact
// round trip for any IEEE-754 double, which is all this helper needs to provide (it feeds
// DoubleParameter::load, which parses via std::stod regardless of the string's exact format).
std::string toExactString(double value)
{
   std::ostringstream stream;
   stream << std::setprecision(17) << value;
   return stream.str();
}

class TestParameterReader : public AbstractParameterReader
{
public:
   explicit TestParameterReader(std::unordered_map<std::string, ParameterData> values) : values_(std::move(values))
   {
   }

protected:
   const std::unordered_map<std::string, ParameterData>& getValues() const override
   {
      return values_;
   }

private:
   std::unordered_map<std::string, ParameterData> values_;
};

TEST(AbstractParameterReaderTest, testReadingNamespacesRegistry)
{
   std::mt19937 random(9492U);
   std::uniform_real_distribution<double> valueDist(0.0, 1.0);
   std::uniform_int_distribution<int> countDist(0, 9);

   for (int i = 0; i < 4; i++)
   {
      std::unique_ptr<registry::YoRegistry> regs[4] = {std::make_unique<registry::YoRegistry>("root"),
                                                         std::make_unique<registry::YoRegistry>("a"),
                                                         std::make_unique<registry::YoRegistry>("b"),
                                                         std::make_unique<registry::YoRegistry>("c")};

      std::string expectedNamespaces[4] = {"root.a.b.c", "a.b.c", "b.c", "c"};

      regs[0]->addChild(regs[1].get());
      regs[1]->addChild(regs[2].get());
      regs[2]->addChild(regs[3].get());

      DoubleParameter parameter("parameter", regs[3].get(), valueDist(random));
      int numberOfDefaults = countDist(random);
      std::vector<std::unique_ptr<DoubleParameter>> defaultParams;
      for (int defaultIdx = 0; defaultIdx < numberOfDefaults; defaultIdx++)
         defaultParams.push_back(std::make_unique<DoubleParameter>("Default" + std::to_string(defaultIdx), regs[3].get(), 0.0));

      std::unordered_map<std::string, ParameterData> values;
      double expectedValue = valueDist(random);
      values.emplace(expectedNamespaces[i] + "." + parameter.getName(), ParameterData(toExactString(expectedValue)));

      int numberOfUnmatched = countDist(random);
      for (int unmatchedIdx = 0; unmatchedIdx < numberOfUnmatched; unmatchedIdx++)
         values.emplace("Unmatched" + std::to_string(unmatchedIdx), ParameterData(toExactString(0.0)));

      TestParameterReader readerRoot(values);
      std::unordered_set<std::string> defaultParameters;
      std::unordered_set<std::string> unmatchedParameters;
      readerRoot.readParametersInRegistry(*regs[i], defaultParameters, unmatchedParameters);

      EXPECT_EQ(static_cast<std::size_t>(numberOfDefaults), defaultParameters.size());
      EXPECT_EQ(static_cast<std::size_t>(numberOfUnmatched), unmatchedParameters.size());
      EXPECT_EQ(expectedValue, parameter.getValue());
   }
}

TEST(AbstractParameterReaderTest, testDoubleRead)
{
   std::mt19937 random(9492U);
   std::uniform_real_distribution<double> valueDist(0.0, 1.0);

   registry::YoRegistry root("root");
   registry::YoRegistry a("a");

   root.addChild(&a);
   DoubleParameter parameter("param", &a, valueDist(random));

   double loadedValue = valueDist(random);

   std::unordered_map<std::string, ParameterData> values;
   values.emplace("root.a.param", ParameterData(toExactString(loadedValue)));
   TestParameterReader reader(values);

   registry::YoRegistry randomRegistry("RandomRegistry");
   reader.readParametersInRegistry(randomRegistry);
   reader.readParametersInRegistry(root);
   EXPECT_EQ(loadedValue, parameter.getValue());
}
} // namespace
} // namespace ihmc::yovariables::parameters
