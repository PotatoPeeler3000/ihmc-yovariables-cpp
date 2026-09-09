#include <gtest/gtest.h>

#include <vector>

#include "ihmc/yovariables/filters/second_order_filtered_yo_double.h"
#include "ihmc/yovariables/filters/second_order_filter_type.h"
#include "ihmc/yovariables/registry/yo_registry.h"

namespace ihmc::yovariables::filters
{
namespace
{
void expectArrayNear(const std::vector<double>& actual, const std::vector<double>& expected, double tolerance)
{
   ASSERT_EQ(actual.size(), expected.size());
   for (std::size_t i = 0; i < expected.size(); i++)
      EXPECT_NEAR(actual[i], expected[i], tolerance) << "at index " << i;
}

TEST(SecondOrderFilteredYoDoubleTest, testLowPassFilterCoefficients)
{
   registry::YoRegistry registry("SecondOrderFilteredYoDoubleTest");
   double dt = 0.001;
   double dampingRatio = 1.0;
   double naturalFrequencyInHz = 10.0;
   std::vector<double> bAssert{3947.8417604357433, 7895.6835208714865, 3947.8417604357433};
   std::vector<double> aAssert{4255275.254047619, -7992104.316479129, 3752620.429473252};

   SecondOrderFilteredYoDouble filteredYoVariable("lowPass", &registry, dt, naturalFrequencyInHz, dampingRatio, SecondOrderFilterType::LOW_PASS);

   std::vector<double> b(3), a(3);
   filteredYoVariable.getFilterCoefficients(b, a);
   expectArrayNear(b, bAssert, 1e-8);
   expectArrayNear(a, aAssert, 1e-8);
}

TEST(SecondOrderFilteredYoDoubleTest, testNotchFilterCoefficients)
{
   registry::YoRegistry registry("SecondOrderFilteredYoDoubleTest");
   double dt = 0.001;
   double dampingRatio = 1.0;
   double naturalFrequencyInHz = 10.0;
   std::vector<double> bAssert{4003947.8417604356, -7992104.316479129, 4003947.8417604356};
   std::vector<double> aAssert{4255275.254047619, -7992104.316479129, 3752620.429473252};

   SecondOrderFilteredYoDouble filteredYoVariable("notch", &registry, dt, naturalFrequencyInHz, dampingRatio, SecondOrderFilterType::NOTCH);

   std::vector<double> b(3), a(3);
   filteredYoVariable.getFilterCoefficients(b, a);
   expectArrayNear(b, bAssert, 1e-8);
   expectArrayNear(a, aAssert, 1e-8);
}

TEST(SecondOrderFilteredYoDoubleTest, testHighPassFilterCoefficients)
{
   registry::YoRegistry registry("SecondOrderFilteredYoDoubleTest");
   double dt = 0.001;
   double dampingRatio = 1.0;
   double naturalFrequencyInHz = 10.0;
   std::vector<double> bAssert{4000000.0, -8000000.0, 4000000.0};
   std::vector<double> aAssert{4255275.254047619, -7992104.316479129, 3752620.429473252};

   SecondOrderFilteredYoDouble filteredYoVariable("highPass", &registry, dt, naturalFrequencyInHz, dampingRatio, SecondOrderFilterType::HIGH_PASS);

   std::vector<double> b(3), a(3);
   filteredYoVariable.getFilterCoefficients(b, a);
   expectArrayNear(b, bAssert, 1e-8);
   expectArrayNear(a, aAssert, 1e-8);
}
} // namespace
} // namespace ihmc::yovariables::filters
