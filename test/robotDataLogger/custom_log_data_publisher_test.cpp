#include <gtest/gtest.h>

#include <cstring>

#include "ihmc/robotDataLogger/custom_log_data_publisher.h"
#include "ihmc/robotDataLogger/registry_send_buffer.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"
#include "ihmc/yovariables/variable/yo_integer.h"
#include "ihmc/yovariables/variable/yo_long.h"

using namespace ihmc::yovariables;

namespace ihmc::robotDataLogger
{
namespace
{
enum class PublisherTestEnum
{
   A,
   B,
   C
};

TEST(CustomLogDataPublisherTest, testEncodeDecodeRoundTripPreservesAllValueTypes)
{
   registry::YoRegistry root("Root");
   variable::YoDouble d("d", &root);
   variable::YoBoolean b("b", &root);
   variable::YoInteger i("i", &root);
   variable::YoLong l("l", &root);
   variable::YoEnum<PublisherTestEnum> e("e", &root, false);

   d.set(3.14159265358979);
   b.set(true);
   i.set(-123456);
   l.set(9876543210LL);
   e.set(PublisherTestEnum::C);

   std::vector<variable::YoVariable*> wireOrder = {&d, &b, &i, &l, &e};
   RegistrySendBuffer buffer(wireOrder);
   buffer.updateFromVariables();
   ASSERT_EQ(5u, buffer.numberOfVariables());
   ASSERT_EQ(40u, buffer.rawValueBytes().size());

   std::vector<std::uint8_t> frame = encodeLogDataFrame(/*uid=*/7, /*timestamp=*/1000, /*transmitTime=*/2000, /*registryID=*/1, buffer);
   DecodedLogDataFrame decoded = decodeLogDataFrame(frame);

   EXPECT_EQ(7, decoded.uid);
   EXPECT_EQ(1000, decoded.timestamp);
   EXPECT_EQ(2000, decoded.transmitTime);
   EXPECT_EQ(LogDataType::DataPacket, decoded.type);
   EXPECT_EQ(1, decoded.registryID);
   EXPECT_EQ(5, decoded.numberOfVariables);
   ASSERT_EQ(5u, decoded.valuesAsLongBits.size());

   // Mirrors YoVariable::setValueFromLongBits() semantics on the decode side.
   double decodedDouble;
   std::int64_t bits = decoded.valuesAsLongBits[0];
   std::memcpy(&decodedDouble, &bits, sizeof(decodedDouble));
   EXPECT_DOUBLE_EQ(3.14159265358979, decodedDouble);

   EXPECT_EQ(1, decoded.valuesAsLongBits[1]); // YoBoolean true -> 1
   EXPECT_EQ(-123456, decoded.valuesAsLongBits[2]);
   EXPECT_EQ(9876543210LL, decoded.valuesAsLongBits[3]);
   EXPECT_EQ(static_cast<std::int64_t>(e.getOrdinal()), decoded.valuesAsLongBits[4]);
}

TEST(CustomLogDataPublisherTest, testEmptyRegistryEncodesAndDecodes)
{
   RegistrySendBuffer buffer(std::vector<variable::YoVariable*>{});
   buffer.updateFromVariables();

   std::vector<std::uint8_t> frame = encodeLogDataFrame(0, 0, 0, 1, buffer);
   DecodedLogDataFrame decoded = decodeLogDataFrame(frame);
   EXPECT_EQ(0, decoded.numberOfVariables);
   EXPECT_TRUE(decoded.valuesAsLongBits.empty());
}
} // namespace
}
