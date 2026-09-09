#include <gtest/gtest.h>

#include "ihmc/robotDataLogger/wire/cdr_buffer.h"

namespace ihmc::robotDataLogger::wire
{
namespace
{
TEST(CDRBufferTest, testPayloadHeaderBytes)
{
   CDRBuffer buffer;
   buffer.writePayloadHeader();
   ASSERT_EQ(4u, buffer.size());
   EXPECT_EQ(0x00, buffer.data()[0]);
   EXPECT_EQ(0x01, buffer.data()[1]);
   EXPECT_EQ(0x00, buffer.data()[2]);
   EXPECT_EQ(0x00, buffer.data()[3]);
}

// Verifies the exact LogData-equivalent header layout from CustomLogDataPublisherType.serialize():
// header, then int64 uid/timestamp/transmitTime, uint8 type, int32 registry/numberOfVariables - all
// little-endian. Confirmed directly from CustomLogDataPublisherType.java's serialize() and
// calculateSizeBytes(): the 1-byte `type` field is immediately followed by a 4-byte-aligned int32
// (`registry`), which inserts 3 padding bytes - calculateSizeBytes() has an explicit
// `CDRBuffer.alignment(currentAlignment, 4)` call for exactly this gap.
TEST(CDRBufferTest, testLogDataHeaderLayoutLittleEndianWithAlignmentPadding)
{
   CDRBuffer buffer;
   buffer.writePayloadHeader();
   buffer.writeInt64(0x0102030405060708);
   buffer.writeInt64(11);
   buffer.writeInt64(22);
   buffer.writeByte(1); // LogDataType::DATA_PACKET
   buffer.writeInt32(7);
   buffer.writeInt32(3);

   // header(4) + 3*int64(24) + byte(1) + 3 padding bytes (to realign to 4) + 2*int32(8) = 40.
   ASSERT_EQ(4u + 8u + 8u + 8u + 1u + 3u + 4u + 4u, buffer.size());

   CDRBuffer reader(buffer.data());
   reader.readPayloadHeader();
   EXPECT_EQ(0x0102030405060708, reader.readInt64());
   EXPECT_EQ(11, reader.readInt64());
   EXPECT_EQ(22, reader.readInt64());
   EXPECT_EQ(1, reader.readByte());
   EXPECT_EQ(7, reader.readInt32());
   EXPECT_EQ(3, reader.readInt32());

   // Spot-check actual byte order (little-endian) and the 3-byte pad: registry=7 starts 3 bytes
   // after the type byte, as bytes {07,00,00,00}.
   std::size_t typeOffset = 4 + 8 + 8 + 8;
   std::size_t registryOffset = typeOffset + 1 + 3;
   EXPECT_EQ(0x00, buffer.data()[typeOffset + 1]) << "padding byte";
   EXPECT_EQ(0x00, buffer.data()[typeOffset + 2]) << "padding byte";
   EXPECT_EQ(0x00, buffer.data()[typeOffset + 3]) << "padding byte";
   EXPECT_EQ(0x07, buffer.data()[registryOffset]);
   EXPECT_EQ(0x00, buffer.data()[registryOffset + 1]);
}

// Verifies VariableChangeRequest's wire format: no CDR header at all, big-endian, and - per the
// alignment-relative-to-(position-4) quirk - zero padding between the int32 and the float64.
TEST(CDRBufferTest, testVariableChangeRequestLayoutBigEndianHeaderless)
{
   CDRBuffer buffer; // no writePayloadHeader() call - stays big-endian, like the Java side.
   buffer.writeInt32(42);
   buffer.writeFloat64(3.5);

   ASSERT_EQ(12u, buffer.size()) << "expected zero padding between int32 and float64";

   // variableID=42 big-endian => {00,00,00,2A}
   EXPECT_EQ(0x00, buffer.data()[0]);
   EXPECT_EQ(0x00, buffer.data()[1]);
   EXPECT_EQ(0x00, buffer.data()[2]);
   EXPECT_EQ(0x2A, buffer.data()[3]);

   CDRBuffer reader(buffer.data());
   EXPECT_EQ(42, reader.readInt32());
   EXPECT_DOUBLE_EQ(3.5, reader.readFloat64());
}

TEST(CDRBufferTest, testStringRoundTrip)
{
   CDRBuffer buffer;
   buffer.writePayloadHeader();
   buffer.writeString("hello");
   buffer.writeString("");
   buffer.writeString("world!!");

   CDRBuffer reader(buffer.data());
   reader.readPayloadHeader();
   EXPECT_EQ("hello", reader.readString());
   EXPECT_EQ("", reader.readString());
   EXPECT_EQ("world!!", reader.readString());
}

TEST(CDRBufferTest, testUInt16RoundTrip)
{
   CDRBuffer buffer;
   buffer.writePayloadHeader();
   buffer.writeUInt16(0);
   buffer.writeUInt16(1);
   buffer.writeUInt16(65535);

   CDRBuffer reader(buffer.data());
   reader.readPayloadHeader();
   EXPECT_EQ(0u, reader.readUInt16());
   EXPECT_EQ(1u, reader.readUInt16());
   EXPECT_EQ(65535u, reader.readUInt16());
}

TEST(CDRBufferTest, testBooleanRoundTrip)
{
   CDRBuffer buffer;
   buffer.writePayloadHeader();
   buffer.writeBoolean(true);
   buffer.writeBoolean(false);

   CDRBuffer reader(buffer.data());
   reader.readPayloadHeader();
   EXPECT_TRUE(reader.readBoolean());
   EXPECT_FALSE(reader.readBoolean());
}

TEST(CDRBufferTest, testReadPayloadHeaderDetectsLittleEndian)
{
   CDRBuffer buffer;
   buffer.writePayloadHeader();
   buffer.writeInt32(-123);

   CDRBuffer reader(buffer.data());
   reader.readPayloadHeader();
   EXPECT_EQ(-123, reader.readInt32());
}

TEST(CDRBufferTest, testBigEndianInt64PackingHelper)
{
   std::vector<std::uint8_t> buffer;
   appendBigEndianInt64(buffer, 0x0102030405060708);
   ASSERT_EQ(8u, buffer.size());
   EXPECT_EQ(0x01, buffer[0]);
   EXPECT_EQ(0x08, buffer[7]);
   EXPECT_EQ(0x0102030405060708, readBigEndianInt64(buffer.data()));
}
} // namespace
}
