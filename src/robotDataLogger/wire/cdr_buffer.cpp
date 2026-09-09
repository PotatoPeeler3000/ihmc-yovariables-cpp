#include "ihmc/robotDataLogger/wire/cdr_buffer.h"

#include <cstring>
#include <stdexcept>

namespace ihmc::robotDataLogger::wire
{
namespace
{
constexpr std::uint8_t kPayloadHeader[CDRBuffer::kPayloadHeaderSize] = {0, 1, 0, 0};
constexpr std::uint16_t kCdrLittleEndian = 0x0001;
constexpr std::uint16_t kCdrBigEndian = 0x0000;
constexpr std::uint16_t kPlCdrLittleEndian = 0x0003;
}

CDRBuffer::CDRBuffer(std::vector<std::uint8_t> data) : data_(std::move(data))
{
}

void CDRBuffer::ensureCapacityForWrite(std::size_t additionalBytes)
{
   if (position_ + additionalBytes > data_.size())
      data_.resize(position_ + additionalBytes);
}

void CDRBuffer::align(std::size_t byteBoundary)
{
   // Alignment is always computed relative to a virtual payload start 4 bytes into the buffer,
   // matching Java's CDRBuffer.alignBuffer() even on buffers where writePayloadHeader() was never
   // called (see class doc comment).
   std::int64_t relativePosition = static_cast<std::int64_t>(position_) - static_cast<std::int64_t>(kPayloadHeaderSize);
   std::int64_t boundary = static_cast<std::int64_t>(byteBoundary);
   std::int64_t remainder = relativePosition % boundary;
   if (remainder == 0)
      return;

   std::int64_t pad = boundary - remainder;
   if (pad < 0)
      pad += boundary;

   ensureCapacityForWrite(static_cast<std::size_t>(pad));
   for (std::int64_t i = 0; i < pad; i++)
      data_[position_++] = 0;
}

void CDRBuffer::writePayloadHeader()
{
   ensureCapacityForWrite(kPayloadHeaderSize);
   for (std::uint8_t byte : kPayloadHeader)
      data_[position_++] = byte;
   littleEndian_ = true;
}

void CDRBuffer::readPayloadHeader()
{
   std::uint8_t high = data_.at(position_);
   std::uint8_t low = data_.at(position_ + 1);
   std::uint16_t representationIdentifier = static_cast<std::uint16_t>((high << 8) | low);
   position_ += 4; // RepresentationIdentifier (2 bytes) + RepresentationOptions (2 bytes, discarded)
   littleEndian_ = representationIdentifier == kCdrLittleEndian || representationIdentifier == kPlCdrLittleEndian;
}

void CDRBuffer::writeByte(std::uint8_t value)
{
   ensureCapacityForWrite(1);
   data_[position_++] = value;
}

void CDRBuffer::writeBoolean(bool value)
{
   writeByte(value ? 1 : 0);
}

void CDRBuffer::writeUInt16(std::uint16_t value)
{
   align(2);
   ensureCapacityForWrite(2);
   if (littleEndian_)
   {
      data_[position_++] = static_cast<std::uint8_t>(value & 0xFF);
      data_[position_++] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
   }
   else
   {
      data_[position_++] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
      data_[position_++] = static_cast<std::uint8_t>(value & 0xFF);
   }
}

void CDRBuffer::writeInt32(std::int32_t value)
{
   align(4);
   ensureCapacityForWrite(4);
   std::uint32_t bits = static_cast<std::uint32_t>(value);
   for (int i = 0; i < 4; i++)
   {
      int shift = littleEndian_ ? (8 * i) : (8 * (3 - i));
      data_[position_++] = static_cast<std::uint8_t>((bits >> shift) & 0xFF);
   }
}

void CDRBuffer::writeInt64(std::int64_t value)
{
   align(8);
   ensureCapacityForWrite(8);
   std::uint64_t bits = static_cast<std::uint64_t>(value);
   for (int i = 0; i < 8; i++)
   {
      int shift = littleEndian_ ? (8 * i) : (8 * (7 - i));
      data_[position_++] = static_cast<std::uint8_t>((bits >> shift) & 0xFF);
   }
}

void CDRBuffer::writeFloat64(double value)
{
   std::uint64_t bits;
   std::memcpy(&bits, &value, sizeof(bits));
   align(8);
   ensureCapacityForWrite(8);
   for (int i = 0; i < 8; i++)
   {
      int shift = littleEndian_ ? (8 * i) : (8 * (7 - i));
      data_[position_++] = static_cast<std::uint8_t>((bits >> shift) & 0xFF);
   }
}

void CDRBuffer::writeString(const std::string& value)
{
   align(4);
   writeInt32(static_cast<std::int32_t>(value.size() + 1));
   ensureCapacityForWrite(value.size() + 1);
   for (char c : value)
      data_[position_++] = static_cast<std::uint8_t>(c);
   data_[position_++] = 0;
}

void CDRBuffer::writeRawBytes(const std::vector<std::uint8_t>& bytes)
{
   ensureCapacityForWrite(bytes.size());
   for (std::uint8_t byte : bytes)
      data_[position_++] = byte;
}

std::uint8_t CDRBuffer::readByte()
{
   return data_.at(position_++);
}

bool CDRBuffer::readBoolean()
{
   return readByte() != 0;
}

std::uint16_t CDRBuffer::readUInt16()
{
   align(2);
   if (position_ + 2 > data_.size())
      throw std::out_of_range("CDRBuffer::readUInt16: buffer underrun");
   std::uint8_t b0 = data_[position_];
   std::uint8_t b1 = data_[position_ + 1];
   position_ += 2;
   return littleEndian_ ? static_cast<std::uint16_t>(b0 | (b1 << 8)) : static_cast<std::uint16_t>((b0 << 8) | b1);
}

std::int32_t CDRBuffer::readInt32()
{
   align(4);
   if (position_ + 4 > data_.size())
      throw std::out_of_range("CDRBuffer::readInt32: buffer underrun");
   std::uint32_t bits = 0;
   for (int i = 0; i < 4; i++)
   {
      int shift = littleEndian_ ? (8 * i) : (8 * (3 - i));
      bits |= static_cast<std::uint32_t>(data_[position_ + i]) << shift;
   }
   position_ += 4;
   return static_cast<std::int32_t>(bits);
}

std::int64_t CDRBuffer::readInt64()
{
   align(8);
   if (position_ + 8 > data_.size())
      throw std::out_of_range("CDRBuffer::readInt64: buffer underrun");
   std::uint64_t bits = 0;
   for (int i = 0; i < 8; i++)
   {
      int shift = littleEndian_ ? (8 * i) : (8 * (7 - i));
      bits |= static_cast<std::uint64_t>(data_[position_ + i]) << shift;
   }
   position_ += 8;
   return static_cast<std::int64_t>(bits);
}

double CDRBuffer::readFloat64()
{
   align(8);
   if (position_ + 8 > data_.size())
      throw std::out_of_range("CDRBuffer::readFloat64: buffer underrun");
   std::uint64_t bits = 0;
   for (int i = 0; i < 8; i++)
   {
      int shift = littleEndian_ ? (8 * i) : (8 * (7 - i));
      bits |= static_cast<std::uint64_t>(data_[position_ + i]) << shift;
   }
   position_ += 8;
   double value;
   std::memcpy(&value, &bits, sizeof(value));
   return value;
}

std::string CDRBuffer::readString()
{
   align(4);
   std::int32_t lengthWithTerminator = readInt32();
   if (lengthWithTerminator < 1)
      throw std::out_of_range("CDRBuffer::readString: invalid length");
   std::size_t charCount = static_cast<std::size_t>(lengthWithTerminator - 1);
   if (position_ + charCount + 1 > data_.size())
      throw std::out_of_range("CDRBuffer::readString: buffer underrun");
   std::string value(reinterpret_cast<const char*>(&data_[position_]), charCount);
   position_ += charCount + 1; // +1 for the NUL terminator
   return value;
}

std::vector<std::uint8_t> CDRBuffer::readRawBytes(std::size_t count)
{
   if (position_ + count > data_.size())
      throw std::out_of_range("CDRBuffer::readRawBytes: buffer underrun");
   std::vector<std::uint8_t> result(data_.begin() + position_, data_.begin() + position_ + count);
   position_ += count;
   return result;
}

void appendBigEndianInt64(std::vector<std::uint8_t>& buffer, std::int64_t value)
{
   std::uint64_t bits = static_cast<std::uint64_t>(value);
   for (int i = 0; i < 8; i++)
      buffer.push_back(static_cast<std::uint8_t>((bits >> (8 * (7 - i))) & 0xFF));
}

std::int64_t readBigEndianInt64(const std::uint8_t* bytes)
{
   std::uint64_t bits = 0;
   for (int i = 0; i < 8; i++)
      bits |= static_cast<std::uint64_t>(bytes[i]) << (8 * (7 - i));
   return static_cast<std::int64_t>(bits);
}
}
