#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ihmc::robotDataLogger::wire
{
/**
 * A minimal, hand-rolled re-implementation of the subset of IHMC's Java
 * `us.ihmc.fastddsjava.cdr.CDRBuffer` needed to talk the YoVariableServer wire protocol: writes and
 * reads the fixed set of primitive types actually used by `CustomLogDataPublisherType` (the
 * variable-data streaming frame) and `VariableChangeRequest`. This is not a general OMG-CDR/IDL
 * implementation - only the primitives, alignment rule, and payload-header handling that those two
 * message types exercise are implemented.
 * <p>
 * Byte order: a freshly constructed buffer defaults to big-endian, mirroring Java NIO's
 * `ByteBuffer` default order - this matters because `VariableChangeRequest` is deliberately never
 * wrapped with `writePayloadHeader()`/`readPayloadHeader()` on the Java side, so it stays
 * big-endian for its entire lifetime. Calling `writePayloadHeader()`/`readPayloadHeader()` switches
 * subsequent reads/writes to little-endian, exactly as the OMG CDR encapsulation header `{0,1,0,0}`
 * ("CDR_LE") declares.
 * </p>
 * <p>
 * Alignment: per the Java source, every aligned field is padded relative to a *virtual* payload
 * start 4 bytes into the buffer (`position - kPayloadHeaderSize`), even on buffers where
 * `writePayloadHeader()` was never actually called (as for `VariableChangeRequest`). This does
 * produce real padding in practice - e.g. `CustomLogDataPublisherType` writes a 1-byte `type` field
 * immediately before a 4-byte-aligned `registry` int32, which inserts 3 padding bytes (confirmed
 * directly from `CustomLogDataPublisherType.calculateSizeBytes()`'s explicit
 * `CDRBuffer.alignment(currentAlignment, 4)` call between those two fields) - see
 * `cdr_buffer_test.cpp` for the verified byte layouts.
 * </p>
 */
class CDRBuffer
{
public:
   static constexpr std::size_t kPayloadHeaderSize = 4;

   /** Fresh write buffer, big-endian until writePayloadHeader() is called. */
   CDRBuffer() = default;

   /** Wraps existing bytes for reading, big-endian until readPayloadHeader() is called. */
   explicit CDRBuffer(std::vector<std::uint8_t> data);

   // --- Writing -------------------------------------------------------------------------------

   /** Writes the 4-byte OMG-CDR encapsulation header {0,1,0,0} and switches to little-endian. */
   void writePayloadHeader();

   void writeByte(std::uint8_t value);
   void writeBoolean(bool value);
   void writeUInt16(std::uint16_t value);
   void writeInt32(std::int32_t value);
   void writeInt64(std::int64_t value);
   void writeFloat64(double value);

   /** Narrow (1-byte-per-char) CDR string: 4-byte-aligned length (char count + 1), chars, NUL. */
   void writeString(const std::string& value);

   /** Appends raw bytes with no alignment - used for the pre-compressed/LZ4 payload blob. */
   void writeRawBytes(const std::vector<std::uint8_t>& bytes);

   // --- Reading ---------------------------------------------------------------------------------

   /** Reads the 2-byte RepresentationIdentifier (big-endian) and selects byte order from it. */
   void readPayloadHeader();

   std::uint8_t readByte();
   bool readBoolean();
   std::uint16_t readUInt16();
   std::int32_t readInt32();
   std::int64_t readInt64();
   double readFloat64();
   std::string readString();
   std::vector<std::uint8_t> readRawBytes(std::size_t count);

   std::size_t position() const
   {
      return position_;
   }

   std::size_t size() const
   {
      return data_.size();
   }

   const std::vector<std::uint8_t>& data() const
   {
      return data_;
   }

private:
   void align(std::size_t byteBoundary);
   void ensureCapacityForWrite(std::size_t additionalBytes);

   std::vector<std::uint8_t> data_;
   std::size_t position_ = 0;
   bool littleEndian_ = false;
};

/**
 * Appends `value` as 8 raw big-endian bytes with no CDR alignment/header semantics - used by
 * RegistrySendBuffer to pack variable values exactly as Java's default-order heap ByteBuffer does,
 * independent of (and inconsistent with) the little-endian CDR framing wrapped around it.
 */
void appendBigEndianInt64(std::vector<std::uint8_t>& buffer, std::int64_t value);

/** Mirror of appendBigEndianInt64 for reading back a value packed that way. */
std::int64_t readBigEndianInt64(const std::uint8_t* bytes);
}
