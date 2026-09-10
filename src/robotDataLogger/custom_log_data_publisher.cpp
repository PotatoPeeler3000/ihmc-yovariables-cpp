#include "ihmc/robotDataLogger/custom_log_data_publisher.h"

#include "ihmc/robotDataLogger/wire/cdr_buffer.h"
#include "ihmc/robotDataLogger/wire/lz4_block.h"

namespace ihmc::robotDataLogger
{
std::vector<std::uint8_t> encodeLogDataFrame(std::int64_t uid, std::int64_t timestamp, std::int64_t transmitTime, std::int32_t registryID,
                                              const RegistrySendBuffer& buffer)
{
   wire::CDRBuffer cdr;
   cdr.writePayloadHeader();
   cdr.writeInt64(uid);
   cdr.writeInt64(timestamp);
   cdr.writeInt64(transmitTime);
   cdr.writeByte(static_cast<std::uint8_t>(LogDataType::DataPacket));
   cdr.writeInt32(registryID);
   cdr.writeInt32(static_cast<std::int32_t>(buffer.numberOfVariables()));

   std::vector<std::uint8_t> compressed = wire::lz4CompressBlock(buffer.rawValueBytes());
   cdr.writeInt32(static_cast<std::int32_t>(compressed.size()));
   cdr.writeRawBytes(compressed);

   // Joint states: always empty in this phase - joints are deferred scope.
   cdr.writeInt32(0);

   return cdr.data();
}

DecodedLogDataFrame decodeLogDataFrame(const std::vector<std::uint8_t>& frame)
{
   wire::CDRBuffer cdr(frame);
   cdr.readPayloadHeader();

   DecodedLogDataFrame result;
   result.uid = cdr.readInt64();
   result.timestamp = cdr.readInt64();
   result.transmitTime = cdr.readInt64();
   result.type = static_cast<LogDataType>(cdr.readByte());
   result.registryID = cdr.readInt32();
   result.numberOfVariables = cdr.readInt32();

   std::int32_t compressedLength = cdr.readInt32();
   std::vector<std::uint8_t> compressed = cdr.readRawBytes(static_cast<std::size_t>(compressedLength));
   std::size_t decompressedSize = static_cast<std::size_t>(result.numberOfVariables) * 8;
   std::vector<std::uint8_t> decompressed = wire::lz4DecompressBlock(compressed.data(), compressed.size(), decompressedSize);

   result.valuesAsLongBits.reserve(static_cast<std::size_t>(result.numberOfVariables));
   for (std::int32_t i = 0; i < result.numberOfVariables; i++)
      result.valuesAsLongBits.push_back(wire::readBigEndianInt64(&decompressed[static_cast<std::size_t>(i) * 8]));

   // Joint state count/array: always 0/empty in this phase - not read further.

   return result;
}
}
