#pragma once

#include <cstdint>
#include <vector>

#include "ihmc/robotDataLogger/registry_send_buffer.h"

namespace ihmc::robotDataLogger
{
/** Mirrors logger_msgs.LogDataType. */
enum class LogDataType : std::uint8_t
{
   KeepAlivePacket = 0,
   DataPacket = 1,
   VideoPacket = 2
};

/** Result of decodeLogDataFrame() - used by tests to verify encodeLogDataFrame() round-trips. */
struct DecodedLogDataFrame
{
   std::int64_t uid = 0;
   std::int64_t timestamp = 0;
   std::int64_t transmitTime = 0;
   LogDataType type = LogDataType::DataPacket;
   std::int32_t registryID = 0;
   std::int32_t numberOfVariables = 0;
   /** Per-variable getValueAsLongBits()-style values, in wire order, after LZ4 decompression. */
   std::vector<std::int64_t> valuesAsLongBits;
};

/**
 * Encodes one variable-data WebSocket binary frame, byte-for-byte matching Java's
 * `CustomLogDataPublisherType.serialize()`: CDR header, uid/timestamp/transmitTime/type/registry/
 * numberOfVariables (little-endian, with the 3-byte alignment pad between `type` and `registry`
 * confirmed directly from that class - see cdr_buffer_test.cpp), then an LZ4-block-compressed
 * variable-value blob (length-prefixed), then a joint-state double count and array (always 0/empty
 * in this phase - joints are deferred scope).
 */
std::vector<std::uint8_t> encodeLogDataFrame(std::int64_t uid, std::int64_t timestamp, std::int64_t transmitTime, std::int32_t registryID,
                                              const RegistrySendBuffer& buffer);

/** Mirror of encodeLogDataFrame(), used by tests to verify a round trip. */
DecodedLogDataFrame decodeLogDataFrame(const std::vector<std::uint8_t>& frame);
}
