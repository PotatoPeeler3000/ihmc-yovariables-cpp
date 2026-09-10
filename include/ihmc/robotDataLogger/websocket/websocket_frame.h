#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <asio.hpp>

namespace ihmc::robotDataLogger::websocket
{
/** RFC6455 section 5.2 opcodes - only the ones this protocol actually uses. */
enum class WebSocketOpcode : std::uint8_t
{
   Continuation = 0x0,
   Text = 0x1,
   Binary = 0x2,
   Close = 0x8,
   Ping = 0x9,
   Pong = 0xA
};

struct WebSocketFrame
{
   WebSocketOpcode opcode;
   std::vector<std::uint8_t> payload;
};

/**
 * Reads exactly one RFC6455 frame (blocking). This server neither sends nor expects fragmented
 * messages - every message type in this protocol (data frames, commands, change requests, control
 * frames) is small and always sent as a single unfragmented frame on the Java side, so continuation
 * frames are not implemented. Returns nullopt on any read error/EOF. Masked client frames (the
 * client is required by RFC6455 to mask everything it sends) are unmasked automatically.
 */
std::optional<WebSocketFrame> readFrame(asio::ip::tcp::socket& socket, asio::error_code& error);

/**
 * Writes one unfragmented frame. Per RFC6455, a server must never mask frames it sends - this
 * never does.
 */
void writeFrame(asio::ip::tcp::socket& socket, WebSocketOpcode opcode, const std::vector<std::uint8_t>& payload, asio::error_code& error);
}
