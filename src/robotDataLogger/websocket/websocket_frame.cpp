#include "ihmc/robotDataLogger/websocket/websocket_frame.h"

#include <array>

namespace ihmc::robotDataLogger::websocket
{
std::optional<WebSocketFrame> readFrame(asio::ip::tcp::socket& socket, asio::error_code& error)
{
   std::uint8_t header[2];
   asio::read(socket, asio::buffer(header, 2), error);
   if (error)
      return std::nullopt;

   WebSocketOpcode opcode = static_cast<WebSocketOpcode>(header[0] & 0x0F);
   bool masked = (header[1] & 0x80) != 0;
   std::uint64_t length = header[1] & 0x7F;

   if (length == 126)
   {
      std::uint8_t extended[2];
      asio::read(socket, asio::buffer(extended, 2), error);
      if (error)
         return std::nullopt;
      length = (static_cast<std::uint64_t>(extended[0]) << 8) | extended[1];
   }
   else if (length == 127)
   {
      std::uint8_t extended[8];
      asio::read(socket, asio::buffer(extended, 8), error);
      if (error)
         return std::nullopt;
      length = 0;
      for (int i = 0; i < 8; i++)
         length = (length << 8) | extended[i];
   }

   std::uint8_t maskKey[4] = {0, 0, 0, 0};
   if (masked)
   {
      asio::read(socket, asio::buffer(maskKey, 4), error);
      if (error)
         return std::nullopt;
   }

   WebSocketFrame frame;
   frame.opcode = opcode;
   frame.payload.resize(static_cast<std::size_t>(length));
   if (length > 0)
   {
      asio::read(socket, asio::buffer(frame.payload), error);
      if (error)
         return std::nullopt;
   }

   if (masked)
   {
      for (std::size_t i = 0; i < frame.payload.size(); i++)
         frame.payload[i] ^= maskKey[i % 4];
   }

   return frame;
}

void writeFrame(asio::ip::tcp::socket& socket, WebSocketOpcode opcode, const std::vector<std::uint8_t>& payload, asio::error_code& error)
{
   std::vector<std::uint8_t> header;
   header.push_back(static_cast<std::uint8_t>(0x80 | static_cast<std::uint8_t>(opcode))); // FIN=1, RSV=0

   std::size_t length = payload.size();
   if (length < 126)
   {
      header.push_back(static_cast<std::uint8_t>(length)); // MASK=0 (server never masks)
   }
   else if (length <= 0xFFFF)
   {
      header.push_back(126);
      header.push_back(static_cast<std::uint8_t>((length >> 8) & 0xFF));
      header.push_back(static_cast<std::uint8_t>(length & 0xFF));
   }
   else
   {
      header.push_back(127);
      for (int shift = 56; shift >= 0; shift -= 8)
         header.push_back(static_cast<std::uint8_t>((static_cast<std::uint64_t>(length) >> shift) & 0xFF));
   }

   std::array<asio::const_buffer, 2> buffers = {asio::buffer(header), asio::buffer(payload)};
   asio::write(socket, buffers, error);
}
}
