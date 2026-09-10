#include <gtest/gtest.h>

#include <asio.hpp>
#include <vector>

#include "ihmc/robotDataLogger/websocket/websocket_frame.h"

namespace ihmc::robotDataLogger::websocket
{
namespace
{
struct SocketPair
{
   asio::io_context ioContext;
   asio::ip::tcp::socket a;
   asio::ip::tcp::socket b;

   SocketPair() : a(ioContext), b(ioContext)
   {
      asio::ip::tcp::acceptor acceptor(ioContext, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), 0));
      a.connect(asio::ip::tcp::endpoint(asio::ip::make_address("127.0.0.1"), acceptor.local_endpoint().port()));
      acceptor.accept(b);
   }
};

TEST(WebSocketFrameTest, testUnmaskedBinaryFrameRoundTrip)
{
   SocketPair sockets;
   std::vector<std::uint8_t> payload = {1, 2, 3, 4, 5, 250, 251, 252};

   asio::error_code writeError;
   writeFrame(sockets.a, WebSocketOpcode::Binary, payload, writeError);
   ASSERT_FALSE(writeError);

   asio::error_code readError;
   std::optional<WebSocketFrame> frame = readFrame(sockets.b, readError);
   ASSERT_TRUE(frame.has_value());
   EXPECT_EQ(WebSocketOpcode::Binary, frame->opcode);
   EXPECT_EQ(payload, frame->payload);
}

TEST(WebSocketFrameTest, testLargePayloadUses16BitExtendedLength)
{
   SocketPair sockets;
   std::vector<std::uint8_t> payload(1000, 0xAB); // > 125, triggers the 126 extended-length form

   asio::error_code writeError;
   writeFrame(sockets.a, WebSocketOpcode::Binary, payload, writeError);
   ASSERT_FALSE(writeError);

   asio::error_code readError;
   std::optional<WebSocketFrame> frame = readFrame(sockets.b, readError);
   ASSERT_TRUE(frame.has_value());
   EXPECT_EQ(payload, frame->payload);
}

// The Java client is required by RFC6455 to mask every frame it sends - verify readFrame()
// correctly unmasks, by hand-constructing a masked frame's raw bytes (readFrame() itself is only
// ever exercised with unmasked payloads elsewhere, since writeFrame() never masks).
TEST(WebSocketFrameTest, testMaskedFrameIsUnmaskedOnRead)
{
   SocketPair sockets;

   std::vector<std::uint8_t> payload = {0x11, 0x22, 0x33, 0x44, 0x55};
   std::uint8_t maskKey[4] = {0xDE, 0xAD, 0xBE, 0xEF};

   std::vector<std::uint8_t> raw;
   raw.push_back(0x80 | static_cast<std::uint8_t>(WebSocketOpcode::Binary)); // FIN=1, opcode=binary
   raw.push_back(0x80 | static_cast<std::uint8_t>(payload.size()));         // MASK=1, len=5
   for (std::uint8_t b : maskKey)
      raw.push_back(b);
   for (std::size_t i = 0; i < payload.size(); i++)
      raw.push_back(payload[i] ^ maskKey[i % 4]);

   asio::write(sockets.a, asio::buffer(raw));

   asio::error_code readError;
   std::optional<WebSocketFrame> frame = readFrame(sockets.b, readError);
   ASSERT_TRUE(frame.has_value());
   EXPECT_EQ(WebSocketOpcode::Binary, frame->opcode);
   EXPECT_EQ(payload, frame->payload);
}
} // namespace
}
