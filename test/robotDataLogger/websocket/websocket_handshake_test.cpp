#include <gtest/gtest.h>

#include "ihmc/robotDataLogger/websocket/websocket_handshake.h"

namespace ihmc::robotDataLogger::websocket
{
namespace
{
// The official worked example from RFC6455 section 1.3.
TEST(WebSocketHandshakeTest, testComputeAcceptKeyMatchesRfc6455Example)
{
   EXPECT_EQ("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", computeAcceptKey("dGhlIHNhbXBsZSBub25jZQ=="));
}
} // namespace
}
