#include <gtest/gtest.h>

#include <asio.hpp>
#include <chrono>
#include <cstring>
#include <istream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <thread>

#include "ihmc/robotDataLogger/custom_log_data_publisher.h"
#include "ihmc/robotDataLogger/websocket/base64.h"
#include "ihmc/robotDataLogger/websocket/websocket_frame.h"
#include "ihmc/robotDataLogger/websocket/websocket_handshake.h"
#include "ihmc/robotDataLogger/yo_variable_server.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

using namespace ihmc::yovariables;

namespace ihmc::robotDataLogger
{
namespace
{
// Minimal blocking HTTP GET, mirroring the one in http_server_test.cpp.
std::string httpGetBody(asio::ip::tcp::socket& socket, const std::string& path)
{
   std::ostringstream request;
   request << "GET " << path << " HTTP/1.1\r\nHost: localhost\r\n\r\n";
   asio::write(socket, asio::buffer(request.str()));

   asio::streambuf buffer;
   asio::read_until(socket, buffer, "\r\n\r\n");
   std::istream stream(&buffer);
   std::string statusLine;
   std::getline(stream, statusLine);

   std::string headerLine;
   int contentLength = 0;
   while (std::getline(stream, headerLine) && headerLine != "\r")
   {
      if (headerLine.rfind("Content-Length:", 0) == 0)
         contentLength = std::stoi(headerLine.substr(headerLine.find(':') + 1));
   }

   std::string body((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
   while (static_cast<int>(body.size()) < contentLength)
   {
      char chunk[256];
      std::size_t n = socket.read_some(asio::buffer(chunk));
      body.append(chunk, n);
   }
   return body;
}

TEST(YoVariableServerTest, testFullProtocolSelfRoundTrip)
{
   registry::YoRegistry root("Main");
   variable::YoDouble position("position", &root);
   position.set(42.5);

   YoVariableServer server("TestServer", DataServerSettings(/*logSession=*/false, /*autoDiscoverable=*/false, /*port=*/0), 0.001);
   server.setMainRegistry(&root);
   server.start();

   asio::io_context ioContext;

   // --- HTTP: announcement.json / handshake.json, exactly as the real client fetches them. ---
   asio::ip::tcp::socket httpSocket(ioContext);
   httpSocket.connect(asio::ip::tcp::endpoint(asio::ip::make_address("127.0.0.1"), server.port()));

   nlohmann::json announcement = nlohmann::json::parse(httpGetBody(httpSocket, "/announcement.json"));
   ASSERT_TRUE(announcement.contains("us::ihmc::robotDataLogger::Announcement"));
   EXPECT_EQ("TestServer", announcement["us::ihmc::robotDataLogger::Announcement"]["name"].get<std::string>());

   nlohmann::json handshake = nlohmann::json::parse(httpGetBody(httpSocket, "/handshake.json"));
   ASSERT_TRUE(handshake.contains("us::ihmc::robotDataLogger::Handshake"));
   const nlohmann::json& variables = handshake["us::ihmc::robotDataLogger::Handshake"]["variables"];
   ASSERT_EQ(1u, variables.size());
   EXPECT_EQ("position", variables[0]["name"].get<std::string>());

   // --- WebSocket upgrade on a fresh connection, matching the real client's flow. ---
   asio::ip::tcp::socket wsSocket(ioContext);
   wsSocket.connect(asio::ip::tcp::endpoint(asio::ip::make_address("127.0.0.1"), server.port()));

   std::uint8_t rawKeyBytes[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
   std::string secWebSocketKey = websocket::base64Encode(rawKeyBytes, 16);

   std::ostringstream upgradeRequest;
   upgradeRequest << "GET /websocket HTTP/1.1\r\n";
   upgradeRequest << "Host: localhost\r\n";
   upgradeRequest << "Upgrade: websocket\r\n";
   upgradeRequest << "Connection: Upgrade\r\n";
   upgradeRequest << "Sec-WebSocket-Key: " << secWebSocketKey << "\r\n";
   upgradeRequest << "Sec-WebSocket-Version: 13\r\n";
   upgradeRequest << "\r\n";
   asio::write(wsSocket, asio::buffer(upgradeRequest.str()));

   asio::streambuf upgradeResponseBuffer;
   asio::read_until(wsSocket, upgradeResponseBuffer, "\r\n\r\n");
   std::istream upgradeResponseStream(&upgradeResponseBuffer);
   std::string statusLine;
   std::getline(upgradeResponseStream, statusLine);
   EXPECT_NE(std::string::npos, statusLine.find("101"));

   std::string expectedAccept = websocket::computeAcceptKey(secWebSocketKey);
   std::string headerLine;
   bool foundAccept = false;
   while (std::getline(upgradeResponseStream, headerLine) && headerLine != "\r")
   {
      if (headerLine.rfind("Sec-WebSocket-Accept:", 0) == 0)
      {
         foundAccept = true;
         EXPECT_NE(std::string::npos, headerLine.find(expectedAccept));
      }
   }
   EXPECT_TRUE(foundAccept);

   // --- Drive an update() and receive the resulting data frame. The server registers the new
   // connection asynchronously right after the 101 response is written, so retry update() briefly
   // until data actually arrives instead of assuming a single call lands after registration. ---
   std::optional<websocket::WebSocketFrame> frame;
   for (int attempt = 0; attempt < 50 && !frame.has_value(); attempt++)
   {
      server.update(1000 + attempt);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (wsSocket.available() > 0)
      {
         asio::error_code readError;
         frame = websocket::readFrame(wsSocket, readError);
         ASSERT_FALSE(readError);
      }
   }

   ASSERT_TRUE(frame.has_value()) << "no data frame received after retrying update()";
   EXPECT_EQ(websocket::WebSocketOpcode::Binary, frame->opcode);

   DecodedLogDataFrame decoded = decodeLogDataFrame(frame->payload);
   EXPECT_EQ(LogDataType::DataPacket, decoded.type);
   EXPECT_EQ(1, decoded.registryID);
   ASSERT_EQ(1u, decoded.valuesAsLongBits.size());

   double decodedValue;
   std::int64_t bits = decoded.valuesAsLongBits[0];
   std::memcpy(&decodedValue, &bits, sizeof(decodedValue));
   EXPECT_DOUBLE_EQ(42.5, decodedValue);

   server.close();
}
} // namespace
}
