#include <gtest/gtest.h>

#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <istream>
#include <sstream>
#include <string>
#include <thread>

#include "ihmc/robotDataLogger/http/http_server.h"

namespace ihmc::robotDataLogger::http
{
namespace
{
struct RawResponse
{
   int statusCode = 0;
   std::string body;
   std::string raw;
};

// Minimal blocking HTTP client used only by these tests - issues one GET and reads the response
// off the same socket (so a caller can issue further sequential requests to test keep-alive).
RawResponse getRequest(asio::ip::tcp::socket& socket, const std::string& path, bool upgrade = false)
{
   std::ostringstream request;
   request << "GET " << path << " HTTP/1.1\r\n";
   request << "Host: localhost\r\n";
   if (upgrade)
   {
      request << "Upgrade: websocket\r\n";
      request << "Connection: Upgrade\r\n";
   }
   request << "\r\n";
   asio::write(socket, asio::buffer(request.str()));

   asio::streambuf buffer;
   asio::read_until(socket, buffer, "\r\n\r\n");
   std::istream stream(&buffer);
   std::string statusLine;
   std::getline(stream, statusLine);

   RawResponse response;
   std::istringstream statusLineStream(statusLine);
   std::string httpVersion;
   statusLineStream >> httpVersion >> response.statusCode;

   std::string headerLine;
   int contentLength = 0;
   while (std::getline(stream, headerLine) && headerLine != "\r")
   {
      if (headerLine.rfind("Content-Length:", 0) == 0)
         contentLength = std::stoi(headerLine.substr(headerLine.find(':') + 1));
   }

   // Anything already buffered past the headers.
   std::string alreadyRead((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
   response.body = alreadyRead;
   while (static_cast<int>(response.body.size()) < contentLength)
   {
      char chunk[256];
      std::size_t n = socket.read_some(asio::buffer(chunk));
      response.body.append(chunk, n);
   }
   return response;
}

class HttpServerTest : public ::testing::Test
{
protected:
   HttpServer server{0}; // port 0 -> OS-assigned ephemeral port
};

TEST_F(HttpServerTest, testJsonEndpointsAndNotFoundOverKeepAliveConnection)
{
   server.setJsonEndpoint("/announcement.json", []() { return std::string("{\"a\":1}"); });
   server.setJsonEndpoint("/handshake.json", []() { return std::string("{\"h\":true}"); });
   server.start();

   asio::io_context ioContext;
   asio::ip::tcp::socket socket(ioContext);
   socket.connect(asio::ip::tcp::endpoint(asio::ip::make_address("127.0.0.1"), server.port()));

   RawResponse announcement = getRequest(socket, "/announcement.json");
   EXPECT_EQ(200, announcement.statusCode);
   EXPECT_EQ("{\"a\":1}", announcement.body);

   // Same connection, sequential second request - matches the real client's flow.
   RawResponse handshake = getRequest(socket, "/handshake.json");
   EXPECT_EQ(200, handshake.statusCode);
   EXPECT_EQ("{\"h\":true}", handshake.body);

   RawResponse missing = getRequest(socket, "/model.sdf");
   EXPECT_EQ(404, missing.statusCode);

   server.stop();
}

TEST_F(HttpServerTest, testUpgradeHandlerReceivesSocketInsteadOfHttpResponse)
{
   std::atomic<bool> upgradeHandlerCalled{false};
   std::atomic<bool> upgradeHadCorrectPath{false};

   server.setUpgradeHandler("/websocket",
                             [&](asio::ip::tcp::socket, const HttpRequest& request)
                             {
                                upgradeHadCorrectPath = (request.path == "/websocket");
                                upgradeHandlerCalled = true;
                             });
   server.start();

   asio::io_context ioContext;
   asio::ip::tcp::socket socket(ioContext);
   socket.connect(asio::ip::tcp::endpoint(asio::ip::make_address("127.0.0.1"), server.port()));

   std::ostringstream request;
   request << "GET /websocket HTTP/1.1\r\n";
   request << "Host: localhost\r\n";
   request << "Upgrade: websocket\r\n";
   request << "Connection: Upgrade\r\n";
   request << "\r\n";
   asio::write(socket, asio::buffer(request.str()));

   for (int i = 0; i < 100 && !upgradeHandlerCalled; i++)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

   EXPECT_TRUE(upgradeHandlerCalled);
   EXPECT_TRUE(upgradeHadCorrectPath);

   server.stop();
}
} // namespace
}
