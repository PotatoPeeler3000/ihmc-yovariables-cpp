#include "ihmc/robotDataLogger/websocket/websocket_handshake.h"

#include <sstream>

#include "ihmc/robotDataLogger/websocket/base64.h"
#include "sha1.hpp"

namespace ihmc::robotDataLogger::websocket
{
namespace
{
// RFC6455 section 1.3.
constexpr const char* kWebSocketMagicGuid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// sha1.hpp's SHA1::final() returns a 40-character lowercase hex digest; RFC6455 needs the raw
// 20-byte digest for base64 encoding, so convert back.
std::vector<std::uint8_t> hexDigestToBytes(const std::string& hex)
{
   std::vector<std::uint8_t> bytes;
   bytes.reserve(hex.size() / 2);
   for (std::size_t i = 0; i + 1 < hex.size(); i += 2)
   {
      std::uint8_t byte = static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16));
      bytes.push_back(byte);
   }
   return bytes;
}
}

std::string computeAcceptKey(const std::string& secWebSocketKey)
{
   SHA1 sha1;
   sha1.update(secWebSocketKey + kWebSocketMagicGuid);
   std::string hexDigest = sha1.final();
   return base64Encode(hexDigestToBytes(hexDigest));
}

bool writeServerHandshakeResponse(asio::ip::tcp::socket& socket, const http::HttpRequest& request)
{
   std::string key = request.header("sec-websocket-key");
   if (key.empty())
   {
      static const std::string badRequest = "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n";
      asio::error_code ignored;
      asio::write(socket, asio::buffer(badRequest), ignored);
      return false;
   }

   std::string acceptKey = computeAcceptKey(key);

   std::ostringstream response;
   response << "HTTP/1.1 101 Switching Protocols\r\n";
   response << "Upgrade: websocket\r\n";
   response << "Connection: Upgrade\r\n";
   response << "Sec-WebSocket-Accept: " << acceptKey << "\r\n";
   response << "\r\n";

   asio::error_code error;
   asio::write(socket, asio::buffer(response.str()), error);
   return !error;
}
}
