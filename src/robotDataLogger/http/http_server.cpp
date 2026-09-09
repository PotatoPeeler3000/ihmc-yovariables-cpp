#include "ihmc/robotDataLogger/http/http_server.h"

#include <algorithm>
#include <cctype>
#include <istream>
#include <optional>
#include <sstream>

namespace ihmc::robotDataLogger::http
{
namespace
{
std::string toLower(std::string value)
{
   std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
   return value;
}

std::string trim(const std::string& value)
{
   std::size_t begin = value.find_first_not_of(" \t\r\n");
   if (begin == std::string::npos)
      return "";
   std::size_t end = value.find_last_not_of(" \t\r\n");
   return value.substr(begin, end - begin + 1);
}

/** Reads and parses one HTTP request line + headers; nullopt if the connection closed/errored. */
std::optional<HttpRequest> readRequest(asio::ip::tcp::socket& socket, asio::streambuf& buffer)
{
   asio::error_code error;
   std::size_t bytesRead = asio::read_until(socket, buffer, "\r\n\r\n", error);
   if (error || bytesRead == 0)
      return std::nullopt;

   std::istream stream(&buffer);
   std::string requestLine;
   std::getline(stream, requestLine);

   HttpRequest request;
   std::istringstream requestLineStream(requestLine);
   std::string httpVersion;
   requestLineStream >> request.method >> request.path >> httpVersion;
   if (request.method.empty() || request.path.empty())
      return std::nullopt;

   std::string headerLine;
   while (std::getline(stream, headerLine))
   {
      std::string trimmed = trim(headerLine);
      if (trimmed.empty())
         break;
      std::size_t colon = trimmed.find(':');
      if (colon == std::string::npos)
         continue;
      request.headers[toLower(trim(trimmed.substr(0, colon)))] = trim(trimmed.substr(colon + 1));
   }

   return request;
}
}

std::string HttpRequest::header(const std::string& lowerCaseName) const
{
   auto it = headers.find(lowerCaseName);
   return it == headers.end() ? "" : it->second;
}

HttpServer::HttpServer(unsigned short port) : acceptor_(ioContext_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port))
{
}

HttpServer::~HttpServer()
{
   stop();
}

void HttpServer::setJsonEndpoint(const std::string& path, JsonBodyProvider bodyProvider)
{
   jsonEndpoints_[path] = std::move(bodyProvider);
}

void HttpServer::setUpgradeHandler(const std::string& path, UpgradeHandler handler)
{
   upgradeHandlers_[path] = std::move(handler);
}

unsigned short HttpServer::port() const
{
   return acceptor_.local_endpoint().port();
}

void HttpServer::start()
{
   running_ = true;
   doAccept();
   ioThread_ = std::thread([this]() { ioContext_.run(); });
}

void HttpServer::stop()
{
   if (!running_.exchange(false))
      return;
   ioContext_.stop();
   if (ioThread_.joinable())
      ioThread_.join();

   // Unblock every still-tracked connection's thread (each is parked in a blocking read waiting on
   // an idle keep-alive client that will never send more data) by shutting its socket down - this
   // deliberately does NOT also close() the socket here: the connection thread's read is
   // implemented under the hood as poll() then recv() (confirmed by sampling a hung process), and
   // closing the fd out from under a different thread that has that fd in an in-flight poll() is a
   // genuine race on this platform - sometimes the poll() notices, sometimes it hangs forever.
   // shutdown(SHUT_RDWR) alone reliably wakes the poll() (the kernel marks the fd readable/EOF),
   // and each shared_ptr<socket> then actually closes only once every reference to it is gone -
   // the connection thread's own copy (dropped when handleConnection() returns) and this class's
   // copy (dropped by connectionSockets_.clear() below, after every connection thread is joined,
   // so nothing is still polling it by then).
   {
      std::lock_guard<std::mutex> lock(connectionsMutex_);
      for (auto& socket : connectionSockets_)
      {
         asio::error_code ignored;
         socket->shutdown(asio::ip::tcp::socket::shutdown_both, ignored);
      }
   }
   for (std::thread& thread : connectionThreads_)
   {
      if (thread.joinable())
         thread.join();
   }
   connectionThreads_.clear();
   std::lock_guard<std::mutex> lock(connectionsMutex_);
   connectionSockets_.clear();
}

void HttpServer::doAccept()
{
   acceptor_.async_accept(
       [this](asio::error_code error, asio::ip::tcp::socket socket)
       {
          if (!error)
          {
             auto socketPtr = std::make_shared<asio::ip::tcp::socket>(std::move(socket));
             std::lock_guard<std::mutex> lock(connectionsMutex_);
             connectionSockets_.push_back(socketPtr);
             connectionThreads_.emplace_back(&HttpServer::handleConnection, this, socketPtr);
          }
          if (running_)
             doAccept();
       });
}

void HttpServer::untrackConnection(const std::shared_ptr<asio::ip::tcp::socket>& socket)
{
   std::lock_guard<std::mutex> lock(connectionsMutex_);
   connectionSockets_.erase(std::remove(connectionSockets_.begin(), connectionSockets_.end(), socket), connectionSockets_.end());
}

void HttpServer::handleConnection(std::shared_ptr<asio::ip::tcp::socket> socketPtr)
{
   asio::ip::tcp::socket& socket = *socketPtr;
   asio::streambuf buffer;
   while (true)
   {
      std::optional<HttpRequest> request = readRequest(socket, buffer);
      if (!request.has_value())
      {
         untrackConnection(socketPtr);
         return; // client closed the connection or sent garbage
      }

      if (request->method != "GET")
      {
         writeResponse(socket, 400, "Bad Request", "text/plain", "");
         untrackConnection(socketPtr);
         return;
      }

      bool isUpgradeRequest = toLower(request->header("upgrade")) == "websocket";
      auto upgradeIt = upgradeHandlers_.find(request->path);
      if (isUpgradeRequest && upgradeIt != upgradeHandlers_.end())
      {
         // Ownership transfers to the upgrade handler from here - stop() no longer force-closes
         // or joins on behalf of this connection (Server Phase 4's WebSocket layer owns that).
         untrackConnection(socketPtr);
         upgradeIt->second(std::move(socket), *request);
         return;
      }

      auto jsonIt = jsonEndpoints_.find(request->path);
      if (jsonIt != jsonEndpoints_.end())
      {
         writeResponse(socket, 200, "OK", "application/json; charset=UTF-8", jsonIt->second());
         continue; // keep the connection alive for the next sequential request
      }

      writeNotFound(socket);
      // The client may issue further sequential requests on this connection even after a 404
      // (e.g. it always probes /resources.zip regardless of whether one exists).
   }
}

void HttpServer::writeResponse(asio::ip::tcp::socket& socket, int statusCode, const std::string& statusText, const std::string& contentType,
                                const std::string& body)
{
   std::ostringstream response;
   response << "HTTP/1.1 " << statusCode << " " << statusText << "\r\n";
   response << "Content-Type: " << contentType << "\r\n";
   response << "Content-Length: " << body.size() << "\r\n";
   response << "Connection: keep-alive\r\n";
   response << "\r\n";
   response << body;

   asio::error_code error;
   asio::write(socket, asio::buffer(response.str()), error);
}

void HttpServer::writeNotFound(asio::ip::tcp::socket& socket)
{
   writeResponse(socket, 404, "Not Found", "text/plain", "");
}
}
