#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// asio.hpp pulls in <windows.h> transitively on Windows; harmless here, just noting it's a heavy
// header - only included from this header and http_server.cpp, never from public headers upstream
// of it.
#include <asio.hpp>

namespace ihmc::robotDataLogger::http
{
/** A parsed HTTP/1.1 request line + headers (no body - this server only ever handles GET). */
struct HttpRequest
{
   std::string method;
   std::string path;
   /** Header names lower-cased for case-insensitive lookup, matching HTTP semantics. */
   std::unordered_map<std::string, std::string> headers;

   std::string header(const std::string& lowerCaseName) const;
};

/**
 * A minimal HTTP/1.1 server covering exactly the surface the Java YoVariableClient needs: fixed
 * GET JSON endpoints (announcement.json, handshake.json) and a WebSocket upgrade path
 * (/websocket). Not a general-purpose HTTP library - deliberately narrow, matching this protocol's
 * actual requirements (see the plan's dependency rationale).
 * <p>
 * Threading model: a single asio::io_context runs only the async accept loop (on its own thread) -
 * this makes stop() safe/portable (io_context::stop() is well-defined to call concurrently, unlike
 * closing a socket out from under a blocking accept() on another thread). Each accepted connection
 * is then handed to its own detached thread that does plain blocking reads/writes for that
 * connection's lifetime. This is a deliberate simplification versus Netty's async multi-event-loop
 * model on the Java side - adequate for a live-viewing server with a handful of concurrent clients,
 * and far simpler to get right than a fully async per-connection callback chain. Every response
 * sets Content-Length (required by the client - see HTTPDataServerConnection.java) and connections
 * are kept alive across sequential requests (the client fetches /announcement.json then
 * /handshake.json on the same connection) until the client disconnects or upgrades to a WebSocket.
 * </p>
 */
class HttpServer
{
public:
   using JsonBodyProvider = std::function<std::string()>;
   using UpgradeHandler = std::function<void(asio::ip::tcp::socket socket, const HttpRequest& request)>;

   explicit HttpServer(unsigned short port);
   ~HttpServer();

   HttpServer(const HttpServer&) = delete;
   HttpServer& operator=(const HttpServer&) = delete;

   /** Registers a GET endpoint that always responds 200 with content-type application/json. */
   void setJsonEndpoint(const std::string& path, JsonBodyProvider bodyProvider);

   /**
    * Registers the WebSocket upgrade path. Once a GET request to this path with an
    * "Upgrade: websocket" header is read, the still-open socket and parsed request are handed to
    * handler (called on the connection's own thread) instead of an HTTP response being written -
    * the handler owns the socket from that point on (completing the RFC6455 handshake and framing
    * is Server Phase 4's responsibility, not this class's).
    */
   void setUpgradeHandler(const std::string& path, UpgradeHandler handler);

   /** Starts accepting connections on a background thread. */
   void start();

   /**
    * Stops accepting new connections, force-closes every still-open HTTP connection (to unblock
    * any thread parked in a blocking read waiting on an idle keep-alive client), and joins every
    * per-connection thread before returning - this makes stop()/the destructor safe to call while
    * clients are connected. Connections already handed off to an upgrade handler are no longer
    * tracked here (see setUpgradeHandler) and are unaffected.
    */
   void stop();

   unsigned short port() const;

private:
   void doAccept();
   void handleConnection(std::shared_ptr<asio::ip::tcp::socket> socket);
   void writeResponse(asio::ip::tcp::socket& socket, int statusCode, const std::string& statusText, const std::string& contentType,
                       const std::string& body);
   void writeNotFound(asio::ip::tcp::socket& socket);
   void untrackConnection(const std::shared_ptr<asio::ip::tcp::socket>& socket);

   asio::io_context ioContext_;
   asio::ip::tcp::acceptor acceptor_;
   std::thread ioThread_;
   std::atomic<bool> running_{false};

   std::unordered_map<std::string, JsonBodyProvider> jsonEndpoints_;
   std::unordered_map<std::string, UpgradeHandler> upgradeHandlers_;

   // Tracks every connection still being served directly by this class (i.e. not yet handed off
   // to an upgrade handler), so stop() can force-close and join them instead of leaking threads
   // that outlive this object - see the real bug this fixed, documented in http_server.cpp.
   std::mutex connectionsMutex_;
   std::vector<std::shared_ptr<asio::ip::tcp::socket>> connectionSockets_;
   std::vector<std::thread> connectionThreads_;
};
}
