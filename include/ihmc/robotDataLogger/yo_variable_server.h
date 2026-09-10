#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <asio.hpp>

#include "ihmc/robotDataLogger/data_server_settings.h"
#include "ihmc/robotDataLogger/handshake/handshake_types.h"
#include "ihmc/robotDataLogger/handshake/yo_variable_hand_shake_builder.h"
#include "ihmc/robotDataLogger/registry_send_buffer.h"

namespace ihmc::yovariables::registry
{
class YoRegistry;
}

namespace ihmc::robotDataLogger::http
{
class HttpServer;
struct HttpRequest;
}

namespace ihmc::robotDataLogger::websocket
{
class WebSocketConnection;
}

namespace ihmc::robotDataLogger
{
/**
 * A C++ implementation of Java's `us.ihmc.robotDataLogger.YoVariableServer`, wire-compatible with
 * the unmodified Java `YoVariableClient`: serves the HTTP announcement/handshake endpoints, accepts
 * WebSocket connections at /websocket, and broadcasts a CustomLogDataPublisherType-equivalent
 * binary frame to every connected client on each update().
 * <p>
 * MVP scope (see the plan): a single main registry, no joints/YoGraphics/parameters-load-status
 * detail beyond what the handshake builder already covers, no inbound VariableChangeRequest
 * handling, no UDP timestamp channel, no autodiscovery broadcast, no disk logging, and a stubbed
 * (non-functional) reconnectKey - none of these affect a first connection from the Java client,
 * only reconnection and advanced tooling (SCS2 change-requests, logger daemons).
 * </p>
 */
class YoVariableServer
{
public:
   YoVariableServer(const std::string& name, const DataServerSettings& settings, double dt);
   ~YoVariableServer();

   YoVariableServer(const YoVariableServer&) = delete;
   YoVariableServer& operator=(const YoVariableServer&) = delete;

   /** Must be called exactly once, before start(). */
   void setMainRegistry(yovariables::registry::YoRegistry* registry);

   /** Builds the handshake from the main registry and starts serving HTTP/WebSocket connections. */
   void start();

   /** Snapshots every variable's current value and broadcasts one frame to all connected clients. */
   void update(std::int64_t timestamp);

   /** Stops serving and closes every connection. Safe to call while clients are connected. */
   void close();

   unsigned short port() const;

private:
   void onWebSocketUpgrade(asio::ip::tcp::socket socket, const http::HttpRequest& request);
   void pruneClosedConnectionsLocked();

   std::string name_;
   DataServerSettings settings_;
   double dt_;

   yovariables::registry::YoRegistry* mainRegistry_ = nullptr;
   bool started_ = false;

   std::unique_ptr<handshake::YoVariableHandShakeBuilder> handshakeBuilder_;
   std::unique_ptr<RegistrySendBuffer> sendBuffer_;
   handshake::Announcement announcement_;

   std::unique_ptr<http::HttpServer> httpServer_;

   std::mutex connectionsMutex_;
   std::vector<std::shared_ptr<websocket::WebSocketConnection>> connections_;

   std::int64_t nextUid_ = 0;
};
}
