#include "ihmc/robotDataLogger/yo_variable_server.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <sstream>
#include <stdexcept>

#include <unistd.h>

#include "ihmc/robotDataLogger/custom_log_data_publisher.h"
#include "ihmc/robotDataLogger/handshake/json_serialization.h"
#include "ihmc/robotDataLogger/http/http_server.h"
#include "ihmc/robotDataLogger/websocket/websocket_connection.h"
#include "ihmc/robotDataLogger/websocket/websocket_handshake.h"
#include "ihmc/yovariables/registry/yo_registry.h"

namespace ihmc::robotDataLogger
{
namespace
{
std::string localHostName()
{
   char buffer[256];
   if (gethostname(buffer, sizeof(buffer)) != 0)
      return "localhost";
   buffer[sizeof(buffer) - 1] = '\0';
   return std::string(buffer);
}

// Not a spec-compliant UUID (Java uses UUID.randomUUID()) - just needs to look plausible and be
// distinct per server start, since the client only stores/round-trips this string.
std::string randomIdentifier()
{
   std::random_device randomDevice;
   std::mt19937_64 random(randomDevice());
   std::uniform_int_distribution<int> hexDigit(0, 15);
   static const char* kHex = "0123456789abcdef";

   std::ostringstream out;
   const int groupLengths[] = {8, 4, 4, 4, 12};
   for (std::size_t group = 0; group < 5; group++)
   {
      if (group > 0)
         out << '-';
      for (int i = 0; i < groupLengths[group]; i++)
         out << kHex[hexDigit(random)];
   }
   return out.str();
}

std::int64_t nowNanoseconds()
{
   return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

YoVariableServer::YoVariableServer(const std::string& name, const DataServerSettings& settings, double dt) : name_(name), settings_(settings), dt_(dt)
{
}

YoVariableServer::~YoVariableServer()
{
   close();
}

void YoVariableServer::setMainRegistry(yovariables::registry::YoRegistry* registry)
{
   mainRegistry_ = registry;
}

void YoVariableServer::start()
{
   if (started_)
      throw std::logic_error("YoVariableServer::start() called more than once");
   if (mainRegistry_ == nullptr)
      throw std::logic_error("YoVariableServer::setMainRegistry() must be called before start()");

   handshakeBuilder_ = std::make_unique<handshake::YoVariableHandShakeBuilder>(name_, dt_);
   handshakeBuilder_->build(*mainRegistry_);
   sendBuffer_ = std::make_unique<RegistrySendBuffer>(handshakeBuilder_->getVariablesInWireOrder());

   announcement_.identifier = randomIdentifier();
   announcement_.name = name_;
   announcement_.hostName = localHostName();
   // Real Java servers hash the CDR-encoded handshake (HandshakeHashCalculator) for this, which
   // the client only checks on reconnect() - not on first connect - so a stub is fine for now
   // (deferred scope: reconnect support).
   announcement_.reconnectKey = "unsupported-reconnect-key";
   announcement_.log = settings_.logSession;
   announcement_.modelFileDescription.hasModel = false;

   httpServer_ = std::make_unique<http::HttpServer>(settings_.port);
   httpServer_->setJsonEndpoint("/announcement.json", [this]() { return handshake::toJsonString(announcement_); });
   httpServer_->setJsonEndpoint("/handshake.json", [this]() { return handshake::toJsonString(handshakeBuilder_->getHandshake()); });
   httpServer_->setUpgradeHandler("/websocket", [this](asio::ip::tcp::socket socket, const http::HttpRequest& request)
                                   { onWebSocketUpgrade(std::move(socket), request); });

   httpServer_->start();
   started_ = true;
}

void YoVariableServer::onWebSocketUpgrade(asio::ip::tcp::socket socket, const http::HttpRequest& request)
{
   if (!websocket::writeServerHandshakeResponse(socket, request))
      return; // socket closes when it goes out of scope

   auto connection = std::make_shared<websocket::WebSocketConnection>(std::move(socket));
   connection->start();

   std::lock_guard<std::mutex> lock(connectionsMutex_);
   connections_.push_back(connection);
}

void YoVariableServer::pruneClosedConnectionsLocked()
{
   connections_.erase(std::remove_if(connections_.begin(), connections_.end(),
                                      [](const std::shared_ptr<websocket::WebSocketConnection>& connection) { return !connection->isOpen(); }),
                       connections_.end());
}

void YoVariableServer::update(std::int64_t timestamp)
{
   if (!started_)
      throw std::logic_error("YoVariableServer::update() called before start()");

   sendBuffer_->updateFromVariables();
   // Registry ID 1 is always the main (and, in this MVP, only) registry's ID - see
   // YoVariableHandShakeBuilder's doc comment on registry numbering.
   std::vector<std::uint8_t> frame = encodeLogDataFrame(nextUid_++, timestamp, nowNanoseconds(), 1, *sendBuffer_);

   std::lock_guard<std::mutex> lock(connectionsMutex_);
   for (auto& connection : connections_)
   {
      if (connection->isOpen())
         connection->sendBinaryFrame(frame);
   }
   pruneClosedConnectionsLocked();
}

void YoVariableServer::close()
{
   if (httpServer_)
      httpServer_->stop();

   std::lock_guard<std::mutex> lock(connectionsMutex_);
   for (auto& connection : connections_)
      connection->close();
   connections_.clear();
}

unsigned short YoVariableServer::port() const
{
   return httpServer_ ? httpServer_->port() : settings_.port;
}
}
