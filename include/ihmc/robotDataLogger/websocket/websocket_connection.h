#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <asio.hpp>

namespace ihmc::robotDataLogger::websocket
{
/**
 * One connected WebSocket client, after the RFC6455 handshake has already completed (see
 * websocket_handshake.h). Owns a reader thread that responds to Ping with Pong and otherwise
 * drops incoming frames - the text command protocol (SEND_TIMESTAMPS etc.) and inbound
 * VariableChangeRequest binary frames are deferred scope (see the plan), so this class only needs
 * to keep the connection alive and detect when the client disconnects. Outgoing data frames are
 * written by whichever thread calls sendBinaryFrame() (the YoVariableServer's update() caller),
 * serialized against the reader thread's occasional Pong writes via writeMutex_.
 */
class WebSocketConnection : public std::enable_shared_from_this<WebSocketConnection>
{
public:
   explicit WebSocketConnection(asio::ip::tcp::socket socket);
   ~WebSocketConnection();

   WebSocketConnection(const WebSocketConnection&) = delete;
   WebSocketConnection& operator=(const WebSocketConnection&) = delete;

   /** Starts the reader thread. Must be called at most once. */
   void start();

   bool isOpen() const;

   /** Thread-safe. Returns false (and marks the connection closed) if the write failed. */
   bool sendBinaryFrame(const std::vector<std::uint8_t>& payload);

   /** Closes the socket and, if called from a thread other than the reader thread, joins it. */
   void close();

private:
   void readLoop();

   asio::ip::tcp::socket socket_;
   std::mutex writeMutex_;
   std::thread readerThread_;
   std::atomic<bool> open_{true};
};
}
