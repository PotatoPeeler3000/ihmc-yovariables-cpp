#include "ihmc/robotDataLogger/websocket/websocket_connection.h"

#include "ihmc/robotDataLogger/websocket/websocket_frame.h"

namespace ihmc::robotDataLogger::websocket
{
WebSocketConnection::WebSocketConnection(asio::ip::tcp::socket socket) : socket_(std::move(socket))
{
}

WebSocketConnection::~WebSocketConnection()
{
   close();
}

void WebSocketConnection::start()
{
   readerThread_ = std::thread(&WebSocketConnection::readLoop, this);
}

bool WebSocketConnection::isOpen() const
{
   return open_;
}

bool WebSocketConnection::sendBinaryFrame(const std::vector<std::uint8_t>& payload)
{
   if (!open_)
      return false;

   std::lock_guard<std::mutex> lock(writeMutex_);
   asio::error_code error;
   writeFrame(socket_, WebSocketOpcode::Binary, payload, error);
   if (error)
   {
      open_ = false;
      return false;
   }
   return true;
}

void WebSocketConnection::close()
{
   open_ = false;

   // shutdown(), not close(): matches the fix in HttpServer::stop() - closing the fd out from
   // under the reader thread's in-flight blocking read is racy on this platform, whereas
   // shutdown(SHUT_RDWR) reliably wakes it. The socket is only actually destroyed/closed once
   // this WebSocketConnection itself is destroyed, by which point the reader thread has been
   // joined below (or was never started).
   asio::error_code ignored;
   socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ignored);

   if (readerThread_.joinable() && readerThread_.get_id() != std::this_thread::get_id())
      readerThread_.join();
}

void WebSocketConnection::readLoop()
{
   while (open_)
   {
      asio::error_code error;
      std::optional<WebSocketFrame> frame = readFrame(socket_, error);
      if (!frame.has_value())
      {
         open_ = false;
         return;
      }

      switch (frame->opcode)
      {
         case WebSocketOpcode::Ping:
         {
            std::lock_guard<std::mutex> lock(writeMutex_);
            asio::error_code writeError;
            writeFrame(socket_, WebSocketOpcode::Pong, frame->payload, writeError);
            if (writeError)
               open_ = false;
            break;
         }
         case WebSocketOpcode::Close:
            open_ = false;
            return;
         default:
            // Text (SEND_TIMESTAMPS/command protocol) and Binary (VariableChangeRequest) frames
            // from the client are deferred scope - see the plan's "explicitly deferred" list.
            break;
      }
   }
}
}
