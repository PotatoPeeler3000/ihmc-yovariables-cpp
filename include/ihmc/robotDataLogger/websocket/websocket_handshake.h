#pragma once

#include <string>

#include "ihmc/robotDataLogger/http/http_server.h"

namespace ihmc::robotDataLogger::websocket
{
/**
 * Computes the RFC6455 `Sec-WebSocket-Accept` value for a given `Sec-WebSocket-Key`:
 * base64(SHA1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11")).
 */
std::string computeAcceptKey(const std::string& secWebSocketKey);

/**
 * Writes the RFC6455 "101 Switching Protocols" response for an upgrade request, or a "400 Bad
 * Request" if the request is missing a Sec-WebSocket-Key. Returns true if the handshake succeeded
 * (a 101 response was written) - the caller should only start treating the socket as a WebSocket
 * connection in that case.
 */
bool writeServerHandshakeResponse(asio::ip::tcp::socket& socket, const http::HttpRequest& request);
}
