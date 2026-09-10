#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ihmc::robotDataLogger::websocket
{
std::string base64Encode(const std::uint8_t* data, std::size_t size);
std::string base64Encode(const std::vector<std::uint8_t>& data);
}
