#include "ihmc/robotDataLogger/websocket/base64.h"

namespace ihmc::robotDataLogger::websocket
{
namespace
{
constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}

std::string base64Encode(const std::uint8_t* data, std::size_t size)
{
   std::string result;
   result.reserve(((size + 2) / 3) * 4);

   std::size_t i = 0;
   while (i + 3 <= size)
   {
      std::uint32_t chunk = (static_cast<std::uint32_t>(data[i]) << 16) | (static_cast<std::uint32_t>(data[i + 1]) << 8) | data[i + 2];
      result.push_back(kAlphabet[(chunk >> 18) & 0x3F]);
      result.push_back(kAlphabet[(chunk >> 12) & 0x3F]);
      result.push_back(kAlphabet[(chunk >> 6) & 0x3F]);
      result.push_back(kAlphabet[chunk & 0x3F]);
      i += 3;
   }

   std::size_t remaining = size - i;
   if (remaining == 1)
   {
      std::uint32_t chunk = static_cast<std::uint32_t>(data[i]) << 16;
      result.push_back(kAlphabet[(chunk >> 18) & 0x3F]);
      result.push_back(kAlphabet[(chunk >> 12) & 0x3F]);
      result.push_back('=');
      result.push_back('=');
   }
   else if (remaining == 2)
   {
      std::uint32_t chunk = (static_cast<std::uint32_t>(data[i]) << 16) | (static_cast<std::uint32_t>(data[i + 1]) << 8);
      result.push_back(kAlphabet[(chunk >> 18) & 0x3F]);
      result.push_back(kAlphabet[(chunk >> 12) & 0x3F]);
      result.push_back(kAlphabet[(chunk >> 6) & 0x3F]);
      result.push_back('=');
   }

   return result;
}

std::string base64Encode(const std::vector<std::uint8_t>& data)
{
   return base64Encode(data.data(), data.size());
}
}
