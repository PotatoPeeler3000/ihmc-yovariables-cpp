#include <gtest/gtest.h>

#include "ihmc/robotDataLogger/websocket/base64.h"

namespace ihmc::robotDataLogger::websocket
{
namespace
{
std::string encodeString(const std::string& s)
{
   return base64Encode(reinterpret_cast<const std::uint8_t*>(s.data()), s.size());
}

// RFC 4648 section 10 test vectors.
TEST(Base64Test, testRfc4648Vectors)
{
   EXPECT_EQ("", encodeString(""));
   EXPECT_EQ("Zg==", encodeString("f"));
   EXPECT_EQ("Zm8=", encodeString("fo"));
   EXPECT_EQ("Zm9v", encodeString("foo"));
   EXPECT_EQ("Zm9vYg==", encodeString("foob"));
   EXPECT_EQ("Zm9vYmE=", encodeString("fooba"));
   EXPECT_EQ("Zm9vYmFy", encodeString("foobar"));
}
} // namespace
}
