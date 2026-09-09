#include <gtest/gtest.h>

#include <random>

#include "ihmc/robotDataLogger/wire/lz4_block.h"

namespace ihmc::robotDataLogger::wire
{
namespace
{
TEST(LZ4BlockTest, testRoundTripRandomBytes)
{
   std::mt19937 random(42U);
   std::uniform_int_distribution<int> byteDist(0, 255);

   std::vector<std::uint8_t> original(1000);
   for (std::uint8_t& b : original)
      b = static_cast<std::uint8_t>(byteDist(random));

   std::vector<std::uint8_t> compressed = lz4CompressBlock(original);
   std::vector<std::uint8_t> decompressed = lz4DecompressBlock(compressed.data(), compressed.size(), original.size());

   EXPECT_EQ(original, decompressed);
}

TEST(LZ4BlockTest, testRoundTripHighlyCompressibleBytes)
{
   std::vector<std::uint8_t> original(5000, 0x42);
   std::vector<std::uint8_t> compressed = lz4CompressBlock(original);
   EXPECT_LT(compressed.size(), original.size());

   std::vector<std::uint8_t> decompressed = lz4DecompressBlock(compressed.data(), compressed.size(), original.size());
   EXPECT_EQ(original, decompressed);
}

TEST(LZ4BlockTest, testRoundTripEmpty)
{
   std::vector<std::uint8_t> original;
   std::vector<std::uint8_t> compressed = lz4CompressBlock(original);
   std::vector<std::uint8_t> decompressed = lz4DecompressBlock(compressed.data(), compressed.size(), 0);
   EXPECT_EQ(original, decompressed);
}
} // namespace
}
