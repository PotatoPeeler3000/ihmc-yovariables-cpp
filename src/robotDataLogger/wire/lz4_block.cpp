#include "ihmc/robotDataLogger/wire/lz4_block.h"

#include <stdexcept>

#include "lz4.h"

namespace ihmc::robotDataLogger::wire
{
std::size_t lz4CompressBound(std::size_t uncompressedSize)
{
   return static_cast<std::size_t>(LZ4_compressBound(static_cast<int>(uncompressedSize)));
}

std::vector<std::uint8_t> lz4CompressBlock(const std::vector<std::uint8_t>& src)
{
   std::vector<std::uint8_t> dst(lz4CompressBound(src.size()));
   int compressedSize = LZ4_compress_default(reinterpret_cast<const char*>(src.data()),
                                              reinterpret_cast<char*>(dst.data()),
                                              static_cast<int>(src.size()),
                                              static_cast<int>(dst.size()));
   if (compressedSize <= 0)
      throw std::runtime_error("LZ4_compress_default failed");
   dst.resize(static_cast<std::size_t>(compressedSize));
   return dst;
}

std::vector<std::uint8_t> lz4DecompressBlock(const std::uint8_t* src, std::size_t srcSize, std::size_t decompressedSize)
{
   std::vector<std::uint8_t> dst(decompressedSize);
   int result = LZ4_decompress_safe(reinterpret_cast<const char*>(src), reinterpret_cast<char*>(dst.data()), static_cast<int>(srcSize),
                                     static_cast<int>(dst.size()));
   if (result < 0 || static_cast<std::size_t>(result) != decompressedSize)
      throw std::runtime_error("LZ4_decompress_safe failed");
   return dst;
}
}
