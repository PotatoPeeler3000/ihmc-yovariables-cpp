#pragma once

#include <cstdint>
#include <vector>

namespace ihmc::robotDataLogger::wire
{
/**
 * Thin wrapper over the vendored LZ4 *block* API (third_party/lz4), matching the wire format
 * produced by Java's `net.jpountz.lz4.LZ4Compressor.compress(ByteBuffer,ByteBuffer)` -
 * `LZ4_compress_default`/`LZ4_decompress_safe` under the hood, i.e. raw compressed blocks with no
 * frame header/footer and no stored uncompressed length. The decompressed size must be known out of
 * band (the YoVariableServer protocol carries it via `numberOfVariables * 8`), exactly as the Java
 * side does.
 */
std::size_t lz4CompressBound(std::size_t uncompressedSize);

/** Compresses src into a buffer sized by lz4CompressBound(), resized down to the actual output. */
std::vector<std::uint8_t> lz4CompressBlock(const std::vector<std::uint8_t>& src);

/** Decompresses src, which must decompress to exactly decompressedSize bytes. */
std::vector<std::uint8_t> lz4DecompressBlock(const std::uint8_t* src, std::size_t srcSize, std::size_t decompressedSize);
}
