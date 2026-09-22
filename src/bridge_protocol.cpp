#include "open1v/bridge_protocol.hpp"

#include <stdexcept>

namespace open1v {
namespace {
constexpr std::uint8_t kMagic0 = 'O';
constexpr std::uint8_t kMagic1 = '1';
constexpr std::uint8_t kVersion = 1;
constexpr std::size_t kHeaderSize = 8;

void appendLe16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8));
}

std::uint16_t readLe16(std::span<const std::uint8_t> data, std::size_t at) {
    return static_cast<std::uint16_t>(data[at]) |
           static_cast<std::uint16_t>(data[at + 1] << 8);
}
} // namespace

std::uint16_t crc16CcittFalse(std::span<const std::uint8_t> bytes) {
    std::uint16_t crc = 0xffff;
    for (const auto byte : bytes) {
        crc ^= static_cast<std::uint16_t>(byte) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = static_cast<std::uint16_t>((crc & 0x8000)
                      ? (crc << 1) ^ 0x1021
                      : crc << 1);
        }
    }
    return crc;
}

std::vector<std::uint8_t> encodeFrame(const Frame& frame) {
    if (frame.payload.size() > kBridgeMaxPayload) {
        throw std::runtime_error("bridge payload is too large");
    }
    std::vector<std::uint8_t> result{kMagic0, kMagic1, kVersion, frame.type};
    appendLe16(result, frame.sequence);
    appendLe16(result, static_cast<std::uint16_t>(frame.payload.size()));
    result.insert(result.end(), frame.payload.begin(), frame.payload.end());
    appendLe16(result, crc16CcittFalse(result));
    return result;
}

Frame decodeFrame(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < kHeaderSize + 2 || bytes[0] != kMagic0 ||
        bytes[1] != kMagic1 || bytes[2] != kVersion) {
        throw std::runtime_error("invalid bridge frame header");
    }
    const auto length = readLe16(bytes, 6);
    if (length > kBridgeMaxPayload || bytes.size() != kHeaderSize + length + 2) {
        throw std::runtime_error("invalid bridge frame length");
    }
    const auto expected = readLe16(bytes, bytes.size() - 2);
    if (crc16CcittFalse(bytes.first(bytes.size() - 2)) != expected) {
        throw std::runtime_error("bridge frame CRC mismatch");
    }
    return Frame{bytes[3], readLe16(bytes, 4),
                 std::vector<std::uint8_t>(bytes.begin() + kHeaderSize,
                                           bytes.end() - 2)};
}

} // namespace open1v

