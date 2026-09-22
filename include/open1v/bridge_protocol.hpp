#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace open1v {

constexpr std::size_t kBridgeMaxPayload = 512;

enum class MessageType : std::uint8_t {
    ping = 0x01,
    status = 0x02,
    linkStatus = 0x03,
    exchange = 0x10,
    release = 0x11,
};

struct Frame {
    std::uint8_t type{};
    std::uint16_t sequence{};
    std::vector<std::uint8_t> payload;
};

std::uint16_t crc16CcittFalse(std::span<const std::uint8_t> bytes);
std::vector<std::uint8_t> encodeFrame(const Frame& frame);
Frame decodeFrame(std::span<const std::uint8_t> bytes);

} // namespace open1v
