#include "open1v/camera_data.hpp"

#include <stdexcept>

namespace open1v {
namespace {
unsigned bcd(std::uint8_t value) {
    if ((value & 0x0f) > 9 || (value >> 4) > 9)
        throw std::runtime_error("camera clock contains invalid BCD");
    return (value >> 4) * 10 + (value & 0x0f);
}
}

const std::vector<std::uint8_t>& cameraBlock(
    const std::vector<CameraPacket>& packets, std::uint8_t command) {
    for (const auto& packet : packets)
        if (!packet.bytes.empty() && packet.bytes[0] == command)
            return packet.bytes;
    throw std::runtime_error("expected camera block missing");
}

CameraIdentity decodeIdentity(const std::vector<CameraPacket>& packets) {
    const auto& block = cameraBlock(packets, 0xf1);
    if (block.size() != 6) throw std::runtime_error("invalid camera identity block");
    return {block[2], static_cast<std::uint8_t>(block[3] & 0x7f), block[4]};
}

CameraClock decodeClock(const std::vector<CameraPacket>& packets) {
    const auto& block = cameraBlock(packets, 0xf3);
    if (block.size() != 9) throw std::runtime_error("invalid camera clock block");
    CameraClock result{2000 + bcd(block[2]), bcd(block[3]), bcd(block[4]),
                       bcd(block[5]), bcd(block[6]), bcd(block[7])};
    if (result.month < 1 || result.month > 12 || result.day < 1 ||
        result.day > 31 || result.hour > 23 || result.minute > 59 ||
        result.second > 59)
        throw std::runtime_error("camera clock is outside its valid range");
    return result;
}

std::array<unsigned, 19> decodeCfnOptions(
    const std::vector<std::uint8_t>& block) {
    if (block.size() < 13) throw std::runtime_error("invalid C.Fn block");
    std::array<unsigned, 19> result{};
    for (unsigned number = 1; number <= 19; ++number) {
        const auto value = block.at(2 + (number - 1) / 2);
        const unsigned bits = number == 19 ? value :
            (number & 1 ? value & 0x0f : value >> 4);
        unsigned option = 0;
        while (option < 8 && bits != (1u << option)) ++option;
        result[number - 1] = option < 8 ? option : 0xff;
    }
    return result;
}

void setCfnOption(std::vector<std::uint8_t>& block, unsigned number,
                  unsigned option) {
    if (number < 1 || number > 19 || option > (number == 19 ? 7u : 3u))
        throw std::runtime_error("C.Fn selection is outside its valid range");
    const auto at = 2 + (number - 1) / 2;
    if (at >= block.size()) throw std::runtime_error("invalid C.Fn block");
    const auto encoded = static_cast<std::uint8_t>(1u << option);
    if (number == 19) block[at] = encoded;
    else if (number & 1) block[at] = static_cast<std::uint8_t>((block[at] & 0xf0) | encoded);
    else block[at] = static_cast<std::uint8_t>((block[at] & 0x0f) | (encoded << 4));
}

std::array<bool, 30> decodePfnEnabled(
    const std::vector<std::uint8_t>& block) {
    if (block.size() < 7) throw std::runtime_error("invalid P.Fn state block");
    std::array<bool, 30> result{};
    for (unsigned number = 1; number <= 30; ++number) {
        const int group = (static_cast<int>(number) - 1) / 8;
        const int bit = (static_cast<int>(number) - 1) % 8;
        result[number - 1] = (block.at(2 + 3 - group) & (1u << bit)) != 0;
    }
    return result;
}

void setPfnEnabled(std::vector<std::uint8_t>& block, unsigned number,
                   bool enabled) {
    if (number < 1 || number > 30 || block.size() < 7)
        throw std::runtime_error("invalid P.Fn selection or state block");
    const int group = (static_cast<int>(number) - 1) / 8;
    const int bit = (static_cast<int>(number) - 1) % 8;
    auto& value = block.at(2 + 3 - group);
    const auto mask = static_cast<std::uint8_t>(1u << bit);
    if (enabled) value |= mask;
    else value &= static_cast<std::uint8_t>(~mask);
}

} // namespace open1v
