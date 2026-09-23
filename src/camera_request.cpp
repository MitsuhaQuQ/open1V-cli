#include "open1v/camera_request.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace open1v {
namespace {
void pause(std::uint16_t milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
}

CameraRequest::CameraRequest(BridgeClient& bridge) : bridge_(bridge) {}

bool CameraRequest::packetValid(const std::vector<std::uint8_t>& packet,
                                std::uint8_t command) {
    if (packet.size() < 3 || packet[0] != command ||
        packet.size() != static_cast<std::size_t>(packet[1]) + 3) return false;
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i + 1 < packet.size(); ++i)
        sum = static_cast<std::uint8_t>(sum + packet[i]);
    return sum == packet.back();
}

std::vector<std::uint8_t> CameraRequest::fixed(
    std::uint8_t command, std::uint16_t expected, ExchangeProfile profile) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto result = bridge_.exchangeProfiled(
            std::span<const std::uint8_t>(&command, 1), expected, profile);
        if (result.status == 0 && packetValid(result.bytes, command) &&
            result.bytes.size() == expected) return result.bytes;
        if (result.bytes == std::vector<std::uint8_t>{0xf4}) serviceAsyncF4();
        pause(attempt == 0 ? 100 : 150);
    }
    throw std::runtime_error("fixed-length camera request failed");
}

std::vector<std::uint8_t> CameraRequest::variable(
    std::uint8_t command, std::uint16_t capacity, ExchangeProfile profile) {
    for (int attempt = 0; attempt < 8; ++attempt) {
        const auto result = bridge_.exchangeProfiled(
            std::span<const std::uint8_t>(&command, 1), capacity, profile);
        if (packetValid(result.bytes, command)) return result.bytes;
        if (result.bytes == std::vector<std::uint8_t>{0xf4}) serviceAsyncF4();
        pause(attempt < 2 ? 100 : 200);
    }
    std::ostringstream message;
    message << "variable-length camera request 0x" << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0') << static_cast<unsigned>(command)
            << " failed";
    throw std::runtime_error(message.str());
}

void CameraRequest::serviceAsyncF4() {
    const std::uint8_t acknowledge = 0xf4;
    (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
    pause(2);
    const std::uint8_t status = 0xf6;
    for (int attempt = 0; attempt < 3; ++attempt) {
        try {
            const auto result = bridge_.exchangeProfiled(
                std::span<const std::uint8_t>(&status, 1), 17,
                ExchangeProfile::normal);
            if (result.status == 0 && packetValid(result.bytes, status)) return;
            if (result.bytes == std::vector<std::uint8_t>{0xf4}) {
                (void)bridge_.exchange(
                    std::span<const std::uint8_t>(&acknowledge, 1), 0);
                pause(2);
            }
        } catch (...) {}
        pause(50);
    }
}

} // namespace open1v
