#pragma once

#include "open1v/camera_protocol.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace open1v {

struct CameraIdentity {
    std::uint8_t modelType{};
    std::uint8_t cameraId{};
    std::uint8_t status{};
};

struct CameraClock {
    unsigned year{};
    unsigned month{};
    unsigned day{};
    unsigned hour{};
    unsigned minute{};
    unsigned second{};
};

const std::vector<std::uint8_t>& cameraBlock(
    const std::vector<CameraPacket>& packets, std::uint8_t command);
CameraIdentity decodeIdentity(const std::vector<CameraPacket>& packets);
CameraClock decodeClock(const std::vector<CameraPacket>& packets);
std::array<unsigned, 19> decodeCfnOptions(
    const std::vector<std::uint8_t>& block);
void setCfnOption(std::vector<std::uint8_t>& block, unsigned number,
                  unsigned option);
std::array<bool, 30> decodePfnEnabled(
    const std::vector<std::uint8_t>& ddBlock);
void setPfnEnabled(std::vector<std::uint8_t>& ddBlock, unsigned number,
                   bool enabled);

} // namespace open1v
