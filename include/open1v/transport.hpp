#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <vector>

namespace open1v {

class ITransport {
public:
    virtual ~ITransport() = default;
    virtual std::vector<std::uint8_t> transact(
        std::span<const std::uint8_t> request,
        std::chrono::milliseconds timeout) = 0;
};

} // namespace open1v

