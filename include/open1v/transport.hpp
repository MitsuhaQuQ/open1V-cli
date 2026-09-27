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

    // Shared transports use these hooks to serialize a complete logical
    // command, rather than merely individual bridge frames, across processes.
    virtual void beginOperation() {}
    virtual void endOperation() noexcept {}
    virtual bool isShared() const noexcept { return false; }
    // Returns true when an already-active camera session was inherited.
    virtual bool acquireCameraSession() { return false; }
    // Returns true when the caller is the final user and must send F2.
    virtual bool releaseCameraSession() { return true; }
};

} // namespace open1v
