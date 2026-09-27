#pragma once

#include "open1v/transport.hpp"

#include <functional>
#include <memory>
#include <string>

namespace open1v {

// On POSIX systems, the first process owns the physical transport and serves
// later processes over a per-device Unix-domain socket. Windows currently
// falls back to the supplied physical transport.
class SharedTransport final : public ITransport {
public:
    using Factory = std::function<std::unique_ptr<ITransport>()>;

    SharedTransport(std::string deviceKey, Factory factory);
    ~SharedTransport() override;
    SharedTransport(const SharedTransport&) = delete;
    SharedTransport& operator=(const SharedTransport&) = delete;

    std::vector<std::uint8_t> transact(
        std::span<const std::uint8_t> request,
        std::chrono::milliseconds timeout) override;
    void beginOperation() override;
    void endOperation() noexcept override;
    bool isShared() const noexcept override;
    bool acquireCameraSession() override;
    bool releaseCameraSession() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace open1v
