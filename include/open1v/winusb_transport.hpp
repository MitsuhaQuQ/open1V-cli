#pragma once

#include "open1v/transport.hpp"

#include <memory>

namespace open1v {

class WinUsbTransport final : public ITransport {
public:
    WinUsbTransport();
    ~WinUsbTransport() override;
    WinUsbTransport(const WinUsbTransport&) = delete;
    WinUsbTransport& operator=(const WinUsbTransport&) = delete;
    std::vector<std::uint8_t> transact(std::span<const std::uint8_t> request,
                                      std::chrono::milliseconds timeout) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace open1v

