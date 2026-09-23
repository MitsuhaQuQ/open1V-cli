#include "open1v/winusb_transport.hpp"

#include <stdexcept>

namespace open1v {
struct WinUsbTransport::Impl {};

WinUsbTransport::WinUsbTransport() : impl_(std::make_unique<Impl>()) {
    throw std::runtime_error("WinUSB transport is available only on Windows; use the serial transport");
}
WinUsbTransport::~WinUsbTransport() = default;
std::vector<std::uint8_t> WinUsbTransport::transact(
    std::span<const std::uint8_t>, std::chrono::milliseconds) {
    throw std::runtime_error("WinUSB transport is unavailable on this platform");
}
} // namespace open1v
