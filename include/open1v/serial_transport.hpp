#pragma once

#include "open1v/transport.hpp"

#include <memory>
#include <string>

namespace open1v {

class SerialTransport final : public ITransport {
public:
    explicit SerialTransport(std::string port = {});
    ~SerialTransport() override;
    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;
    std::vector<std::uint8_t> transact(std::span<const std::uint8_t> request,
                                      std::chrono::milliseconds timeout) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace open1v
