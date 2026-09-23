#pragma once

#include "open1v/bridge_protocol.hpp"
#include "open1v/transport.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace open1v {

struct BridgeStatus {
    std::vector<std::uint8_t> raw;
};

struct LinkStatus {
    std::uint32_t usbRxBytes{};
    std::uint32_t uartTxBytes{};
    std::uint32_t uartRxBytes{};
    std::uint32_t queueDroppedBytes{};
    std::uint32_t forwardedFrames{};
    std::uint32_t usbTxAcceptedBytes{};
    std::uint32_t usbTxZeroWrites{};
};

struct ExchangeResult {
    std::uint8_t status{};
    std::vector<std::uint8_t> bytes;
};

class BridgeClient {
public:
    explicit BridgeClient(ITransport& transport);
    std::string ping();
    BridgeStatus status();
    LinkStatus linkStatus();
    std::vector<std::uint8_t> exchange(std::span<const std::uint8_t> tx,
                                      std::uint16_t expectedBytes,
                                      std::uint16_t firstByteTimeoutMs = 500,
                                      std::uint16_t interByteTimeoutMs = 50);
    ExchangeResult exchangeResult(std::span<const std::uint8_t> tx,
                                  std::uint16_t expectedBytes,
                                  std::uint16_t firstByteTimeoutMs = 500,
                                  std::uint16_t interByteTimeoutMs = 50);
    ExchangeResult exchangeProfiled(std::span<const std::uint8_t> tx,
                                    std::uint16_t expectedBytes,
                                    ExchangeProfile profile);
    void release();

private:
    Frame request(MessageType type, std::span<const std::uint8_t> payload,
                  int timeoutMs = 1000, bool requireOk = true);
    ITransport& transport_;
    std::uint16_t nextSequence_{1};
    bool profiledExchangeUnsupported_{};
};

} // namespace open1v
