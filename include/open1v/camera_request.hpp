#pragma once

#include "open1v/bridge_client.hpp"

#include <cstdint>
#include <vector>

namespace open1v {

// Business-neutral EOS request executor. It owns packet validation, bridge
// timing-profile selection, asynchronous F4 servicing, and bounded retries.
class CameraRequest {
public:
    explicit CameraRequest(BridgeClient& bridge);
    std::vector<std::uint8_t> fixed(std::uint8_t command,
                                    std::uint16_t expected,
                                    ExchangeProfile profile = ExchangeProfile::normal);
    std::vector<std::uint8_t> variable(std::uint8_t command,
                                       std::uint16_t capacity = 36,
                                       ExchangeProfile profile = ExchangeProfile::normal);
    static bool packetValid(const std::vector<std::uint8_t>& packet,
                            std::uint8_t command);

private:
    void serviceAsyncF4();
    BridgeClient& bridge_;
};

} // namespace open1v
