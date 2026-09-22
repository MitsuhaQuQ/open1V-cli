#include "open1v/bridge_client.hpp"

#include <chrono>
#include <stdexcept>

namespace open1v {
namespace {
void appendLe16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8));
}

std::uint16_t readLe16(std::span<const std::uint8_t> bytes, std::size_t at) {
    return static_cast<std::uint16_t>(bytes[at]) |
           static_cast<std::uint16_t>(bytes[at + 1] << 8);
}

std::uint32_t readLe32(std::span<const std::uint8_t> bytes, std::size_t at) {
    return static_cast<std::uint32_t>(bytes[at]) |
           (static_cast<std::uint32_t>(bytes[at + 1]) << 8) |
           (static_cast<std::uint32_t>(bytes[at + 2]) << 16) |
           (static_cast<std::uint32_t>(bytes[at + 3]) << 24);
}
} // namespace

BridgeClient::BridgeClient(ITransport& transport) : transport_(transport) {}

Frame BridgeClient::request(MessageType type, std::span<const std::uint8_t> payload,
                            int timeoutMs, bool requireOk) {
    const auto sequence = nextSequence_++;
    const auto wire = encodeFrame(
        Frame{static_cast<std::uint8_t>(type), sequence,
              std::vector<std::uint8_t>(payload.begin(), payload.end())});
    const auto response = decodeFrame(
        transport_.transact(wire, std::chrono::milliseconds(timeoutMs)));
    if (response.sequence != sequence ||
        response.type != (static_cast<std::uint8_t>(type) | 0x80)) {
        throw std::runtime_error("unexpected bridge response");
    }
    if (response.payload.empty() || (requireOk && response.payload[0] != 0)) {
        throw std::runtime_error("bridge reported an error");
    }
    return response;
}

std::string BridgeClient::ping() {
    const auto response = request(MessageType::ping, std::span<const std::uint8_t>{});
    return std::string(response.payload.begin() + 1, response.payload.end());
}

BridgeStatus BridgeClient::status() {
    auto response = request(MessageType::status, std::span<const std::uint8_t>{});
    return {{response.payload.begin() + 1, response.payload.end()}};
}

LinkStatus BridgeClient::linkStatus() {
    const auto response = request(MessageType::linkStatus,
                                  std::span<const std::uint8_t>{});
    if (response.payload.size() != 29) {
        throw std::runtime_error("short bridge link-status response");
    }
    return {readLe32(response.payload, 1), readLe32(response.payload, 5),
            readLe32(response.payload, 9), readLe32(response.payload, 13),
            readLe32(response.payload, 17), readLe32(response.payload, 21),
            readLe32(response.payload, 25)};
}

std::vector<std::uint8_t> BridgeClient::exchange(
    std::span<const std::uint8_t> tx, std::uint16_t expectedBytes,
    std::uint16_t firstByteTimeoutMs, std::uint16_t interByteTimeoutMs) {
    const auto result = exchangeResult(tx, expectedBytes, firstByteTimeoutMs,
                                       interByteTimeoutMs);
    if (result.status != 0) throw std::runtime_error("bridge camera exchange failed");
    return result.bytes;
}

ExchangeResult BridgeClient::exchangeResult(
    std::span<const std::uint8_t> tx, std::uint16_t expectedBytes,
    std::uint16_t firstByteTimeoutMs, std::uint16_t interByteTimeoutMs) {
    if (tx.size() > 64) {
        throw std::runtime_error("camera transmission is too large");
    }
    std::vector<std::uint8_t> payload;
    appendLe16(payload, expectedBytes);
    appendLe16(payload, firstByteTimeoutMs);
    appendLe16(payload, interByteTimeoutMs);
    appendLe16(payload, static_cast<std::uint16_t>(tx.size()));
    payload.insert(payload.end(), tx.begin(), tx.end());
    const auto timeout = firstByteTimeoutMs + interByteTimeoutMs * expectedBytes + 750;
    auto response = request(MessageType::exchange, payload, timeout, false);
    if (response.payload.size() < 3) {
        throw std::runtime_error("short bridge exchange response");
    }
    const auto count = readLe16(response.payload, 1);
    if (response.payload.size() != static_cast<std::size_t>(count) + 3) {
        throw std::runtime_error("inconsistent camera response length");
    }
    return {response.payload[0],
            {response.payload.begin() + 3, response.payload.end()}};
}

void BridgeClient::release() {
    (void)request(MessageType::release, std::span<const std::uint8_t>{});
}

} // namespace open1v
