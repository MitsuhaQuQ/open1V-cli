#include "open1v/self_test.hpp"

#include "open1v/bridge_client.hpp"
#include "open1v/camera_protocol.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace open1v {
namespace {
class FakeTransport final : public ITransport {
public:
    std::vector<std::uint8_t> transact(std::span<const std::uint8_t> bytes,
                                      std::chrono::milliseconds) override {
        const auto request = decodeFrame(bytes);
        std::vector<std::uint8_t> payload{0};
        if (request.type == static_cast<std::uint8_t>(MessageType::ping)) {
            payload.insert(payload.end(), {'o', 'p', 'e', 'n', '1', 'V'});
        } else if (request.type == static_cast<std::uint8_t>(MessageType::status)) {
            payload.insert(payload.end(), {1, 2, 3});
        } else if (request.type == static_cast<std::uint8_t>(MessageType::exchange)) {
            std::vector<std::uint8_t> camera;
            if (request.payload.size() == 8) {
                payload = {4, 0, 0};
                return encodeFrame({static_cast<std::uint8_t>(request.type | 0x80),
                                    request.sequence, payload});
            }
            const auto command = request.payload.at(8);
            if (command == 0xff) camera = {0xf4};
            if (command == 0xf6) camera = {0xf6,0x0e,0x38,0xff,0x1a,0x17,0x41,0x18,
                                           0x10,0x1c,0x00,0x04,0x00,0x00,0x00,0x00,0xf1};
            if (command == 0xf1) camera = {0xf1,0x03,0x01,0x00,0x34,0x35};
            if (command == 0xf2) camera = goodbyeSeen_++ == 0
                ? std::vector<std::uint8_t>{0xf4}
                : std::vector<std::uint8_t>{0xf2};
            payload.push_back(static_cast<std::uint8_t>(camera.size()));
            payload.push_back(0);
            payload.insert(payload.end(), camera.begin(), camera.end());
        }
        return encodeFrame({static_cast<std::uint8_t>(request.type | 0x80),
                            request.sequence, payload});
    }

private:
    int goodbyeSeen_{};
};

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
} // namespace

int runSelfTests() {
    const std::string check = "123456789";
    require(crc16CcittFalse(std::span<const std::uint8_t>(
                reinterpret_cast<const std::uint8_t*>(check.data()), check.size())) == 0x29b1,
            "CRC test failed");
    const Frame original{0x10, 0x1234, {1, 2, 3}};
    const auto decoded = decodeFrame(encodeFrame(original));
    require(decoded.type == original.type && decoded.sequence == original.sequence &&
            decoded.payload == original.payload, "frame round-trip failed");
    FakeTransport transport;
    BridgeClient bridge(transport);
    require(bridge.ping() == "open1V", "bridge ping failed");
    require(bridge.status().raw == std::vector<std::uint8_t>({1, 2, 3}),
            "bridge status failed");
    CameraProtocolSession camera(bridge);
    const auto packets = camera.readOnce(CameraRead::identity);
    const auto found = std::find_if(packets.begin(), packets.end(),
        [](const CameraPacket& packet) {
            return !packet.bytes.empty() && packet.bytes[0] == 0xf1;
        });
    require(found != packets.end() && found->bytes[2] == 1 &&
            found->bytes[3] == 0 && found->bytes[4] == 0x34,
            "camera protocol identity failed");
    std::cout << "All offline tests passed.\n";
    return 0;
}

} // namespace open1v
