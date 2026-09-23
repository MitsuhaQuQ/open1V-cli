#include "open1v/self_test.hpp"

#include "open1v/bridge_client.hpp"
#include "open1v/camera_protocol.hpp"
#include "open1v/camera_data.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace open1v {
namespace {
class FakeTransport final : public ITransport {
public:
    explicit FakeTransport(bool filmCountChanges = false)
        : filmCountChanges_(filmCountChanges) {}

    std::vector<std::uint8_t> transact(std::span<const std::uint8_t> bytes,
                                      std::chrono::milliseconds) override {
        const auto request = decodeFrame(bytes);
        std::vector<std::uint8_t> payload{0};
        if (request.type == static_cast<std::uint8_t>(MessageType::ping)) {
            payload.insert(payload.end(), {'o', 'p', 'e', 'n', '1', 'V'});
        } else if (request.type == static_cast<std::uint8_t>(MessageType::status)) {
            payload.insert(payload.end(), {1, 2, 3});
        } else if (request.type == static_cast<std::uint8_t>(MessageType::exchange) ||
                   request.type == static_cast<std::uint8_t>(MessageType::profiledExchange)) {
            std::vector<std::uint8_t> camera;
            const bool profiled = request.type ==
                static_cast<std::uint8_t>(MessageType::profiledExchange);
            const std::size_t commandOffset = profiled ? 5 : 8;
            if (request.payload.size() == commandOffset) {
                ++idlePolls_;
                payload = {4, 0, 0};
                return encodeFrame({static_cast<std::uint8_t>(request.type | 0x80),
                                    request.sequence, payload});
            }
            const auto command = request.payload.at(commandOffset);
            if (command == 0xff) { ++helloCount_; camera = {0xf4}; }
            if (command == 0xf6) camera = {0xf6,0x0e,0x38,0xff,0x1a,0x17,0x41,0x18,
                                           0x10,0x1c,0x00,0x04,0x00,0x00,0x00,0x00,0xf1};
            if (command == 0xf1) camera = {0xf1,0x03,0x01,0x00,0x34,0x35};
            if (command == 0xf2) { ++goodbyeCount_; camera = goodbyeSeen_++ == 0
                ? std::vector<std::uint8_t>{0xf4}
                : std::vector<std::uint8_t>{0xf2}; }
            if (filmCountChanges_ && command == 0xe1)
                camera = {0xe1, 0x02, 0x00, 0x01, 0x01};
            if (filmCountChanges_ && command == 0xe3) {
                if (filmHeaders_++ < 2) {
                    camera.assign(36, 0);
                    camera[0] = 0xe3;
                    camera[1] = 33;
                } else {
                    camera = {0xe3, 0x01, 0x00, 0x00};
                }
            }
            if (filmCountChanges_ && command == 0xe4)
                camera = {0xe4, 0x01, 0x00, 0x00};
            payload.push_back(static_cast<std::uint8_t>(camera.size()));
            payload.push_back(0);
            payload.insert(payload.end(), camera.begin(), camera.end());
        }
        return encodeFrame({static_cast<std::uint8_t>(request.type | 0x80),
                            request.sequence, payload});
    }

    [[nodiscard]] int helloCount() const noexcept { return helloCount_; }
    [[nodiscard]] int goodbyeCount() const noexcept { return goodbyeCount_; }
    [[nodiscard]] int idlePolls() const noexcept { return idlePolls_; }

private:
    int goodbyeSeen_{};
    int helloCount_{};
    int goodbyeCount_{};
    int idlePolls_{};
    int filmHeaders_{};
    bool filmCountChanges_{};
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
    std::vector<CameraPacket> modelPackets{{"F1", {0xf1,0x03,0x01,0x8c,0x34,0xc1}}};
    const auto identity = decodeIdentity(modelPackets);
    require(identity.modelType == 1 && identity.cameraId == 12 && identity.status == 0x34,
            "camera identity model failed");
    std::vector<std::uint8_t> cfn(14, 0); cfn[0] = 0xd1;
    setCfnOption(cfn, 1, 2); setCfnOption(cfn, 19, 7);
    const auto cfnOptions = decodeCfnOptions(cfn);
    require(cfnOptions[0] == 2 && cfnOptions[18] == 7,
            "C.Fn model edit failed");
    std::vector<std::uint8_t> pfn(8, 0); pfn[0] = 0xdd;
    setPfnEnabled(pfn, 1, true); setPfnEnabled(pfn, 30, true);
    const auto pfnEnabled = decodePfnEnabled(pfn);
    require(pfnEnabled[0] && pfnEnabled[29], "P.Fn model edit failed");
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

    // Canon's desktop software retains one physical PC-mode connection while
    // it performs several logical actions. Lock that behavior down: later
    // actions use the in-session boundary and F2 is reserved for final close.
    FakeTransport persistentTransport;
    BridgeClient persistentBridge(persistentTransport);
    CameraProtocolSession persistentCamera(persistentBridge);
    (void)persistentCamera.beginSession();
    require(persistentCamera.sessionActive(), "persistent session did not open");
    (void)persistentCamera.perform(CameraRead::identity);
    (void)persistentCamera.perform(CameraRead::identity);
    require(persistentTransport.goodbyeCount() == 0,
            "persistent session exited between logical actions");
    require(persistentTransport.helloCount() == 3,
            "persistent session did not use the captured next-action handshake");
    require(persistentTransport.idlePolls() > 0,
            "persistent session did not wait for the next action boundary");
    persistentCamera.endSession();
    require(!persistentCamera.sessionActive(), "persistent session did not close");
    require(persistentTransport.goodbyeCount() == 2,
            "persistent session did not perform the captured two-stage exit");

    // E1 is advisory. E3's explicit all-end packet is authoritative when the
    // camera exposes another roll segment after the E1 snapshot was read.
    FakeTransport changingFilmTransport(true);
    BridgeClient changingFilmBridge(changingFilmTransport);
    CameraProtocolSession changingFilmCamera(changingFilmBridge);
    const auto filmPackets = changingFilmCamera.readOnce(CameraRead::filmRecords);
    const auto filmHeaders = std::count_if(filmPackets.begin(), filmPackets.end(),
        [](const CameraPacket& packet) {
            return packet.label == "FILM E3" && packet.bytes.size() == 36;
        });
    require(filmHeaders == 2,
            "film download stopped at the stale E1 roll-count snapshot");
    std::cout << "All offline tests passed.\n";
    return 0;
}

} // namespace open1v
