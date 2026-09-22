#pragma once

#include "open1v/bridge_client.hpp"

#include <array>
#include <string>
#include <vector>

namespace open1v {

struct CameraPacket {
    std::string label;
    std::vector<std::uint8_t> bytes;
};

enum class CameraRead {
    identity,
    settings,
    cfn,
    pfn,
    clock,
    all,
};

enum class CfnBank {
    current,
    registered1,
    registered2,
    registered3,
};

class CameraProtocolSession {
public:
    explicit CameraProtocolSession(BridgeClient& bridge);
    // CLI convenience transaction. It opens a session, performs one selected
    // task (CameraRead::all is a predefined multi-action task), and sends F2.
    std::vector<CameraPacket> readOnce(CameraRead selection);

    // Long-lived application lifecycle for a future GUI. beginSession() is
    // called once, perform() may be called repeatedly, and endSession() sends
    // the only F2. A later perform() automatically starts the next logical
    // action while retaining the physical PC-mode session.
    std::vector<CameraPacket> beginSession();
    std::vector<CameraPacket> perform(CameraRead selection);
    void endSession();

    // Verified setting writes. Each method owns one complete session and
    // returns before/after wire packets for presentation or logging.
    std::vector<CameraPacket> setCameraId(std::uint8_t id);
    std::vector<CameraPacket> setClock(
        const std::array<std::uint8_t, 6>& bcdDateTime);
    std::vector<CameraPacket> setCurrentCfn(std::uint8_t number,
                                            std::uint8_t option);
    std::vector<CameraPacket> setCfn(CfnBank bank, std::uint8_t number,
                                     std::uint8_t option);
    std::vector<CameraPacket> setPfnEnabled(std::uint8_t number, bool enabled);
    std::vector<CameraPacket> setPfn27Dials(std::uint8_t option);
    std::vector<CameraPacket> setPfnBlock(
        std::uint8_t readCommand, std::span<const std::uint8_t> target);

    // Explicit lifecycle control is retained only for protocol diagnostics
    // that intentionally keep PC mode open across separate invocations.
    void closeSession();

    // Exposes captured and write-validation flows for protocol development.
    // It deliberately returns the same CameraPacket representation as read().
    std::vector<CameraPacket> runDiagnostic(const std::string& flow,
                                            bool allowWrite = false,
                                            const std::string& argument = {});
    std::vector<CameraPacket> writePfn(std::uint8_t readCommand,
        std::span<const std::uint8_t> expected,
        std::span<const std::uint8_t> target, bool authorized);
    std::vector<CameraPacket> writeCfn(std::uint8_t readCommand,
        std::span<const std::uint8_t> expected,
        std::span<const std::uint8_t> target, bool authorized);

private:
    void begin(std::vector<CameraPacket>& output);
    void nextAction(std::vector<CameraPacket>& output);
    void close();
    std::vector<std::uint8_t> fixed(std::uint8_t command,
                                    std::uint16_t expected,
                                    std::uint16_t timeoutMs = 1000);
    std::vector<std::uint8_t> variable(std::uint8_t command,
                                       std::uint16_t capacity = 36,
                                       std::uint16_t timeoutMs = 1000);
    void addFixed(std::vector<CameraPacket>& output, const char* label,
                  std::uint8_t command, std::uint16_t expected,
                  std::uint16_t delayMs = 78);
    void writeData(std::uint8_t command, std::span<const std::uint8_t> data);
    void roundTrip(std::vector<CameraPacket>& output, const char* label,
                   std::uint8_t readCommand, std::uint16_t readLength,
                   std::uint8_t writeCommand,
                   std::span<const std::uint8_t> original,
                   std::span<const std::uint8_t> test);
    BridgeClient& bridge_;
    bool sessionActive_{};
    bool actionUsed_{};
};

} // namespace open1v

