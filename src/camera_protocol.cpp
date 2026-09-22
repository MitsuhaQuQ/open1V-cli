#include "open1v/camera_protocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace open1v {
namespace {
using namespace std::chrono_literals;

bool packetValid(const std::vector<std::uint8_t>& packet,
                 std::uint8_t command) {
    if (packet.size() < 3 || packet[0] != command ||
        packet.size() != static_cast<std::size_t>(packet[1]) + 3) return false;
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i + 1 < packet.size(); ++i)
        sum = static_cast<std::uint8_t>(sum + packet[i]);
    return sum == packet.back();
}

void pause(std::uint16_t milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
} // namespace

CameraProtocolSession::CameraProtocolSession(BridgeClient& bridge) : bridge_(bridge) {}

std::vector<CameraPacket> CameraProtocolSession::writePfn(
    std::uint8_t readCommand, std::span<const std::uint8_t> expected,
    std::span<const std::uint8_t> target, bool authorized) {
    if (!authorized) throw std::runtime_error("P.Fn write requires --allow-write-debug");
    struct Mapping { std::uint8_t read; std::uint8_t write; std::uint8_t length; };
    static constexpr std::array<Mapping,16> mappings{{
        {0xd3,0xd4,4},{0xdd,0xde,5},{0xc5,0xb5,1},{0xc6,0xb6,1},
        {0xc1,0xb1,1},{0xc3,0xb3,2},{0xc4,0xb4,2},{0xcb,0xbb,1},
        {0xcc,0xbc,4},{0xca,0xba,1},{0xc7,0xb7,2},{0xc8,0xb8,2},
        {0xc0,0xb0,2},{0xcd,0xbd,5},{0xcf,0xbf,1},{0xce,0xbe,1}}};
    const auto found = std::find_if(mappings.begin(), mappings.end(),
        [readCommand](const Mapping& value) { return value.read == readCommand; });
    if (found == mappings.end()) throw std::runtime_error("unknown P.Fn read block");
    if (expected.size() != found->length || target.size() != found->length)
        throw std::runtime_error("P.Fn payload length does not match block");
    std::vector<CameraPacket> output;
    struct Guard { BridgeClient& bridge; ~Guard(){ try { bridge.release(); } catch (...) {} } } guard{bridge_};
    begin(output);
    pause(78);
    auto packet = fixed(found->read, static_cast<std::uint16_t>(found->length + 3));
    output.push_back({"P.Fn BEFORE", packet});
    if (!std::equal(expected.begin(), expected.end(), packet.begin() + 2))
        throw std::runtime_error("P.Fn expected value mismatch; nothing written");
    writeData(found->write, target); // Exactly one setting write in this PC session.
    pause(78);
    packet = fixed(found->read, static_cast<std::uint16_t>(found->length + 3));
    output.push_back({"P.Fn AFTER", packet});
    if (!std::equal(target.begin(), target.end(), packet.begin() + 2))
        throw std::runtime_error("P.Fn write read-back mismatch");
    return output;
}

std::vector<CameraPacket> CameraProtocolSession::writeCfn(
    std::uint8_t readCommand, std::span<const std::uint8_t> expected,
    std::span<const std::uint8_t> target, bool authorized) {
    if (!authorized) throw std::runtime_error("C.Fn write requires --allow-write-debug");
    struct Mapping { std::uint8_t read; std::uint8_t write; std::uint8_t length; };
    static constexpr std::array<Mapping,4> mappings{{
        {0xd1,0xd2,11},{0xd5,0xd6,10},{0xd7,0xd8,10},{0xd9,0xda,10}}};
    const auto found = std::find_if(mappings.begin(), mappings.end(),
        [readCommand](const Mapping& value) { return value.read == readCommand; });
    if (found == mappings.end()) throw std::runtime_error("unknown C.Fn group");
    if (expected.size() != found->length || target.size() != found->length)
        throw std::runtime_error("C.Fn payload length does not match group");
    std::vector<CameraPacket> output;
    struct Guard { BridgeClient& bridge; ~Guard(){ try { bridge.release(); } catch (...) {} } } guard{bridge_};
    begin(output);
    pause(78);
    auto packet = fixed(found->read, static_cast<std::uint16_t>(found->length + 3));
    output.push_back({"C.Fn BEFORE", packet});
    if (!std::equal(expected.begin(), expected.end(), packet.begin() + 2))
        throw std::runtime_error("C.Fn expected value mismatch; nothing written");
    writeData(found->write, target); // Exactly one setting write in this PC session.
    pause(78);
    packet = fixed(found->read, static_cast<std::uint16_t>(found->length + 3));
    output.push_back({"C.Fn AFTER", packet});
    if (!std::equal(target.begin(), target.end(), packet.begin() + 2))
        throw std::runtime_error("C.Fn write read-back mismatch");
    return output;
}

std::vector<std::uint8_t> CameraProtocolSession::fixed(
    std::uint8_t value, std::uint16_t expected, std::uint16_t timeoutMs) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto result = bridge_.exchangeResult(
            std::span<const std::uint8_t>(&value, 1), expected, timeoutMs, 50);
        if (result.status == 0 && packetValid(result.bytes, value) &&
            result.bytes.size() == expected) return result.bytes;
        if (result.bytes == std::vector<std::uint8_t>{0xf4}) {
            serviceAsyncF4();
        }
        pause(attempt == 0 ? 100 : 150);
    }
    throw std::runtime_error("debug fixed-length camera command failed");
}

std::vector<std::uint8_t> CameraProtocolSession::variable(
    std::uint8_t value, std::uint16_t capacity, std::uint16_t timeoutMs) {
    // A camera-originated F4 can arrive between any two P.Fn blocks. The
    // original Remote answers F4 and requests F6 before retrying the pending
    // read. Allow a longer bounded retry window because the camera may be
    // busy completing that asynchronous status update.
    for (int attempt = 0; attempt < 8; ++attempt) {
        const auto result = bridge_.exchangeResult(
            std::span<const std::uint8_t>(&value, 1), capacity, timeoutMs, 50);
        if (packetValid(result.bytes, value)) return result.bytes;
        if (result.bytes == std::vector<std::uint8_t>{0xf4}) {
            serviceAsyncF4();
        }
        pause(attempt < 2 ? 100 : 200);
    }
    std::ostringstream message;
    message << "debug variable-length camera command 0x"
            << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(value) << " failed";
    throw std::runtime_error(message.str());
}

void CameraProtocolSession::serviceAsyncF4() {
    const std::uint8_t acknowledge = 0xf4;
    (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
    pause(2);

    // F6 is the status response that the original application requests after
    // acknowledging an unsolicited F4. Failure is left to the pending read's
    // normal retry budget; the camera may still be busy and return no bytes.
    const std::uint8_t status = 0xf6;
    for (int attempt = 0; attempt < 3; ++attempt) {
        try {
            const auto result = bridge_.exchangeResult(
                std::span<const std::uint8_t>(&status, 1), 17, 1000, 50);
            if (result.status == 0 && packetValid(result.bytes, status)) return;
            if (result.bytes == std::vector<std::uint8_t>{0xf4}) {
                (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
                pause(2);
            }
        } catch (...) {
            // Do not turn an asynchronous status refresh into a hard failure
            // for the read that was already in progress.
        }
        pause(50);
    }
}

void CameraProtocolSession::begin(std::vector<CameraPacket>& output) {
    bridge_.release();
    const std::uint8_t hello = 0xff;
    const std::uint8_t acknowledge = 0xf4;
    auto reply = bridge_.exchange(std::span<const std::uint8_t>(&hello, 1), 1);
    if (reply != std::vector<std::uint8_t>{0xf4})
        throw std::runtime_error("debug session FF handshake failed");
    (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
    pause(300);
    output.push_back({"F6", fixed(0xf6, 17, 500)});
    output.push_back({"F1", fixed(0xf1, 6, 500)});

    const auto sync = bridge_.exchangeResult({}, 1, 1500, 50);
    if (sync.status == 0) {
        if (sync.bytes != std::vector<std::uint8_t>{0xf4})
            throw std::runtime_error("debug session unexpected sync byte");
        (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
        pause(2);
        output.push_back({"SYNC F6", fixed(0xf6, 17, 500)});
    } else if (sync.status != 0x04 || !sync.bytes.empty()) {
        throw std::runtime_error("debug session synchronization failed");
    }

    pause(1200);
    reply = bridge_.exchange(std::span<const std::uint8_t>(&hello, 1), 1);
    if (reply != std::vector<std::uint8_t>{0xf4})
        throw std::runtime_error("debug logical session FF failed");
    (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
    pause(63);
    output.push_back({"ACTION F1", fixed(0xf1, 6, 500)});
}

void CameraProtocolSession::nextAction(std::vector<CameraPacket>& output) {
    const std::uint8_t acknowledge = 0xf4;
    const auto deadline = std::chrono::steady_clock::now() + 3600ms;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto wait = static_cast<std::uint16_t>((std::min)(remaining.count(), 100LL));
        const auto idle = bridge_.exchangeResult({}, 1, wait ? wait : 1, 20);
        if (idle.status == 0) {
            if (idle.bytes != std::vector<std::uint8_t>{0xf4})
                throw std::runtime_error("unexpected idle camera byte");
            (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
        } else if (idle.status != 0x04 || !idle.bytes.empty()) {
            throw std::runtime_error("idle camera synchronization failed");
        }
    }
    const std::uint8_t hello = 0xff;
    auto reply = bridge_.exchange(std::span<const std::uint8_t>(&hello, 1), 1, 1000);
    if (reply != std::vector<std::uint8_t>{0xf4})
        throw std::runtime_error("next debug action FF failed");
    (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
    pause(78);
    output.push_back({"NEXT F1", fixed(0xf1, 6, 1000)});
}

void CameraProtocolSession::close() {
    const std::uint8_t closeByte = 0xf2;
    const std::uint8_t acknowledge = 0xf4;
    auto reply = bridge_.exchange(std::span<const std::uint8_t>(&closeByte, 1), 1);
    if (reply == std::vector<std::uint8_t>{0xf4}) {
        (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
        pause(305);
        reply = bridge_.exchange(std::span<const std::uint8_t>(&closeByte, 1), 1);
    }
    if (reply != std::vector<std::uint8_t>{0xf2})
        throw std::runtime_error("debug session cleanup failed");
    bridge_.release();
}

void CameraProtocolSession::addFixed(std::vector<CameraPacket>& output,
                                  const char* label, std::uint8_t command,
                                  std::uint16_t expected, std::uint16_t delayMs) {
    pause(delayMs);
    output.push_back({label, fixed(command, expected)});
}

void CameraProtocolSession::writeData(std::uint8_t command,
                                   std::span<const std::uint8_t> data) {
    const std::uint8_t acknowledge = 0xf4;
    // The camera can stay silent for several seconds after a setting write while
    // retaining the physical PC session. Treat silence as busy and wait; F2 is
    // never used here. Only the command byte is retried before its echo, so an
    // ambiguously acknowledged payload is never submitted twice.
    const auto commandDeadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < commandDeadline) {
        const auto result = bridge_.exchangeResult(
            std::span<const std::uint8_t>(&command, 1), 1, 1200, 50);
        if (result.status == 0 && result.bytes == std::vector<std::uint8_t>{0xf4}) {
            (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
            pause(250);
            continue;
        }
        if (result.status == 0x04 && result.bytes.empty()) {
            pause(250);
            continue;
        }
        if (result.status != 0 || result.bytes != std::vector<std::uint8_t>{command})
            throw std::runtime_error("debug write command was not echoed");
        std::vector<std::uint8_t> packet;
        packet.push_back(static_cast<std::uint8_t>(data.size()));
        packet.insert(packet.end(), data.begin(), data.end());
        std::uint8_t checksum = 0;
        for (const auto byte : data) checksum = static_cast<std::uint8_t>(checksum + byte);
        packet.push_back(checksum);
        auto payloadResult = bridge_.exchangeResult(packet, 1, 1500, 50);
        if (payloadResult.status == 0 &&
            payloadResult.bytes == std::vector<std::uint8_t>{0x01}) return;
        if (payloadResult.status != 0x04 || !payloadResult.bytes.empty())
            throw std::runtime_error("debug write returned an unexpected acknowledgement");

        const auto acknowledgementDeadline = std::chrono::steady_clock::now() + 15s;
        while (std::chrono::steady_clock::now() < acknowledgementDeadline) {
            payloadResult = bridge_.exchangeResult({}, 1, 500, 50);
            if (payloadResult.status == 0 &&
                payloadResult.bytes == std::vector<std::uint8_t>{0x01}) return;
            if (payloadResult.status == 0 &&
                payloadResult.bytes == std::vector<std::uint8_t>{0xf4}) {
                (void)bridge_.exchange(std::span<const std::uint8_t>(&acknowledge, 1), 0);
                continue;
            }
            if (payloadResult.status != 0x04 || !payloadResult.bytes.empty())
                throw std::runtime_error("debug write returned unexpected delayed data");
        }
        throw std::runtime_error(
            "debug write acknowledgement timed out; payload was not resent");
    }
    throw std::runtime_error("debug write remained busy for 15 seconds");
}

void CameraProtocolSession::roundTrip(
    std::vector<CameraPacket>& output, const char* label,
    std::uint8_t readCommand, std::uint16_t readLength,
    std::uint8_t writeCommand, std::span<const std::uint8_t> original,
    std::span<const std::uint8_t> test) {
    auto read = [&] {
        pause(78);
        return fixed(readCommand, readLength);
    };
    auto current = read();
    output.push_back({std::string(label) + " ORIGINAL", current});
    const auto payload = std::span<const std::uint8_t>(current).subspan(2, original.size());
    if (!std::equal(payload.begin(), payload.end(), original.begin(), original.end()))
        throw std::runtime_error(std::string(label) + " protected baseline mismatch; nothing written");

    bool restored = false;
    try {
        pause(78);
        writeData(writeCommand, test);
        current = read();
        output.push_back({std::string(label) + " TEST", current});
        const auto testRead = std::span<const std::uint8_t>(current).subspan(2, test.size());
        if (!std::equal(testRead.begin(), testRead.end(), test.begin(), test.end()))
            throw std::runtime_error(std::string(label) + " test read-back mismatch");
        pause(78);
        writeData(writeCommand, original);
        current = read();
        const auto restoredRead = std::span<const std::uint8_t>(current).subspan(2, original.size());
        restored = std::equal(restoredRead.begin(), restoredRead.end(),
                              original.begin(), original.end());
        output.push_back({std::string(label) + " RESTORED", current});
    } catch (...) {
        for (int attempt = 0; attempt < 5 && !restored; ++attempt) {
            try {
                pause(150);
                writeData(writeCommand, original);
                const auto verify = read();
                const auto value = std::span<const std::uint8_t>(verify).subspan(2, original.size());
                restored = std::equal(value.begin(), value.end(),
                                      original.begin(), original.end());
                if (restored) output.push_back({std::string(label) + " RESTORED", verify});
            } catch (...) {}
        }
        if (!restored)
            throw std::runtime_error(std::string(label) + " RESTORE FAILED");
        throw;
    }
    if (!restored) throw std::runtime_error(std::string(label) + " restore verification failed");
}

std::vector<CameraPacket> CameraProtocolSession::readOnce(CameraRead selection) {
    std::vector<CameraPacket> packets;
    try {
        auto opened = beginSession();
        packets.insert(packets.end(), opened.begin(), opened.end());
        auto result = perform(selection);
        packets.insert(packets.end(), result.begin(), result.end());
        endSession();
        return packets;
    } catch (...) {
        try { if (sessionActive_) endSession(); } catch (...) {}
        throw;
    }
}

std::vector<CameraPacket> CameraProtocolSession::clearFilmRecords() {
    if (sessionActive_)
        throw std::runtime_error("film-record clear requires a fresh camera session");

    std::vector<CameraPacket> packets;
    try {
        auto opened = beginSession();
        packets.insert(packets.end(), opened.begin(), opened.end());

        pause(78);
        const std::uint8_t command = 0xe2;
        const auto result = bridge_.exchangeResult(
            std::span<const std::uint8_t>(&command, 1), 2, 2000, 50);
        if (result.status != 0 || result.bytes.size() != 2 ||
            result.bytes[0] != command || result.bytes[1] == 0) {
            throw std::runtime_error(
                "film-record clear acknowledgement was missing or invalid; "
                "E2 was not retried because the camera state is ambiguous");
        }
        packets.push_back({"CLEAR E2", result.bytes});

        // The camera remains busy for an indeterminate period after E2. Do
        // not send F2 or start a new PC session here: the original application
        // keeps this session alive and refreshes E1/FC in place.
        pause(200);
        bool empty = false;
        const auto deadline = std::chrono::steady_clock::now() + 15s;
        while (std::chrono::steady_clock::now() < deadline) {
            try {
                auto e1 = fixed(0xe1, 5, 1000);
                packets.push_back({"CLEAR E1", e1});
                if (e1[2] == 0 && e1[3] == 0) {
                    empty = true;
                    break;
                }
            } catch (const std::exception&) {
                // E2 can temporarily leave only a completion byte available.
                // E1 is read-only and may safely be retried within this session.
            }
            pause(200);
        }
        if (!empty)
            throw std::runtime_error(
                "camera acknowledged the clear command, but E1 did not report zero rolls within 15 seconds");

        pause(78);
        packets.push_back({"CLEAR FC", fixed(0xfc, 5, 1000)});
        endSession();
        return packets;
    } catch (...) {
        try { if (sessionActive_) endSession(); } catch (...) {}
        throw;
    }
}

std::vector<CameraPacket> CameraProtocolSession::beginSession() {
    if (sessionActive_) throw std::runtime_error("camera session is already active");
    std::vector<CameraPacket> packets;
    sessionActive_ = true;
    actionUsed_ = false;
    try {
        begin(packets);
    } catch (...) {
        try { close(); } catch (...) {}
        sessionActive_ = false;
        throw;
    }
    return packets;
}

std::vector<CameraPacket> CameraProtocolSession::perform(CameraRead selection) {
    if (!sessionActive_) throw std::runtime_error("camera session is not active");
    std::vector<CameraPacket> output;
    if (actionUsed_) nextAction(output);

    auto cfn = [&] {
        addFixed(output, "D5", 0xd5, 13);
        addFixed(output, "D7", 0xd7, 13);
        addFixed(output, "D9", 0xd9, 13);
        addFixed(output, "D1", 0xd1, 14);
    };
    auto pfn = [&] {
        static constexpr std::array<std::uint8_t, 16> commands{
            0xd3,0xdd,0xc5,0xc6,0xc1,0xc3,0xc4,0xcb,
            0xcc,0xca,0xc7,0xc8,0xc0,0xcd,0xcf,0xce};
        for (const auto command : commands) {
            pause(command == 0xc4 ? 150 : 78);
            output.push_back({"P.Fn", variable(command, 36)});
        }
    };
    auto clock = [&] {
        addFixed(output, "F3", 0xf3, 9);
        addFixed(output, "A1", 0xa1, 5);
        addFixed(output, "D1", 0xd1, 14);
    };
    auto settings = [&] {
        addFixed(output, "E8", 0xe8, 11);
        addFixed(output, "FC", 0xfc, 5);
        addFixed(output, "E1", 0xe1, 5);
    };
    auto filmRecords = [&] {
        addFixed(output, "FILM E1", 0xe1, 5, 650);
        const auto& e1 = output.back().bytes;
        const auto reportedRolls = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(e1[2]) << 8 | e1[3]);
        if (reportedRolls > 100)
            throw std::runtime_error("camera reported an unreasonable film roll count");
        if (reportedRolls == 0) return;

        std::uint16_t rollsRead = 0;
        while (true) {
            pause(15);
            auto header = variable(0xe3, 36, 1200);
            output.push_back({"FILM E3", header});
            if (header.size() == 4 && header[1] == 1 &&
                header[2] == 0 && header[3] == 0) break;
            if (header.size() != 36)
                throw std::runtime_error("unexpected E3 film header length");
            if (rollsRead >= reportedRolls)
                throw std::runtime_error("camera returned more film rolls than E1 reported");

            std::size_t frameCount = 0;
            while (true) {
                pause(15);
                auto frame = variable(0xe4, 36, 1200);
                output.push_back({"FILM E4", frame});
                if (frame.size() == 4 && frame[1] == 1 &&
                    frame[2] == 0 && frame[3] == 0) break;
                if (++frameCount > 1000)
                    throw std::runtime_error("film roll exceeded the safety frame limit");
            }
            ++rollsRead;
        }
        if (rollsRead != reportedRolls)
            throw std::runtime_error("film roll count did not match E1");
    };

    // Even a failed operation consumed the current logical action. A caller
    // retry must therefore use nextAction() rather than continuing mid-flow.
    actionUsed_ = true;
    switch (selection) {
    case CameraRead::identity:
        // ACTION F1 is already part of beginSession(). A later identity action
        // uses the NEXT F1 emitted by nextAction().
        break;
    case CameraRead::settings: settings(); break;
    case CameraRead::cfn: cfn(); break;
    case CameraRead::pfn: pfn(); break;
    case CameraRead::clock: clock(); break;
    case CameraRead::filmRecords: filmRecords(); break;
    case CameraRead::all:
        cfn();
        nextAction(output); pfn();
        nextAction(output); clock();
        nextAction(output); settings();
        break;
    }
    return output;
}

void CameraProtocolSession::endSession() {
    if (!sessionActive_) throw std::runtime_error("camera session is not active");
    try {
        close();
        sessionActive_ = false;
        actionUsed_ = false;
    } catch (...) {
        sessionActive_ = false;
        actionUsed_ = false;
        throw;
    }
}

std::vector<CameraPacket> CameraProtocolSession::setCameraId(std::uint8_t id) {
    if (id > 99) throw std::runtime_error("camera ID must be 0..99");
    const bool ownsSession = !sessionActive_;
    auto output = ownsSession ? beginSession() : std::vector<CameraPacket>{};
    try {
        if (!ownsSession) nextAction(output);
        auto before = fixed(0xf1, 6); output.push_back({"ID BEFORE", before});
        const std::array<std::uint8_t,1> value{id};
        pause(78); writeData(0xf9, value);
        auto after = fixed(0xf1, 6); output.push_back({"ID AFTER", after});
        if ((after[3] & 0x7f) != id) throw std::runtime_error("camera ID read-back mismatch");
        if (ownsSession) endSession(); return output;
    } catch (...) { try { if (ownsSession && sessionActive_) endSession(); } catch (...) {} throw; }
}

std::vector<CameraPacket> CameraProtocolSession::setClock(
    const std::array<std::uint8_t,6>& value) {
    auto decimal=[](std::uint8_t v){if((v>>4)>9||(v&15)>9)throw std::runtime_error("clock contains invalid BCD");return unsigned((v>>4)*10+(v&15));};
    std::array<unsigned,6> d{}; for(std::size_t i=0;i<6;++i)d[i]=decimal(value[i]);
    static constexpr std::array<unsigned,12> days{31,28,31,30,31,30,31,31,30,31,30,31};
    const bool leap=(2000+d[0])%4==0; if(d[1]<1||d[1]>12||d[3]>23||d[4]>59||d[5]>59)throw std::runtime_error("clock value is outside its valid range");
    const auto maxDay=days[d[1]-1]+((d[1]==2&&leap)?1u:0u); if(d[2]<1||d[2]>maxDay)throw std::runtime_error("clock date is invalid");
    const bool ownsSession=!sessionActive_; auto output=ownsSession?beginSession():std::vector<CameraPacket>{};
    try {
        if(!ownsSession)nextAction(output);
        addFixed(output,"CLOCK BEFORE",0xf3,9,63); pause(63); writeData(0xf8,value);
        addFixed(output,"CLOCK AFTER",0xf3,9,78);
        const auto& after=output.back().bytes; if(!std::equal(value.begin(),value.end(),after.begin()+2))throw std::runtime_error("clock read-back mismatch");
        if(ownsSession)endSession(); return output;
    } catch (...) { try { if(ownsSession&&sessionActive_)endSession(); } catch (...) {} throw; }
}

std::vector<CameraPacket> CameraProtocolSession::setCurrentCfn(
    std::uint8_t number, std::uint8_t option) {
    return setCfn(CfnBank::current, number, option);
}

std::vector<CameraPacket> CameraProtocolSession::setCfn(
    CfnBank bank, std::uint8_t number, std::uint8_t option) {
    if(number<1||number>19||option>(number==19?7:3))throw std::runtime_error("C.Fn number or option is outside its valid range");
    std::uint8_t readCommand=0xd1, writeCommand=0xd2;
    switch(bank){
    case CfnBank::current: break;
    case CfnBank::registered1: readCommand=0xd5;writeCommand=0xd6;break;
    case CfnBank::registered2: readCommand=0xd7;writeCommand=0xd8;break;
    case CfnBank::registered3: readCommand=0xd9;writeCommand=0xda;break;
    }
    const bool ownsSession=!sessionActive_; auto output=ownsSession?beginSession():std::vector<CameraPacket>{};
    try {
        if(!ownsSession)nextAction(output);
        auto before=fixed(readCommand,readCommand==0xd1?14:13); output.push_back({"C.Fn BEFORE",before});
        const auto payloadSize=static_cast<std::size_t>(readCommand==0xd1?11:10);
        std::vector<std::uint8_t> data(before.begin()+2,before.begin()+2+payloadSize);
        const auto encoded=static_cast<std::uint8_t>(1u<<option); const auto at=(number-1)/2;
        if(number==19)data[at]=encoded; else if(number&1)data[at]=static_cast<std::uint8_t>((data[at]&0xf0)|encoded); else data[at]=static_cast<std::uint8_t>((data[at]&0x0f)|(encoded<<4));
        pause(78); writeData(writeCommand,data); auto after=fixed(readCommand,readCommand==0xd1?14:13); output.push_back({"C.Fn AFTER",after});
        if(!std::equal(data.begin(),data.end(),after.begin()+2))throw std::runtime_error("C.Fn read-back mismatch");
        if(ownsSession)endSession(); return output;
    } catch (...) { try { if(ownsSession&&sessionActive_)endSession(); } catch (...) {} throw; }
}

std::vector<CameraPacket> CameraProtocolSession::setPfnEnabled(
    std::uint8_t number, bool enabled) {
    if(number<1||number>30)throw std::runtime_error("P.Fn number must be 1..30");
    const bool ownsSession=!sessionActive_; auto output=ownsSession?beginSession():std::vector<CameraPacket>{};
    try {
        if(!ownsSession)nextAction(output);
        auto before=variable(0xdd,16); output.push_back({"P.Fn BEFORE",before});
        std::array<std::uint8_t,5> data{}; std::copy_n(before.begin()+2,5,data.begin());
        const auto group=(number-1)/8, bit=(number-1)%8, at=3-group;
        const auto mask=static_cast<std::uint8_t>(1u<<bit);
        if(enabled)data[at]|=mask;else data[at]&=static_cast<std::uint8_t>(~mask);
        pause(78); writeData(0xde,data); auto after=variable(0xdd,16); output.push_back({"P.Fn AFTER",after});
        if(!std::equal(data.begin(),data.end(),after.begin()+2))throw std::runtime_error("P.Fn read-back mismatch");
        if(ownsSession)endSession(); return output;
    } catch (...) { try { if(ownsSession&&sessionActive_)endSession(); } catch (...) {} throw; }
}

std::vector<CameraPacket> CameraProtocolSession::setPfn27Dials(std::uint8_t option) {
    if(option>2)throw std::runtime_error("P.Fn-27 option must be 0, 1, or 2");
    const bool ownsSession=!sessionActive_;auto output=ownsSession?beginSession():std::vector<CameraPacket>{};
    try {
        if(!ownsSession)nextAction(output);
        auto before=variable(0xdd,16);output.push_back({"P.Fn-27 BEFORE",before});
        std::array<std::uint8_t,5> data{};std::copy_n(before.begin()+2,5,data.begin());data[4]=option;
        pause(78);writeData(0xde,data);auto after=variable(0xdd,16);output.push_back({"P.Fn-27 AFTER",after});
        if(after.at(6)!=option)throw std::runtime_error("P.Fn-27 option read-back mismatch");
        if(ownsSession)endSession();return output;
    }catch(...){try{if(ownsSession&&sessionActive_)endSession();}catch(...){}throw;}
}

std::vector<CameraPacket> CameraProtocolSession::setPfnBlock(
    std::uint8_t readCommand, std::span<const std::uint8_t> target) {
    struct Mapping { std::uint8_t read, write, length; };
    static constexpr std::array<Mapping,15> mappings{{
        {0xdd,0xde,5},{0xc5,0xb5,1},{0xc6,0xb6,1},{0xc1,0xb1,1},
        {0xc3,0xb3,2},{0xc4,0xb4,2},{0xcb,0xbb,1},{0xcc,0xbc,4},
        {0xca,0xba,1},{0xc7,0xb7,2},{0xc8,0xb8,2},{0xc0,0xb0,2},
        {0xcd,0xbd,5},{0xcf,0xbf,1},{0xce,0xbe,1}}};
    const auto found=std::find_if(mappings.begin(),mappings.end(),
        [readCommand](const Mapping& m){return m.read==readCommand;});
    if(found==mappings.end()||target.size()!=found->length)
        throw std::runtime_error("unsupported P.Fn parameter block");
    const bool ownsSession=!sessionActive_;auto output=ownsSession?beginSession():std::vector<CameraPacket>{};
    try {
        if(!ownsSession)nextAction(output);
        auto before=variable(readCommand,16);output.push_back({"P.Fn BLOCK BEFORE",before});
        pause(78);writeData(found->write,target);
        auto after=variable(readCommand,16);output.push_back({"P.Fn BLOCK AFTER",after});
        if(!std::equal(target.begin(),target.end(),after.begin()+2))
            throw std::runtime_error("P.Fn parameter read-back mismatch");
        if(ownsSession)endSession();return output;
    }catch(...){try{if(ownsSession&&sessionActive_)endSession();}catch(...){}throw;}
}

void CameraProtocolSession::closeSession() {
    if (sessionActive_) endSession();
    else (void)runDiagnostic("exit-debug");
}

std::vector<CameraPacket> CameraProtocolSession::runDiagnostic(
                                                 const std::string& flow,
                                                 bool allowWriteDebug,
                                                 const std::string& argument) {
    std::vector<CameraPacket> output;
    struct Guard {
        BridgeClient& bridge;
        ~Guard() { try { bridge.release(); } catch (...) {} }
    } guard{bridge_};
    if (flow == "exit-debug") {
        close();
        return output;
    }
    begin(output);

    if (flow == "handshake-debug") {
        return output;
    }
    if (flow == "settings-debug") {
        addFixed(output, "E8", 0xe8, 11);
        addFixed(output, "FC", 0xfc, 5);
        addFixed(output, "E1", 0xe1, 5);
    } else if (flow == "cfn-debug") {
        addFixed(output, "D5", 0xd5, 13);
        addFixed(output, "D7", 0xd7, 13, 63);
        addFixed(output, "D9", 0xd9, 13, 63);
        addFixed(output, "D1", 0xd1, 14);
    } else if (flow == "pfn-debug") {
        static constexpr std::array<std::uint8_t, 16> commands{
            0xd3,0xdd,0xc5,0xc6,0xc1,0xc3,0xc4,0xcb,
            0xcc,0xca,0xc7,0xc8,0xc0,0xcd,0xcf,0xce};
        for (const auto command : commands) {
            pause(78);
            output.push_back({"P.Fn", variable(command, 16)});
        }
    } else if (flow == "clock-debug") {
        addFixed(output, "F3", 0xf3, 9, 63);
        addFixed(output, "F3", 0xf3, 9, 63);
        addFixed(output, "A1", 0xa1, 5, 63);
        addFixed(output, "D1", 0xd1, 14, 63);
    } else if (flow == "unknown-read-debug") {
        // Read-only sampling of fields whose transport is known but whose
        // semantics are not. Repeat them across logical actions so captures
        // can distinguish time-varying state from stable stored settings.
        for (int sample = 1; sample <= 3; ++sample) {
            addFixed(output, ("S" + std::to_string(sample) + " F3").c_str(),
                     0xf3, 9, 63);
            addFixed(output, ("S" + std::to_string(sample) + " A1").c_str(),
                     0xa1, 5, 63);
            addFixed(output, ("S" + std::to_string(sample) + " D1").c_str(),
                     0xd1, 14, 63);
            output.push_back({"S" + std::to_string(sample) + " DD",
                              variable(0xdd, 16)});
            if (sample != 3) nextAction(output);
        }
    } else if (flow == "film-header-debug" || flow == "film-download-debug") {
        addFixed(output, "E1", 0xe1, 5, 650);
        const auto rolls = static_cast<std::uint16_t>(output.back().bytes[2] << 8) |
                           output.back().bytes[3];
        if (rolls != 0) {
            pause(15);
            output.push_back({"E3", variable(0xe3, 36, 1200)});
        }
        if (flow == "film-download-debug") {
            for (std::uint16_t roll = 0; roll < rolls && roll < 100; ++roll) {
                for (int record = 0; record < 40; ++record) {
                    pause(15);
                    auto packet = variable(0xe4, 36, 1000);
                    output.push_back({"E4", packet});
                    if (packet.size() == 4 && packet[1] == 1 &&
                        packet[2] == 0 && packet[3] == 0) break;
                }
                if (roll + 1 < rolls) {
                    pause(15);
                    output.push_back({"E3", variable(0xe3, 36, 1200)});
                }
            }
        }
    } else if (flow == "continuous-debug") {
        addFixed(output, "D5", 0xd5, 13);
        addFixed(output, "D7", 0xd7, 13);
        addFixed(output, "D9", 0xd9, 13);
        addFixed(output, "D1", 0xd1, 14);
        nextAction(output);
        static constexpr std::array<std::uint8_t, 16> pfn{
            0xd3,0xdd,0xc5,0xc6,0xc1,0xc3,0xc4,0xcb,
            0xcc,0xca,0xc7,0xc8,0xc0,0xcd,0xcf,0xce};
        for (const auto command : pfn) {
            pause(78); output.push_back({"P.Fn", variable(command, 36)});
        }
        nextAction(output);
        addFixed(output, "F3", 0xf3, 9);
        addFixed(output, "A1", 0xa1, 5);
        addFixed(output, "D1", 0xd1, 14);
        nextAction(output);
        addFixed(output, "E8", 0xe8, 11);
        addFixed(output, "FC", 0xfc, 5);
        addFixed(output, "E1", 0xe1, 5);
    } else if (flow == "clock-set-debug") {
        if (!allowWriteDebug)
            throw std::runtime_error("clock write debug requires --allow-write-debug");
        if (argument.size() != 12 ||
            !std::all_of(argument.begin(), argument.end(), [](char c) { return c >= '0' && c <= '9'; }))
            throw std::runtime_error("clock value must be YYMMDDhhmmss");
        std::array<unsigned,6> value{};
        std::array<std::uint8_t,6> bcd{};
        for (std::size_t i = 0; i < value.size(); ++i) {
            value[i] = static_cast<unsigned>((argument[i * 2] - '0') * 10 +
                                             argument[i * 2 + 1] - '0');
            bcd[i] = static_cast<std::uint8_t>(((argument[i * 2] - '0') << 4) |
                                               (argument[i * 2 + 1] - '0'));
        }
        static constexpr std::array<unsigned,12> days{31,28,31,30,31,30,31,31,30,31,30,31};
        const unsigned year = 2000 + value[0];
        const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
        if (value[1] < 1 || value[1] > 12 || value[3] > 23 || value[4] > 59 || value[5] > 59)
            throw std::runtime_error("clock value is outside its valid range");
        const auto maxDay = days[value[1] - 1] + ((value[1] == 2 && leap) ? 1u : 0u);
        if (value[2] < 1 || value[2] > maxDay)
            throw std::runtime_error("clock date is invalid");
        addFixed(output, "CLOCK AUX A1", 0xa1, 5, 63);
        addFixed(output, "CLOCK AUX D1", 0xd1, 14, 63);
        pause(63);
        writeData(0xf8, bcd);
        addFixed(output, "CLOCK F3 VERIFY", 0xf3, 9, 78);
    } else if (flow.ends_with("-roundtrip-debug")) {
        if (!allowWriteDebug)
            throw std::runtime_error("write debug flow requires --allow-write-debug");
        if (flow == "pfn4-roundtrip-debug") {
            const std::array<std::uint8_t,2> original{0xa0,0x10}, test{0x98,0x10};
            roundTrip(output, "P.Fn-4", 0xc3, 5, 0xb3, original, test);
        } else if (flow == "pfn5-roundtrip-debug") {
            const std::array<std::uint8_t,2> original{0x70,0x08}, test{0x68,0x08};
            roundTrip(output, "P.Fn-5", 0xc4, 5, 0xb4, original, test);
        } else if (flow == "pfn3-roundtrip-debug") {
            const std::array<std::uint8_t,1> original{0x20}, test{0x10};
            roundTrip(output, "P.Fn-3", 0xc1, 4, 0xb1, original, test);
        } else if (flow == "pfn12-roundtrip-debug") {
            const std::array<std::uint8_t,1> original{0x40}, test{0x00};
            roundTrip(output, "P.Fn-12", 0xcb, 4, 0xbb, original, test);
        } else if (flow == "pfn-global-roundtrip-debug") {
            const std::array<std::uint8_t,5> original{0x13,0x02,0xd8,0x00,0x00};
            const std::array<std::uint8_t,5> test{0x13,0x02,0xd8,0x00,0x01};
            roundTrip(output, "P.Fn global", 0xdd, 8, 0xde, original, test);
        } else if (flow == "cfn19-roundtrip-debug") {
            const std::array<std::uint8_t,11> original{
                0x21,0x81,0x12,0x21,0x11,0x11,0x11,0x11,0x11,0x01,0x02};
            auto test = original; test[9] = 0x08;
            roundTrip(output, "C.Fn-19", 0xd1, 14, 0xd2, original, test);
        } else if (flow == "camera-id-roundtrip-debug") {
            const std::array<std::uint8_t,1> original{0x00}, test{0x0c};
            // F1 contains type before the ID, so protect/compare a dedicated slice.
            auto current = fixed(0xf1, 6);
            output.push_back({"CAMERA ID ORIGINAL", current});
            if ((current[3] & 0x7f) != original[0])
                throw std::runtime_error("camera ID protected baseline mismatch; nothing written");
            pause(78); writeData(0xf9, test);
            current = fixed(0xf1, 6);
            if ((current[3] & 0x7f) != test[0])
                throw std::runtime_error("camera ID test read-back mismatch; run restore-debug");
            output.push_back({"CAMERA ID TEST", current});
            pause(78); writeData(0xf9, original);
            current = fixed(0xf1, 6);
            if ((current[3] & 0x7f) != original[0])
                throw std::runtime_error("camera ID restore verification failed");
            output.push_back({"CAMERA ID RESTORED", current});
        } else if (flow == "shooting-mask-roundtrip-debug" ||
                   flow == "shooting-width16-roundtrip-debug" ||
                   flow == "shooting-width8-roundtrip-debug") {
            const std::array<std::uint8_t,8> original{
                0xff,0xff,0x0c,0x3f,0x00,0x08,0x7f,0x00};
            const std::array<std::uint8_t,8> maskTest{
                0xf7,0xff,0x0c,0x3f,0x00,0x08,0x7f,0x00};
            const std::array<std::uint8_t,8> width16{
                0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00};
            const std::array<std::uint8_t,8> width8{
                0xf6,0x09,0x00,0x00,0x00,0x00,0x00,0x00};
            const auto& target = flow == "shooting-mask-roundtrip-debug" ? maskTest :
                                 (flow == "shooting-width16-roundtrip-debug" ? width16 : width8);
            const std::uint8_t targetWidth = flow == "shooting-width16-roundtrip-debug" ? 0x10 :
                                             (flow == "shooting-width8-roundtrip-debug" ? 0x08 : 0x20);
            auto readMask = [&] {
                pause(78); return fixed(0xe8, 11);
            };
            auto maskEquals = [](const std::vector<std::uint8_t>& packet,
                                 const std::array<std::uint8_t,8>& value) {
                return std::equal(value.begin(), value.end(), packet.begin() + 2,
                                  packet.begin() + 10);
            };
            auto apply = [&](std::uint8_t width,
                             const std::array<std::uint8_t,8>& mask) {
                pause(78); writeData(0xe7, std::span<const std::uint8_t>(&width, 1));
                pause(78); writeData(0xe9, mask);
            };
            auto current = readMask();
            output.push_back({"SHOOTING MASK ORIGINAL", current});
            if (!maskEquals(current, original))
                throw std::runtime_error("shooting mask protected baseline mismatch; nothing written");
            bool restored = false;
            try {
                apply(targetWidth, target);
                current = readMask();
                if (!maskEquals(current, target))
                    throw std::runtime_error("shooting mask test read-back mismatch");
                output.push_back({"SHOOTING MASK TEST", current});
                apply(0x20, original);
                current = readMask();
                restored = maskEquals(current, original);
                output.push_back({"SHOOTING MASK RESTORED", current});
            } catch (...) {
                for (int attempt = 0; attempt < 5 && !restored; ++attempt) {
                    try {
                        pause(150); apply(0x20, original);
                        current = readMask(); restored = maskEquals(current, original);
                    } catch (...) {}
                }
                if (!restored) throw std::runtime_error("shooting mask RESTORE FAILED");
                throw;
            }
            if (!restored) throw std::runtime_error("shooting mask restore verification failed");
        } else {
            throw std::runtime_error("unknown write debug flow");
        }
    } else {
        throw std::runtime_error("unknown camera debug flow");
    }
    return output;
}

} // namespace open1v




