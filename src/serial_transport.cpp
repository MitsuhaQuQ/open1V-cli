#include "open1v/serial_transport.hpp"
#include "open1v/bridge_protocol.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <devguid.h>
#include <setupapi.h>

#include <array>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

namespace open1v {
namespace {
std::runtime_error windowsError(const char* what) {
    return std::runtime_error(std::string(what) + " (Windows error " +
                              std::to_string(GetLastError()) + ")");
}

std::wstring property(HDEVINFO devices, SP_DEVINFO_DATA& device, DWORD key) {
    DWORD type = 0;
    DWORD needed = 0;
    SetupDiGetDeviceRegistryPropertyW(devices, &device, key, &type, nullptr, 0,
                                      &needed);
    if (!needed) return {};
    std::vector<std::byte> buffer(needed);
    if (!SetupDiGetDeviceRegistryPropertyW(
            devices, &device, key, &type,
            reinterpret_cast<PBYTE>(buffer.data()), needed, nullptr)) return {};
    return reinterpret_cast<const wchar_t*>(buffer.data());
}

std::wstring findArduinoPort() {
    HDEVINFO devices = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr,
                                             DIGCF_PRESENT);
    if (devices == INVALID_HANDLE_VALUE) return {};
    std::wstring match;
    for (DWORD index = 0;; ++index) {
        SP_DEVINFO_DATA device{sizeof(device)};
        if (!SetupDiEnumDeviceInfo(devices, index, &device)) break;
        const auto hardwareId = property(devices, device, SPDRP_HARDWAREID);
        if (hardwareId.find(L"VID_2341&PID_1002") == std::wstring::npos) continue;
        const auto friendlyName = property(devices, device, SPDRP_FRIENDLYNAME);
        const auto begin = friendlyName.rfind(L"(COM");
        const auto end = friendlyName.rfind(L')');
        if (begin == std::wstring::npos || end <= begin + 1) continue;
        const auto port = friendlyName.substr(begin + 1, end - begin - 1);
        if (!match.empty() && match != port) {
            SetupDiDestroyDeviceInfoList(devices);
            throw std::runtime_error("multiple Arduino UNO R4 WiFi serial ports found; use --port");
        }
        match = port;
    }
    SetupDiDestroyDeviceInfoList(devices);
    return match;
}

std::wstring devicePath(const std::wstring& port) {
    return L"\\\\.\\" + port;
}
} // namespace

struct SerialTransport::Impl {
    HANDLE port{INVALID_HANDLE_VALUE};
    ~Impl() {
        if (port != INVALID_HANDLE_VALUE) CloseHandle(port);
    }
};

SerialTransport::SerialTransport(std::wstring port) : impl_(std::make_unique<Impl>()) {
    if (port.empty()) port = findArduinoPort();
    if (port.empty()) throw std::runtime_error("Arduino UNO R4 WiFi serial port not found");
    impl_->port = CreateFileW(devicePath(port).c_str(), GENERIC_READ | GENERIC_WRITE,
                              0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (impl_->port == INVALID_HANDLE_VALUE) throw windowsError("cannot open Arduino serial port");

    DCB settings{};
    settings.DCBlength = sizeof(settings);
    if (!GetCommState(impl_->port, &settings)) throw windowsError("cannot read serial settings");
    settings.BaudRate = CBR_115200;
    settings.ByteSize = 8;
    settings.Parity = NOPARITY;
    settings.StopBits = ONESTOPBIT;
    settings.fBinary = TRUE;
    settings.fDtrControl = DTR_CONTROL_ENABLE;
    settings.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(impl_->port, &settings)) throw windowsError("cannot configure serial port");
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.WriteTotalTimeoutConstant = 1000;
    if (!SetCommTimeouts(impl_->port, &timeouts)) throw windowsError("cannot configure serial timeouts");
    // The official UNO R4 WiFi USB bridge applies the CDC line state to its
    // RA4 link asynchronously. Do not let the first protocol frame race that
    // transition immediately after opening the COM port.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    PurgeComm(impl_->port, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

SerialTransport::~SerialTransport() = default;

std::vector<std::uint8_t> SerialTransport::transact(
    std::span<const std::uint8_t> request, std::chrono::milliseconds timeout) {
    DWORD written = 0;
    if (!WriteFile(impl_->port, request.data(), static_cast<DWORD>(request.size()),
                   &written, nullptr) || written != request.size()) {
        throw windowsError("serial write failed");
    }

    std::vector<std::uint8_t> response;
    std::array<std::uint8_t, 64> chunk{};
    std::size_t required = 0;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        COMSTAT status{};
        DWORD errors = 0;
        if (!ClearCommError(impl_->port, &errors, &status))
            throw windowsError("serial status failed");
        if (!status.cbInQue) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const auto wanted = static_cast<DWORD>(
            (std::min)(chunk.size(), static_cast<std::size_t>(status.cbInQue)));
        DWORD received = 0;
        if (!ReadFile(impl_->port, chunk.data(), wanted, &received, nullptr))
            throw windowsError("serial read failed");
        response.insert(response.end(), chunk.begin(), chunk.begin() + received);
        if (!required && response.size() >= 8) {
            const auto payload = static_cast<std::size_t>(response[6]) |
                                 (static_cast<std::size_t>(response[7]) << 8);
            if (payload > kBridgeMaxPayload)
                throw std::runtime_error("oversized serial response");
            required = 8 + payload + 2;
        }
        if (required && response.size() >= required) break;
    }
    if (!required || response.size() < required)
        throw std::runtime_error("serial response timed out");
    if (response.size() != required)
        throw std::runtime_error("extra bytes in serial response");
    return response;
}

} // namespace open1v
