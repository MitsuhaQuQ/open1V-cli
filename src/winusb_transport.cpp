#include "open1v/winusb_transport.hpp"
#include "open1v/bridge_protocol.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <setupapi.h>
#include <winusb.h>

#include <array>
#include <chrono>
#include <cwctype>
#include <stdexcept>
#include <string>
#include <thread>

namespace open1v {
namespace {
constexpr GUID kDeviceInterfaceGuid =
    {0x975f44d9, 0x0d08, 0x43fd, {0x8b, 0x3e, 0x12, 0x7c, 0xa8, 0xaf, 0xff, 0x9d}};
constexpr GUID kCanonEsE1ZadigInterfaceGuid =
    {0x677481f5, 0x0ce1, 0x4dc0, {0xbe, 0x6b, 0x65, 0x01, 0x2e, 0x89, 0xa3, 0xde}};
constexpr GUID kUsbDeviceInterfaceGuid =
    {0xa5dcbf10, 0x6530, 0x11d2, {0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed}};

std::runtime_error windowsError(const char* what) {
    return std::runtime_error(std::string(what) + " (Windows error " +
                              std::to_string(GetLastError()) + ")");
}

std::wstring findDevicePath(const GUID& interfaceGuid,
                            const wchar_t* requiredText = nullptr) {
    HDEVINFO info = SetupDiGetClassDevsW(&interfaceGuid, nullptr, nullptr,
                                         DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (info == INVALID_HANDLE_VALUE) return {};
    std::wstring match;
    for (DWORD index = 0;; ++index) {
        SP_DEVICE_INTERFACE_DATA data{sizeof(data)};
        if (!SetupDiEnumDeviceInterfaces(info, nullptr, &interfaceGuid, index, &data)) break;
        DWORD required = 0;
        SetupDiGetDeviceInterfaceDetailW(info, &data, nullptr, 0, &required, nullptr);
        std::vector<std::byte> storage(required);
        auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(storage.data());
        detail->cbSize = sizeof(*detail);
        if (!SetupDiGetDeviceInterfaceDetailW(info, &data, detail, required, nullptr, nullptr))
            continue;
        std::wstring path = detail->DevicePath;
        std::wstring folded = path;
        for (auto& character : folded) character = static_cast<wchar_t>(std::towlower(character));
        if (!requiredText || folded.find(requiredText) != std::wstring::npos) {
            if (!match.empty()) {
                SetupDiDestroyDeviceInfoList(info);
                throw std::runtime_error("multiple matching open1V WinUSB bridges found");
            }
            match = std::move(path);
        }
    }
    SetupDiDestroyDeviceInfoList(info);
    return match;
}
} // namespace

struct WinUsbTransport::Impl {
    HANDLE device{INVALID_HANDLE_VALUE};
    WINUSB_INTERFACE_HANDLE usb{nullptr};
    WINUSB_INTERFACE_HANDLE associatedUsb{nullptr};
    WINUSB_INTERFACE_HANDLE ioUsb{nullptr};
    UCHAR bulkIn{};
    UCHAR bulkOut{};

    ~Impl() {
        if (associatedUsb) WinUsb_Free(associatedUsb);
        if (usb) WinUsb_Free(usb);
        if (device != INVALID_HANDLE_VALUE) CloseHandle(device);
    }
};

WinUsbTransport::WinUsbTransport() : impl_(std::make_unique<Impl>()) {
    auto path = findDevicePath(kDeviceInterfaceGuid);
    if (path.empty()) {
        path = findDevicePath(kCanonEsE1ZadigInterfaceGuid,
                              L"vid_04a9&pid_3040");
    }
    if (path.empty()) {
        path = findDevicePath(kUsbDeviceInterfaceGuid, L"vid_303a&pid_1001");
    }
    if (path.empty()) throw std::runtime_error("open1V WinUSB bridge not found");
    impl_->device = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                                nullptr);
    if (impl_->device == INVALID_HANDLE_VALUE) throw windowsError("cannot open WinUSB bridge");
    if (!WinUsb_Initialize(impl_->device, &impl_->usb)) throw windowsError("WinUsb_Initialize failed");

    const auto inspectInterface = [&](WINUSB_INTERFACE_HANDLE handle) {
        USB_INTERFACE_DESCRIPTOR descriptor{};
        if (!WinUsb_QueryInterfaceSettings(handle, 0, &descriptor)) return;
        UCHAR bulkIn = 0;
        UCHAR bulkOut = 0;
        for (UCHAR index = 0; index < descriptor.bNumEndpoints; ++index) {
            WINUSB_PIPE_INFORMATION pipe{};
            if (!WinUsb_QueryPipe(handle, 0, index, &pipe)) continue;
            if (pipe.PipeType != UsbdPipeTypeBulk) continue;
            if (USB_ENDPOINT_DIRECTION_IN(pipe.PipeId)) bulkIn = pipe.PipeId;
            else bulkOut = pipe.PipeId;
        }
        if (bulkIn && bulkOut) {
            impl_->ioUsb = handle;
            impl_->bulkIn = bulkIn;
            impl_->bulkOut = bulkOut;
        }
    };

    inspectInterface(impl_->usb);
    if (!impl_->ioUsb &&
        WinUsb_GetAssociatedInterface(impl_->usb, 0, &impl_->associatedUsb)) {
        inspectInterface(impl_->associatedUsb);
    }
    if (!impl_->bulkIn || !impl_->bulkOut) throw std::runtime_error("WinUSB bulk endpoints not found");

    if (impl_->ioUsb == impl_->associatedUsb) {
        // Zadig binds WinUSB to the complete CDC device instead of loading
        // usbser.sys. Reproduce the two CDC ACM requests normally issued by
        // the serial driver so Arduino Serial accepts data on interface 1.
        std::array<UCHAR, 7> lineCoding{
            0x00, 0xc2, 0x01, 0x00, // 115200, little endian
            0x00,                   // one stop bit
            0x00,                   // no parity
            0x08};                  // eight data bits
        WINUSB_SETUP_PACKET setLineCoding{0x21, 0x20, 0, 0,
                                           static_cast<USHORT>(lineCoding.size())};
        ULONG transferred = 0;
        const BOOL lineCodingOk = WinUsb_ControlTransfer(
            impl_->usb, setLineCoding, lineCoding.data(),
            static_cast<ULONG>(lineCoding.size()), &transferred, nullptr);
        if (!lineCodingOk) {
            if (GetLastError() != ERROR_SEM_TIMEOUT)
                throw windowsError("cannot set CDC line coding");
        } else if (transferred != lineCoding.size()) {
            throw std::runtime_error("short CDC line-coding transfer");
        }
        WINUSB_SETUP_PACKET setControlLines{0x21, 0x22, 0x0003, 0, 0};
        transferred = 0;
        if (!WinUsb_ControlTransfer(impl_->usb, setControlLines, nullptr, 0,
                                    &transferred, nullptr)) {
            if (GetLastError() != ERROR_SEM_TIMEOUT)
                throw windowsError("cannot enable CDC control lines");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Match SerialTransport's open behavior. A previous process can leave a
    // complete O1 response queued after a CDC control timeout or early exit.
    // It must not be mistaken for the first response of this session.
    if (!WinUsb_FlushPipe(impl_->ioUsb, impl_->bulkIn))
        throw windowsError("cannot flush WinUSB input");
    ULONG drainTimeout = 10;
    WinUsb_SetPipePolicy(impl_->ioUsb, impl_->bulkIn, PIPE_TRANSFER_TIMEOUT,
                         sizeof(drainTimeout), &drainTimeout);
    std::array<UCHAR, 64> stale{};
    for (;;) {
        ULONG received = 0;
        if (!WinUsb_ReadPipe(impl_->ioUsb, impl_->bulkIn, stale.data(),
                             static_cast<ULONG>(stale.size()), &received,
                             nullptr)) {
            if (GetLastError() == ERROR_SEM_TIMEOUT) break;
            throw windowsError("cannot drain WinUSB input");
        }
        if (!received) break;
    }
}

WinUsbTransport::~WinUsbTransport() = default;

std::vector<std::uint8_t> WinUsbTransport::transact(
    std::span<const std::uint8_t> request, std::chrono::milliseconds timeout) {
    ULONG timeoutValue = static_cast<ULONG>(timeout.count());
    WinUsb_SetPipePolicy(impl_->ioUsb, impl_->bulkIn, PIPE_TRANSFER_TIMEOUT,
                         sizeof(timeoutValue), &timeoutValue);
    WinUsb_SetPipePolicy(impl_->ioUsb, impl_->bulkOut, PIPE_TRANSFER_TIMEOUT,
                         sizeof(timeoutValue), &timeoutValue);
    ULONG written = 0;
    if (!WinUsb_WritePipe(impl_->ioUsb, impl_->bulkOut,
                          const_cast<PUCHAR>(request.data()),
                          static_cast<ULONG>(request.size()), &written, nullptr) ||
        written != request.size()) {
        throw windowsError("WinUSB write failed");
    }

    std::vector<std::uint8_t> response;
    std::array<std::uint8_t, 64> chunk{};
    std::size_t required = 0;
    do {
        ULONG received = 0;
        if (!WinUsb_ReadPipe(impl_->ioUsb, impl_->bulkIn, chunk.data(),
                             static_cast<ULONG>(chunk.size()), &received, nullptr))
            throw windowsError("WinUSB read failed");
        response.insert(response.end(), chunk.begin(), chunk.begin() + received);
        if (!required && response.size() >= 8) {
            const auto payload = static_cast<std::size_t>(response[6]) |
                                 (static_cast<std::size_t>(response[7]) << 8);
            if (payload > kBridgeMaxPayload) throw std::runtime_error("oversized WinUSB response");
            required = 8 + payload + 2;
        }
    } while (!required || response.size() < required);
    if (response.size() != required) throw std::runtime_error("extra bytes in WinUSB response");
    return response;
}

} // namespace open1v
