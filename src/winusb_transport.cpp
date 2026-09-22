#include "open1v/winusb_transport.hpp"
#include "open1v/bridge_protocol.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <setupapi.h>
#include <winusb.h>

#include <array>
#include <cwctype>
#include <stdexcept>
#include <string>

namespace open1v {
namespace {
constexpr GUID kDeviceInterfaceGuid =
    {0x975f44d9, 0x0d08, 0x43fd, {0x8b, 0x3e, 0x12, 0x7c, 0xa8, 0xaf, 0xff, 0x9d}};
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
    UCHAR bulkIn{};
    UCHAR bulkOut{};

    ~Impl() {
        if (usb) WinUsb_Free(usb);
        if (device != INVALID_HANDLE_VALUE) CloseHandle(device);
    }
};

WinUsbTransport::WinUsbTransport() : impl_(std::make_unique<Impl>()) {
    auto path = findDevicePath(kDeviceInterfaceGuid);
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

    USB_INTERFACE_DESCRIPTOR descriptor{};
    if (!WinUsb_QueryInterfaceSettings(impl_->usb, 0, &descriptor))
        throw windowsError("cannot query WinUSB interface");
    for (UCHAR index = 0; index < descriptor.bNumEndpoints; ++index) {
        WINUSB_PIPE_INFORMATION pipe{};
        if (!WinUsb_QueryPipe(impl_->usb, 0, index, &pipe)) continue;
        if (pipe.PipeType != UsbdPipeTypeBulk) continue;
        if (USB_ENDPOINT_DIRECTION_IN(pipe.PipeId)) impl_->bulkIn = pipe.PipeId;
        else impl_->bulkOut = pipe.PipeId;
    }
    if (!impl_->bulkIn || !impl_->bulkOut) throw std::runtime_error("WinUSB bulk endpoints not found");
}

WinUsbTransport::~WinUsbTransport() = default;

std::vector<std::uint8_t> WinUsbTransport::transact(
    std::span<const std::uint8_t> request, std::chrono::milliseconds timeout) {
    ULONG timeoutValue = static_cast<ULONG>(timeout.count());
    WinUsb_SetPipePolicy(impl_->usb, impl_->bulkIn, PIPE_TRANSFER_TIMEOUT,
                         sizeof(timeoutValue), &timeoutValue);
    WinUsb_SetPipePolicy(impl_->usb, impl_->bulkOut, PIPE_TRANSFER_TIMEOUT,
                         sizeof(timeoutValue), &timeoutValue);
    ULONG written = 0;
    if (!WinUsb_WritePipe(impl_->usb, impl_->bulkOut,
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
        if (!WinUsb_ReadPipe(impl_->usb, impl_->bulkIn, chunk.data(),
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
