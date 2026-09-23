#include "open1v/serial_transport.hpp"
#include "open1v/bridge_protocol.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#if defined(__APPLE__)
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOBSD.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#elif defined(__linux__)
#include <filesystem>
#include <fstream>
#endif

namespace open1v {
namespace {
bool supportedUsbId(unsigned vendor, unsigned product) {
    return (vendor == 0x2341 &&
            (product == 0x1002 || product == 0x0069 || product == 0x006a)) ||
           (vendor == 0x04a9 && product == 0x3040);
}

std::runtime_error posixError(const char* what) {
    return std::runtime_error(std::string(what) + ": " + std::strerror(errno));
}

#if defined(__APPLE__)
unsigned numberProperty(io_registry_entry_t entry, CFStringRef key) {
    CFTypeRef value = IORegistryEntrySearchCFProperty(
        entry, kIOServicePlane, key, kCFAllocatorDefault,
        kIORegistryIterateRecursively | kIORegistryIterateParents);
    unsigned result = 0;
    if (value && CFGetTypeID(value) == CFNumberGetTypeID())
        CFNumberGetValue(static_cast<CFNumberRef>(value), kCFNumberIntType, &result);
    if (value) CFRelease(value);
    return result;
}

std::string stringProperty(io_registry_entry_t entry, CFStringRef key) {
    CFTypeRef value = IORegistryEntryCreateCFProperty(
        entry, key, kCFAllocatorDefault, 0);
    char buffer[PATH_MAX]{};
    if (value && CFGetTypeID(value) == CFStringGetTypeID())
        CFStringGetCString(static_cast<CFStringRef>(value), buffer, sizeof(buffer),
                           kCFStringEncodingUTF8);
    if (value) CFRelease(value);
    return buffer;
}

std::string findBridgePort() {
    CFMutableDictionaryRef matching = IOServiceMatching(kIOSerialBSDServiceValue);
    if (!matching) return {};
    CFDictionarySetValue(matching, CFSTR(kIOSerialBSDTypeKey),
                         CFSTR(kIOSerialBSDAllTypes));
    io_iterator_t iterator = IO_OBJECT_NULL;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iterator) !=
        KERN_SUCCESS) return {};
    std::string match;
    for (io_object_t service; (service = IOIteratorNext(iterator));) {
        const auto vendor = numberProperty(service, CFSTR("idVendor"));
        const auto product = numberProperty(service, CFSTR("idProduct"));
        if (supportedUsbId(vendor, product)) {
            const auto path = stringProperty(service, CFSTR(kIOCalloutDeviceKey));
            if (!path.empty() && !match.empty() && match != path) {
                IOObjectRelease(service);
                IOObjectRelease(iterator);
                throw std::runtime_error(
                    "multiple compatible serial bridges found; use --port");
            }
            if (!path.empty()) match = path;
        }
        IOObjectRelease(service);
    }
    IOObjectRelease(iterator);
    return match;
}
#elif defined(__linux__)
unsigned readHex(const std::filesystem::path& path) {
    std::ifstream input(path);
    unsigned value = 0;
    input >> std::hex >> value;
    return input ? value : 0;
}

std::string findBridgePort() {
    namespace fs = std::filesystem;
    std::string match;
    std::error_code error;
    for (const auto& item : fs::directory_iterator("/sys/class/tty", error)) {
        fs::path device = fs::canonical(item.path() / "device", error);
        if (error) { error.clear(); continue; }
        for (auto parent = device; parent != parent.root_path(); parent = parent.parent_path()) {
            const unsigned vendor = readHex(parent / "idVendor");
            const unsigned product = readHex(parent / "idProduct");
            if (!supportedUsbId(vendor, product)) continue;
            const auto path = "/dev/" + item.path().filename().string();
            if (!match.empty() && match != path)
                throw std::runtime_error(
                    "multiple compatible serial bridges found; use --port");
            match = path;
            break;
        }
    }
    return match;
}
#endif

void waitFor(int fd, short events, std::chrono::steady_clock::time_point deadline,
             const char* timeoutMessage) {
    for (;;) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0) throw std::runtime_error(timeoutMessage);
        pollfd descriptor{fd, events, 0};
        const int result = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
        if (result > 0) {
            if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                throw std::runtime_error("serial device disconnected");
            if (descriptor.revents & events) return;
        } else if (result == 0) {
            throw std::runtime_error(timeoutMessage);
        } else if (errno != EINTR) {
            throw posixError("serial poll failed");
        }
    }
}
} // namespace

struct SerialTransport::Impl {
    int port{-1};
    ~Impl() { if (port >= 0) ::close(port); }
};

SerialTransport::SerialTransport(std::string path) : impl_(std::make_unique<Impl>()) {
    if (path.empty()) path = findBridgePort();
    if (path.empty()) throw std::runtime_error("compatible serial bridge not found");
    impl_->port = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (impl_->port < 0) throw posixError("cannot open Arduino serial port");

    termios settings{};
    if (tcgetattr(impl_->port, &settings) != 0) throw posixError("cannot read serial settings");
    cfmakeraw(&settings);
    cfsetispeed(&settings, B115200);
    cfsetospeed(&settings, B115200);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(CSTOPB | PARENB | CSIZE);
    settings.c_cflag |= CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(impl_->port, TCSANOW, &settings) != 0)
        throw posixError("cannot configure serial port");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    if (tcflush(impl_->port, TCIOFLUSH) != 0) throw posixError("cannot flush serial port");
}

SerialTransport::~SerialTransport() = default;

std::vector<std::uint8_t> SerialTransport::transact(
    std::span<const std::uint8_t> request, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::size_t offset = 0;
    while (offset < request.size()) {
        waitFor(impl_->port, POLLOUT, deadline, "serial write timed out");
        const auto count = ::write(impl_->port, request.data() + offset,
                                   request.size() - offset);
        if (count > 0) offset += static_cast<std::size_t>(count);
        else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            throw posixError("serial write failed");
    }

    std::vector<std::uint8_t> response;
    std::array<std::uint8_t, 64> chunk{};
    std::size_t required = 0;
    while (!required || response.size() < required) {
        waitFor(impl_->port, POLLIN, deadline, "serial response timed out");
        const auto count = ::read(impl_->port, chunk.data(), chunk.size());
        if (count > 0) response.insert(response.end(), chunk.begin(), chunk.begin() + count);
        else if (count == 0) throw std::runtime_error("serial device disconnected");
        else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            throw posixError("serial read failed");
        if (!required && response.size() >= 8) {
            const auto payload = static_cast<std::size_t>(response[6]) |
                                 (static_cast<std::size_t>(response[7]) << 8);
            if (payload > kBridgeMaxPayload)
                throw std::runtime_error("oversized serial response");
            required = 8 + payload + 2;
        }
        if (required && response.size() > required)
            throw std::runtime_error("extra bytes in serial response");
    }
    return response;
}
} // namespace open1v
