#include "open1v/shared_transport.hpp"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace open1v {
namespace {
std::uint64_t stableKeyHash(const std::string& value) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto byte : value) {
        hash ^= static_cast<unsigned char>(byte);
        hash *= 1099511628211ull;
    }
    return hash;
}
}
struct SharedTransport::Impl {
    std::unique_ptr<ITransport> transport;
    HANDLE exclusive{nullptr};
    ~Impl() {
        if (exclusive) { ReleaseMutex(exclusive); CloseHandle(exclusive); }
    }
};
SharedTransport::SharedTransport(std::string deviceKey, Factory factory)
    : impl_(std::make_unique<Impl>()) {
    const auto name = L"Local\\open1v-exclusive-" +
        std::to_wstring(static_cast<unsigned long long>(stableKeyHash(deviceKey)));
    impl_->exclusive = CreateMutexW(nullptr, FALSE, name.c_str());
    const auto wait = impl_->exclusive
        ? WaitForSingleObject(impl_->exclusive, 0) : WAIT_FAILED;
    if (wait != WAIT_OBJECT_0 && wait != WAIT_ABANDONED) {
        if (impl_->exclusive) { CloseHandle(impl_->exclusive); impl_->exclusive = nullptr; }
        throw std::runtime_error("camera connection is already owned by another open1V program");
    }
    impl_->transport = factory();
}
SharedTransport::~SharedTransport() = default;
std::vector<std::uint8_t> SharedTransport::transact(
    std::span<const std::uint8_t> request, std::chrono::milliseconds timeout) {
    return impl_->transport->transact(request, timeout);
}
void SharedTransport::beginOperation() { impl_->transport->beginOperation(); }
void SharedTransport::endOperation() noexcept { impl_->transport->endOperation(); }
bool SharedTransport::isShared() const noexcept { return false; }
bool SharedTransport::acquireCameraSession() {
    return impl_->transport->acquireCameraSession();
}
bool SharedTransport::releaseCameraSession() {
    return impl_->transport->releaseCameraSession();
}
} // namespace open1v

#else

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace open1v {
namespace {
constexpr std::uint32_t kMagic = 0x31565353; // "SSV1"
constexpr std::uint32_t kMaxWireBytes = 1024 * 1024;
enum class Kind : std::uint32_t {
    transact = 1, beginOperation, endOperation,
    acquireSession, releaseSession, detach
};
struct Header {
    std::uint32_t magic{kMagic};
    std::uint32_t kind{};
    std::uint32_t value{};
    std::uint32_t size{};
};
struct Reply {
    std::uint32_t magic{kMagic};
    std::uint32_t status{};
    std::uint32_t value{};
    std::uint32_t size{};
};

std::runtime_error systemError(const char* text) {
    return std::runtime_error(std::string(text) + ": " + std::strerror(errno));
}

void writeAll(int fd, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    while (size) {
#ifdef MSG_NOSIGNAL
        const auto count = ::send(fd, bytes, size, MSG_NOSIGNAL);
#else
        const auto count = ::send(fd, bytes, size, 0);
#endif
        if (count > 0) { bytes += count; size -= static_cast<std::size_t>(count); }
        else if (count < 0 && errno == EINTR) continue;
        else throw systemError("shared-session write failed");
    }
}

bool readAllOrEof(int fd, void* data, std::size_t size) {
    auto* bytes = static_cast<std::uint8_t*>(data);
    std::size_t done = 0;
    while (done < size) {
        const auto count = ::recv(fd, bytes + done, size - done, 0);
        if (count > 0) done += static_cast<std::size_t>(count);
        else if (count == 0) {
            if (done == 0) return false;
            throw std::runtime_error("truncated shared-session message");
        }
        else if (errno != EINTR) throw systemError("shared-session read failed");
    }
    return true;
}

void readAll(int fd, void* data, std::size_t size) {
    if (!readAllOrEof(fd, data, size))
        throw std::runtime_error("shared-session peer disconnected");
}

std::string endpointFor(const std::string& key, const char* suffix) {
    std::uint64_t value = 1469598103934665603ull;
    for (const auto byte : key) {
        value ^= static_cast<unsigned char>(byte);
        value *= 1099511628211ull;
    }
    return "/tmp/open1v-" + std::to_string(static_cast<unsigned long>(::getuid())) +
           "-" + std::to_string(static_cast<unsigned long long>(value)) + suffix;
}

int connectSocket(const std::string& path) {
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) throw systemError("cannot create shared-session socket");
#ifdef SO_NOSIGPIPE
    int enabled = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path.size() >= sizeof(address.sun_path)) {
        ::close(fd); throw std::runtime_error("shared-session path is too long");
    }
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0)
        return fd;
    const int error = errno;
    ::close(fd);
    if (error == ENOENT || error == ECONNREFUSED) return -1;
    errno = error;
    throw systemError("cannot connect to shared camera session");
}
} // namespace

struct SharedTransport::Impl {
    explicit Impl(std::string key) : socketPath(endpointFor(key, ".sock")),
        lockPath(endpointFor(key, ".lock")) {}

    std::string socketPath;
    std::string lockPath;
    int client{-1};
    int listener{-1};
    int electionLock{-1};
    bool owner{};
    bool operationHeld{};
    std::unique_ptr<ITransport> physical;
    std::thread acceptThread;
    std::mutex workersMutex;
    std::vector<std::thread> workers;
    std::mutex stateMutex;
    std::condition_variable stateChanged;
    std::uint64_t nextClientId{1};
    std::uint64_t nextTicket{};
    std::uint64_t servingTicket{};
    std::uint64_t operationOwner{};
    std::unordered_set<std::uint64_t> clients;
    std::unordered_set<std::uint64_t> sessionOwners;
    bool stopping{};

    ~Impl() {
        try {
            if (client >= 0) {
                Header request{kMagic, static_cast<std::uint32_t>(Kind::detach), 0, 0};
                writeAll(client, &request, sizeof(request));
                Reply reply{}; readAll(client, &reply, sizeof(reply));
                ::close(client); client = -1;
            }
        } catch (...) { if (client >= 0) ::close(client); }
        if (!owner) return;
        {
            std::unique_lock lock(stateMutex);
            stateChanged.wait(lock, [&] { return clients.empty(); });
            stopping = true;
        }
        if (listener >= 0) { ::shutdown(listener, SHUT_RDWR); ::close(listener); listener = -1; }
        if (acceptThread.joinable()) acceptThread.join();
        std::vector<std::thread> joined;
        { std::lock_guard lock(workersMutex); joined.swap(workers); }
        for (auto& worker : joined) if (worker.joinable()) worker.join();
        ::unlink(socketPath.c_str());
        if (electionLock >= 0) { ::flock(electionLock, LOCK_UN); ::close(electionLock); }
    }

    void reply(int fd, std::uint32_t status, std::uint32_t value,
               std::span<const std::uint8_t> body = {}) {
        Reply response{kMagic, status, value, static_cast<std::uint32_t>(body.size())};
        writeAll(fd, &response, sizeof(response));
        if (!body.empty()) writeAll(fd, body.data(), body.size());
    }

    void fail(int fd, const std::exception& error) {
        const std::string text = error.what();
        reply(fd, 1, 0, std::span<const std::uint8_t>(
            reinterpret_cast<const std::uint8_t*>(text.data()), text.size()));
    }

    void releaseClient(std::uint64_t id) {
        std::lock_guard lock(stateMutex);
        sessionOwners.erase(id);
        if (operationOwner == id) {
            operationOwner = 0;
            ++servingTicket;
        }
        clients.erase(id);
        stateChanged.notify_all();
    }

    void serve(int fd, std::uint64_t id) {
        try {
            for (;;) {
                Header request{};
                if (!readAllOrEof(fd, &request, sizeof(request))) break;
                if (request.magic != kMagic || request.size > kMaxWireBytes)
                    throw std::runtime_error("invalid shared-session request");
                std::vector<std::uint8_t> body(request.size);
                if (!body.empty()) readAll(fd, body.data(), body.size());
                const auto kind = static_cast<Kind>(request.kind);
                if (kind == Kind::beginOperation) {
                    std::unique_lock lock(stateMutex);
                    const auto ticket = nextTicket++;
                    stateChanged.wait(lock, [&] {
                        return ticket == servingTicket && operationOwner == 0;
                    });
                    operationOwner = id;
                    lock.unlock();
                    reply(fd, 0, 0);
                } else if (kind == Kind::endOperation) {
                    {
                        std::lock_guard lock(stateMutex);
                        if (operationOwner != id)
                            throw std::runtime_error("shared operation owner mismatch");
                        operationOwner = 0;
                        ++servingTicket;
                        stateChanged.notify_all();
                    }
                    reply(fd, 0, 0);
                } else if (kind == Kind::transact) {
                    {
                        std::lock_guard lock(stateMutex);
                        if (operationOwner != id)
                            throw std::runtime_error("bridge request outside shared operation");
                    }
                    const auto response = physical->transact(
                        body, std::chrono::milliseconds(request.value));
                    reply(fd, 0, 0, response);
                } else if (kind == Kind::acquireSession) {
                    bool inherited;
                    {
                        std::lock_guard lock(stateMutex);
                        if (operationOwner != id)
                            throw std::runtime_error("session acquire outside shared operation");
                        inherited = !sessionOwners.empty();
                        sessionOwners.insert(id);
                    }
                    reply(fd, 0, inherited ? 1u : 0u);
                } else if (kind == Kind::releaseSession) {
                    bool last;
                    {
                        std::lock_guard lock(stateMutex);
                        if (operationOwner != id)
                            throw std::runtime_error("session release outside shared operation");
                        sessionOwners.erase(id);
                        last = sessionOwners.empty();
                    }
                    reply(fd, 0, last ? 1u : 0u);
                } else if (kind == Kind::detach) {
                    reply(fd, 0, 0);
                    break;
                } else throw std::runtime_error("unknown shared-session request");
            }
        } catch (const std::exception& error) {
            try { fail(fd, error); } catch (...) {}
        }
        ::close(fd);
        releaseClient(id);
    }

    void acceptLoop() {
        while (!stopping) {
            const int fd = ::accept(listener, nullptr, nullptr);
            if (fd < 0) {
                if (errno == EINTR) continue;
                break;
            }
#ifdef SO_NOSIGPIPE
            int enabled = 1;
            (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif
            std::uint64_t id;
            {
                std::lock_guard lock(stateMutex);
                id = nextClientId++;
                clients.insert(id);
            }
            std::lock_guard lock(workersMutex);
            workers.emplace_back([this, fd, id] { serve(fd, id); });
        }
    }

    void start(Factory& factory) {
        electionLock = ::open(lockPath.c_str(), O_CREAT | O_RDWR, 0600);
        if (electionLock < 0) throw systemError("cannot open shared-session lock");
        if (::flock(electionLock, LOCK_EX) != 0)
            throw systemError("cannot lock shared-session election");
        client = connectSocket(socketPath);
        if (client >= 0) { ::flock(electionLock, LOCK_UN); ::close(electionLock); electionLock = -1; return; }

        physical = factory();
        listener = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (listener < 0) throw systemError("cannot create shared-session listener");
        ::unlink(socketPath.c_str());
        sockaddr_un address{}; address.sun_family = AF_UNIX;
        std::memcpy(address.sun_path, socketPath.c_str(), socketPath.size() + 1);
        if (::bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
            throw systemError("cannot bind shared-session listener");
        ::chmod(socketPath.c_str(), 0600);
        if (::listen(listener, 8) != 0) throw systemError("cannot listen for shared sessions");
        owner = true;
        acceptThread = std::thread([this] { acceptLoop(); });
        for (int attempt = 0; attempt < 50 && client < 0; ++attempt) {
            client = connectSocket(socketPath);
            if (client < 0) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (client < 0) throw std::runtime_error("cannot connect to new shared-session server");
        ::flock(electionLock, LOCK_UN);
        ::close(electionLock);
        electionLock = -1;
    }

    std::pair<std::uint32_t, std::vector<std::uint8_t>> call(
        Kind kind, std::uint32_t value = 0, std::span<const std::uint8_t> body = {}) {
        Header request{kMagic, static_cast<std::uint32_t>(kind), value,
                       static_cast<std::uint32_t>(body.size())};
        writeAll(client, &request, sizeof(request));
        if (!body.empty()) writeAll(client, body.data(), body.size());
        Reply response{}; readAll(client, &response, sizeof(response));
        if (response.magic != kMagic || response.size > kMaxWireBytes)
            throw std::runtime_error("invalid shared-session response");
        std::vector<std::uint8_t> result(response.size);
        if (!result.empty()) readAll(client, result.data(), result.size());
        if (response.status != 0)
            throw std::runtime_error(std::string(result.begin(), result.end()));
        return {response.value, std::move(result)};
    }
};

SharedTransport::SharedTransport(std::string deviceKey, Factory factory)
    : impl_(std::make_unique<Impl>(std::move(deviceKey))) {
    impl_->client = connectSocket(impl_->socketPath);
    if (impl_->client < 0) impl_->start(factory);
}
SharedTransport::~SharedTransport() = default;

std::vector<std::uint8_t> SharedTransport::transact(
    std::span<const std::uint8_t> request, std::chrono::milliseconds timeout) {
    if (!impl_->operationHeld)
        throw std::runtime_error("shared bridge transaction has no operation lease");
    return impl_->call(Kind::transact, static_cast<std::uint32_t>(timeout.count()), request).second;
}
void SharedTransport::beginOperation() {
    if (impl_->operationHeld) throw std::runtime_error("shared operation is already active");
    impl_->call(Kind::beginOperation); impl_->operationHeld = true;
}
void SharedTransport::endOperation() noexcept {
    if (!impl_->operationHeld) return;
    try { impl_->call(Kind::endOperation); } catch (...) {}
    impl_->operationHeld = false;
}
bool SharedTransport::isShared() const noexcept { return true; }
bool SharedTransport::acquireCameraSession() {
    return impl_->call(Kind::acquireSession).first != 0;
}
bool SharedTransport::releaseCameraSession() {
    return impl_->call(Kind::releaseSession).first != 0;
}

} // namespace open1v
#endif
