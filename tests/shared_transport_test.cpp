#include "open1v/shared_transport.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {
class EchoTransport final : public open1v::ITransport {
public:
    std::vector<std::uint8_t> transact(
        std::span<const std::uint8_t> request,
        std::chrono::milliseconds) override {
        if (!request.empty() && request[0] == 1)
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
        return {request.begin(), request.end()};
    }
};

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
}

int main() {
    try {
        const auto key = "shared-transport-test-" + std::to_string(::getpid());
        std::atomic<unsigned> factoryCalls{};
        auto factory = [&]() -> std::unique_ptr<open1v::ITransport> {
            ++factoryCalls;
            return std::make_unique<EchoTransport>();
        };
        open1v::SharedTransport first(key, factory);
        open1v::SharedTransport second(key, factory);
        require(factoryCalls == 1, "physical transport was opened more than once");

        first.beginOperation();
        require(!first.acquireCameraSession(), "first client inherited a nonexistent session");
        first.endOperation();
        second.beginOperation();
        require(second.acquireCameraSession(), "second client did not inherit the active session");
        second.endOperation();
        first.beginOperation();
        require(!first.releaseCameraSession(), "non-final client was allowed to close the session");
        first.endOperation();
        second.beginOperation();
        require(second.releaseCameraSession(), "final client was prevented from closing the session");
        second.endOperation();

        std::atomic<bool> firstHasLease{};
        std::atomic<bool> secondHasLease{};
        std::thread a([&] {
            first.beginOperation();
            firstHasLease = true;
            const std::uint8_t byte = 1;
            const auto reply = first.transact({&byte, 1}, std::chrono::seconds(1));
            require(reply == std::vector<std::uint8_t>{1}, "first reply mismatch");
            first.endOperation();
        });
        while (!firstHasLease) std::this_thread::yield();
        std::thread b([&] {
            second.beginOperation();
            secondHasLease = true;
            const std::uint8_t byte = 2;
            const auto reply = second.transact({&byte, 1}, std::chrono::seconds(1));
            require(reply == std::vector<std::uint8_t>{2}, "second reply mismatch");
            second.endOperation();
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        require(!secondHasLease, "second command ran before the first reply completed");
        a.join(); b.join();
        require(secondHasLease, "queued second command never ran");
        std::cout << "shared transport test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
