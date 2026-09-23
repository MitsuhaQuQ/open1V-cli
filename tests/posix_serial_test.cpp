#include "open1v/bridge_protocol.hpp"
#include "open1v/serial_transport.hpp"

#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <util.h>

int main() {
    int master = -1;
    int slave = -1;
    char slaveName[256]{};
    if (openpty(&master, &slave, slaveName, nullptr, nullptr) != 0) {
        std::cerr << "openpty failed: " << std::strerror(errno) << '\n';
        return 1;
    }
    close(slave);

    try {
        const std::string path = slaveName;
        open1v::SerialTransport transport(path);
        const std::array<std::uint8_t, 3> request{0x11, 0x22, 0x33};
        const auto expected = open1v::encodeFrame({0x02, 7, {0xaa, 0xbb, 0xcc}});

        std::thread bridge([&] {
            std::array<std::uint8_t, 16> received{};
            std::size_t total = 0;
            while (total < request.size()) {
                const auto count = read(master, received.data() + total,
                                        received.size() - total);
                if (count > 0) total += static_cast<std::size_t>(count);
            }
            write(master, expected.data(), 5);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            write(master, expected.data() + 5, expected.size() - 5);
        });

        const auto actual = transport.transact(request, std::chrono::seconds(1));
        bridge.join();
        close(master);
        if (actual != expected) throw std::runtime_error("fragmented response changed in transit");
    } catch (const std::exception& error) {
        if (master >= 0) close(master);
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
