cmake_minimum_required(VERSION 3.20)
project(open1V VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OPEN1V_CORE_SOURCES
    src/bridge_client.cpp
    src/bridge_protocol.cpp
    src/camera_protocol.cpp)

if(WIN32)
    list(APPEND OPEN1V_CORE_SOURCES src/serial_transport.cpp src/winusb_transport.cpp)
else()
    list(APPEND OPEN1V_CORE_SOURCES src/serial_transport_posix.cpp src/winusb_transport_stub.cpp)
endif()

add_library(open1v-core STATIC ${OPEN1V_CORE_SOURCES})
target_include_directories(open1v-core PUBLIC include)
if(WIN32)
    target_link_libraries(open1v-core PRIVATE setupapi winusb)
elseif(APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(open1v-core PRIVATE Threads::Threads)
    target_link_libraries(open1v-core PRIVATE "-framework IOKit" "-framework CoreFoundation")
else()
    find_package(Threads REQUIRED)
    target_link_libraries(open1v-core PRIVATE Threads::Threads)
endif()

add_executable(open1V src/main.cpp)
target_link_libraries(open1V PRIVATE open1v-core)

add_executable(open1V-debug src/main.cpp src/self_test.cpp)
target_compile_definitions(open1V-debug PRIVATE OPEN1V_DEBUG_CLI)
target_link_libraries(open1V-debug PRIVATE open1v-core)

include(CTest)
if(BUILD_TESTING)
    add_test(NAME open1v-self-test COMMAND open1V-debug self-test)
    if(UNIX)
        add_executable(open1v-posix-serial-test tests/posix_serial_test.cpp)
        target_link_libraries(open1v-posix-serial-test PRIVATE open1v-core)
        if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
            target_link_libraries(open1v-posix-serial-test PRIVATE util)
        endif()
        add_test(NAME open1v-posix-serial-test COMMAND open1v-posix-serial-test)
    endif()
endif()
