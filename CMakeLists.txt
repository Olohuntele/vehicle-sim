cmake_minimum_required(VERSION 3.12)
project(VehicleSim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(sim src/main.cpp src/Vehicle.cpp)
target_include_directories(sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim PRIVATE raylib)

enable_testing()
add_executable(sim_tests tests/test_main.cpp src/Vehicle.cpp)
target_include_directories(sim_tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME VehicleSimUnitTests COMMAND sim_tests)
