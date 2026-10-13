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

add_library(vehicle_core STATIC src/vehicle_core.cpp src/Vehicle.cpp)
target_include_directories(vehicle_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(vehicle_bridge SHARED src/vehicle_bridge.cpp)
target_link_libraries(vehicle_bridge PRIVATE vehicle_core)
target_include_directories(vehicle_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(sim src/main.cpp)
target_link_libraries(sim PRIVATE vehicle_core raylib)

enable_testing()
add_executable(sim_tests tests/test_main.cpp)
target_link_libraries(sim_tests PRIVATE vehicle_core vehicle_bridge)
add_test(NAME VehicleSimUnitTests COMMAND sim_tests)

add_subdirectory(godot-cpp)

add_library(easydrive_extension SHARED
    src/gdextension/easy_drive_bridge.cpp
    src/gdextension/register_types.cpp
)
target_link_libraries(easydrive_extension PRIVATE godot-cpp)
target_include_directories(easydrive_extension PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gdextension
)
set_target_properties(easydrive_extension PROPERTIES
    PREFIX ""
    SUFFIX ".dll"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/godot_sim/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/godot_sim/bin"
)

add_custom_command(TARGET easydrive_extension POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "$<TARGET_FILE:easydrive_extension>"
        "${CMAKE_CURRENT_SOURCE_DIR}/godot_sim/bin/easydrive_extension.dll"
)
