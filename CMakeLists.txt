cmake_minimum_required(VERSION 3.20)

project(RadarTracker
    VERSION 1.0.0
    DESCRIPTION "Automotive Radar Multi-Target Tracker (MTT) with EKF & Track Lifecycle"
    LANGUAGES CXX
)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Configure output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Compiler warnings and optimization options
add_library(project_warnings INTERFACE)
if(MSVC)
    target_compile_options(project_warnings INTERFACE
        /W4
        /permissive-
        /EHsc
        $<$<CONFIG:Release>:/O2 /Ob2 /Oi>
    )
else()
    target_compile_options(project_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        $<$<CONFIG:Release>:-O3>
    )
endif()

# Core tracker library
add_library(radar_tracker_core STATIC
    src/MultiTargetTracker.cpp
)

target_include_directories(radar_tracker_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(radar_tracker_core
    PUBLIC
        project_warnings
)

# Main CLI demo
add_executable(radar_tracker
    src/main.cpp
)

target_link_libraries(radar_tracker
    PRIVATE
        radar_tracker_core
)

# Test suite executable
enable_testing()

add_executable(test_tracker
    tests/test_tracker.cpp
)

target_link_libraries(test_tracker
    PRIVATE
        radar_tracker_core
)

add_test(
    NAME RadarTrackerAcceptanceTests
    COMMAND test_tracker
)

# ==============================================================================
# Dear ImGui Integration (Win32 + DirectX 11 native backend)
# ==============================================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/imgui.cpp")
    add_library(imgui STATIC
        third_party/imgui/imgui.cpp
        third_party/imgui/imgui_draw.cpp
        third_party/imgui/imgui_tables.cpp
        third_party/imgui/imgui_widgets.cpp
        third_party/imgui/backends/imgui_impl_win32.cpp
        third_party/imgui/backends/imgui_impl_dx11.cpp
    )

    target_include_directories(imgui
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends
    )

    if(WIN32)
        target_link_libraries(imgui
            PUBLIC
                d3d11
                dxgi
                d3dcompiler
                dwmapi
                gdi32
        )
    endif()

    # GUI Visualizer Executable
    add_executable(radar_tracker_gui
        src/gui/main_gui.cpp
    )

    target_link_libraries(radar_tracker_gui
        PRIVATE
            radar_tracker_core
            imgui
    )

    if(NOT MSVC)
        target_link_options(radar_tracker_gui PRIVATE -mwindows)
    endif()
endif()
