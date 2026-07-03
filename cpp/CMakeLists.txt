cmake_minimum_required(VERSION 3.16)
project(adsb_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(adsb_core
    src/kalman_filter.cpp
    src/track_manager.cpp
    src/association.cpp
    src/csv_replay.cpp
)
target_include_directories(adsb_core PUBLIC include)

if(MSVC)
    target_compile_options(adsb_core PRIVATE /W4)
else()
    target_compile_options(adsb_core PRIVATE -Wall -Wextra)
endif()

add_executable(adsb_replay src/main.cpp)
target_link_libraries(adsb_replay PRIVATE adsb_core)

enable_testing()
add_executable(adsb_tests tests/test_main.cpp)
target_link_libraries(adsb_tests PRIVATE adsb_core)
add_test(NAME unit_tests COMMAND adsb_tests)

# MinGW: link the runtime statically so the executables run without the
# compiler's bin directory on PATH.
if(MINGW)
    target_link_options(adsb_replay PRIVATE -static)
    target_link_options(adsb_tests PRIVATE -static)
endif()
