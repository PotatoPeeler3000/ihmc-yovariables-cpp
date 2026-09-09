# lz4 (vendored under third_party/lz4, see third_party/lz4/LICENSE) is built as its own small C
# library so it gets its own (plain C) compile settings and never pollutes the C++ target's flags.
enable_language(C)
add_library(lz4_vendored STATIC ${CMAKE_SOURCE_DIR}/third_party/lz4/lz4.c)
target_include_directories(lz4_vendored PUBLIC ${CMAKE_SOURCE_DIR}/third_party/lz4)
set_target_properties(lz4_vendored PROPERTIES C_STANDARD 11 POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
FetchContent_Declare(
   nlohmann_json
   GIT_REPOSITORY https://github.com/nlohmann/json.git
   GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# Standalone (non-Boost) Asio - header-only, but ships no CMakeLists.txt of its own, so it's
# wrapped in a small INTERFACE target here.
FetchContent_Declare(
   asio
   GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
   GIT_TAG asio-1-38-2
)
FetchContent_MakeAvailable(asio)

add_library(asio_standalone INTERFACE)
target_include_directories(asio_standalone INTERFACE ${asio_SOURCE_DIR}/asio/include)
target_compile_definitions(asio_standalone INTERFACE ASIO_STANDALONE ASIO_NO_DEPRECATED)

find_package(Threads REQUIRED)

# Sources are added explicitly (no globbing), same convention as src/CMakeLists.txt, so CMake
# always re-runs when a file is added rather than silently missing it.
set(IHMC_ROBOT_DATA_LOGGER_SOURCES
   wire/cdr_buffer.cpp
   wire/lz4_block.cpp
   handshake/json_serialization.cpp
   handshake/yo_variable_hand_shake_builder.cpp
   http/http_server.cpp
)

add_library(ihmc_robot_data_logger ${IHMC_ROBOT_DATA_LOGGER_SOURCES})
add_library(ihmc::robot_data_logger ALIAS ihmc_robot_data_logger)

target_include_directories(ihmc_robot_data_logger PUBLIC
   $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
)

target_compile_features(ihmc_robot_data_logger PUBLIC cxx_std_17)
target_link_libraries(ihmc_robot_data_logger PUBLIC ihmc::yovariables nlohmann_json::nlohmann_json asio_standalone Threads::Threads)
target_link_libraries(ihmc_robot_data_logger PRIVATE lz4_vendored)
