# Sources are added explicitly (no globbing) as each port phase lands, so CMake always
# re-runs when a file is added rather than silently missing it.
set(IHMC_YOVARIABLES_SOURCES
   version.cpp
)

add_library(ihmc_yovariables ${IHMC_YOVARIABLES_SOURCES})
add_library(ihmc::yovariables ALIAS ihmc_yovariables)

target_include_directories(ihmc_yovariables PUBLIC
   $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
)

target_compile_features(ihmc_yovariables PUBLIC cxx_std_17)
