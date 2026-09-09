include(FetchContent)
FetchContent_Declare(
   googletest
   GIT_REPOSITORY https://github.com/google/googletest.git
   GIT_TAG v1.15.2
)
# Windows: link against the DLL runtime so gtest and this project use the same CRT.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Test sources are added explicitly, one line per ported JUnit test class, as each lands.
set(IHMC_YOVARIABLES_TEST_SOURCES
   version_test.cpp
   phase1_smoke_test.cpp
   phase2_smoke_test.cpp
   phase3_smoke_test.cpp
)

find_package(Threads REQUIRED)

add_executable(ihmc_yovariables_tests ${IHMC_YOVARIABLES_TEST_SOURCES})
target_link_libraries(ihmc_yovariables_tests PRIVATE ihmc::yovariables GTest::gtest_main Threads::Threads)

include(GoogleTest)
gtest_discover_tests(ihmc_yovariables_tests)
