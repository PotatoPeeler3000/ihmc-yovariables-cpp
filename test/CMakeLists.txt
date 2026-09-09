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
   phase4_smoke_test.cpp
   phase5_smoke_test.cpp
   variable/yo_boolean_test.cpp
   variable/yo_double_test.cpp
   variable/yo_integer_test.cpp
   variable/yo_long_test.cpp
   variable/yo_enum_test.cpp
   variable/yo_variable_test.cpp
   registry/yo_namespace_test.cpp
   registry/yo_factories_test.cpp
   registry/yo_tools_test.cpp
   registry/yo_registry_test.cpp
   registry/yo_variable_list_test.cpp
   registry/yo_variable_holder_implementation_test.cpp
   registry/yo_variable_holder_implementation_new_test.cpp
   buffer/key_points_handler_test.cpp
   buffer/yo_buffer_variable_entry_test.cpp
   buffer/yo_buffer_test.cpp
   parameters/double_parameter_test.cpp
   parameters/boolean_parameter_test.cpp
   parameters/integer_parameter_test.cpp
   parameters/long_parameter_test.cpp
   parameters/enum_parameter_test.cpp
   parameters/abstract_parameter_reader_test.cpp
   parameters/default_parameter_reader_test.cpp
   parameters/xml_parameter_io_test.cpp
)

find_package(Threads REQUIRED)

add_executable(ihmc_yovariables_tests ${IHMC_YOVARIABLES_TEST_SOURCES})
target_link_libraries(ihmc_yovariables_tests PRIVATE ihmc::yovariables GTest::gtest_main Threads::Threads)

include(GoogleTest)
gtest_discover_tests(ihmc_yovariables_tests)
