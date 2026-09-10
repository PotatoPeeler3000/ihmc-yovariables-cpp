cmake_minimum_required(VERSION 3.16)
project(ihmc_yovariables CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

option(YOVARIABLES_BUILD_TESTS "Build unit tests" ON)
option(YOVARIABLES_BUILD_SERVER "Build the YoVariableServer networking library (ihmc_robot_data_logger)" ON)
option(YOVARIABLES_BUILD_EXAMPLES "Build runnable examples (requires YOVARIABLES_BUILD_SERVER)" ON)

add_subdirectory(src)

if(YOVARIABLES_BUILD_SERVER)
   add_subdirectory(src/robotDataLogger)

   if(YOVARIABLES_BUILD_EXAMPLES)
      add_subdirectory(examples)
   endif()
endif()

if(YOVARIABLES_BUILD_TESTS)
   enable_testing()
   add_subdirectory(test)
endif()
