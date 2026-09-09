include(FetchContent)
FetchContent_Declare(
   magic_enum
   GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
   GIT_TAG v0.9.7
)
FetchContent_MakeAvailable(magic_enum)

FetchContent_Declare(
   pugixml
   GIT_REPOSITORY https://github.com/zeux/pugixml.git
   GIT_TAG v1.14
)
FetchContent_MakeAvailable(pugixml)

# Sources are added explicitly (no globbing) as each port phase lands, so CMake always
# re-runs when a file is added rather than silently missing it.
set(IHMC_YOVARIABLES_SOURCES
   version.cpp
   tools/yo_tools.cpp
   tools/yo_search_tools.cpp
   registry/yo_namespace.cpp
   registry/yo_registry.cpp
   variable/yo_variable.cpp
   variable/yo_boolean.cpp
   variable/yo_double.cpp
   variable/yo_integer.cpp
   variable/yo_long.cpp
   buffer/yo_buffer_bounds.cpp
   buffer/yo_buffer_variable_entry.cpp
   buffer/key_points_handler.cpp
   buffer/yo_buffer.cpp
   parameters/yo_parameter.cpp
   parameters/boolean_parameter.cpp
   parameters/double_parameter.cpp
   parameters/integer_parameter.cpp
   parameters/long_parameter.cpp
   parameters/parameter_data.cpp
   parameters/abstract_parameter_reader.cpp
   parameters/abstract_parameter_writer.cpp
   parameters/default_parameter_reader.cpp
   parameters/single_parameter_reader.cpp
   parameters/xml/parameter.cpp
   parameters/xml/registry.cpp
   parameters/xml/parameters.cpp
   parameters/xml_parameter_reader.cpp
   parameters/xml_parameter_writer.cpp
   tools/yo_geometry_name_tools.cpp
   tools/yo_factories.cpp
   filters/alpha_filter_tools.cpp
   filters/alpha_based_on_break_frequency_provider.cpp
   filters/alpha_filtered_yo_variable.cpp
   filters/alpha_filtered_wrapping_yo_variable.cpp
   filters/alpha_beta_filtered_yo_variable.cpp
   filters/acceleration_limited_yo_variable.cpp
   filters/backlash_processing_yo_variable.cpp
   filters/backlash_compensating_velocity_yo_variable.cpp
   filters/butterworth_filtered_yo_variable.cpp
   filters/deadbanded_yo_variable.cpp
   filters/delayed_yo_boolean.cpp
   filters/delayed_yo_double.cpp
   filters/delta_limited_yo_variable.cpp
   filters/filtered_finite_difference_yo_variable.cpp
   filters/filtered_discrete_velocity_yo_variable.cpp
   filters/first_order_filtered_yo_double.cpp
   filters/first_order_band_pass_filtered_yo_double.cpp
   filters/glitch_filtered_yo_boolean.cpp
   filters/glitch_filtered_yo_integer.cpp
   filters/jerk_limited_yo_double.cpp
   filters/rate_limited_yo_variable.cpp
   filters/running_average_yo_double.cpp
   filters/second_order_filtered_yo_variable_parameters.cpp
   filters/second_order_filtered_yo_double.cpp
   filters/simple_moving_average_filtered_yo_variable.cpp
)

add_library(ihmc_yovariables ${IHMC_YOVARIABLES_SOURCES})
add_library(ihmc::yovariables ALIAS ihmc_yovariables)

target_include_directories(ihmc_yovariables PUBLIC
   $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
)

target_compile_features(ihmc_yovariables PUBLIC cxx_std_17)
target_link_libraries(ihmc_yovariables PUBLIC magic_enum::magic_enum pugixml::pugixml)
