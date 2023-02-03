cmake_minimum_required(VERSION 3.8)
project(mbari_wec_template_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(buoy_interfaces REQUIRED)
find_package(buoy_api_cpp REQUIRED COMPONENTS buoy_api)

add_executable(controller src/controller.cpp)
target_link_libraries(controller PUBLIC buoy_api_cpp::buoy_api)
ament_target_dependencies(controller PUBLIC rclcpp buoy_interfaces)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

###########################################

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

###########################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
