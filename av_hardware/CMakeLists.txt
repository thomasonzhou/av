cmake_minimum_required(VERSION 3.8)
project(av_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/av_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
)
ament_export_libraries(${PROJECT_NAME})
pluginlib_export_plugin_description_file(hardware_interface av_hardware.xml)

install(DIRECTORY include/
  DESTINATION include
)
install(FILES
  av_hardware.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gmock(test_av_hardware
    test/test_av_hardware.cpp
  )

  target_link_libraries(test_av_hardware
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_av_hardware
    hardware_interface
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
