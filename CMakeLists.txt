cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

project(robodog)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all necessary ROS 2 and system packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost COMPONENTS filesystem thread REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find the serial driver and io_context packages
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)



# Define all the source files for our static library
set(DOG_LIB_SOURCES
    "src/IK.cpp"
    "src/Leg.cpp"
    "src/Joints/JointSingle.cpp" "src/Joints/JointDouble.cpp"
    "src/Servos/LX16a.cpp" "src/Servos/Servo.cpp" "src/Servos/ServoLX16a.cpp" "src/Servos/ServoController.cpp"
    "src/Robots/RobotV2.cpp" "src/Robots/RobotV3.cpp"
    "src/Trajectory/MoveParams.cpp" "src/Trajectory/BaseState.cpp" "src/Trajectory/GCodeController.cpp" "src/Trajectory/MovePrimitives.cpp"
    "src/RemoteControl/Button.cpp" "src/RemoteControl/CommandMenu.cpp" "src/RemoteControl/RemoteController.cpp"
    "src/Utils.cpp" "src/MathHelpers.cpp" "src/Filters.cpp"
)

# Add include directories
include_directories(
    "include"
    ${Boost_INCLUDE_DIRS}
)

# Create the static library from our source files
add_library(${PROJECT_NAME}_lib STATIC ${DOG_LIB_SOURCES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  serial_driver
  io_context
  yaml-cpp
  ${rosidl_get_targets_for_interface_packages}
)

# Create the main hardware node executable
add_executable(${PROJECT_NAME}_hw
  "src/Services/RobotHW.cpp"
)

# Link the hardware node against its dependencies
ament_target_dependencies(${PROJECT_NAME}_hw rclcpp std_msgs serial_driver io_context)
target_link_libraries(${PROJECT_NAME}_hw
    ${PROJECT_NAME}_lib
    ${Boost_LIBRARIES}
    ${rosidl_get_libraries_for_interface_packages}
)

# Create the client executable
add_executable(${PROJECT_NAME}_client "src/Services/RobotClient.cpp")
ament_target_dependencies(${PROJECT_NAME}_client rclcpp std_msgs)
# The client also needs to link to the generated service code
target_link_libraries(${PROJECT_NAME}_client
  ${rosidl_get_libraries_for_interface_packages}
)

# Create the remote controller executable
add_executable(${PROJECT_NAME}_remote_controller
  "src/RemoteControl/RemoteController.cpp"
)

# Link the remote controller against its dependencies
ament_target_dependencies(${PROJECT_NAME}_remote_controller rclcpp sensor_msgs yaml-cpp)
target_link_libraries(${PROJECT_NAME}_remote_controller
  ${PROJECT_NAME}_lib
  yaml-cpp
  ${rosidl_get_libraries_for_interface_packages}
)

# Create the servo controller diagnostics executable
add_executable(${PROJECT_NAME}_servo_diagnostics
  "src/Services/ServoControllerDiagnostics.cpp"
)

# Link the servo diagnostics against its dependencies
ament_target_dependencies(${PROJECT_NAME}_servo_diagnostics rclcpp serial_driver io_context)
target_link_libraries(${PROJECT_NAME}_servo_diagnostics
  ${PROJECT_NAME}_lib
  ${rosidl_get_libraries_for_interface_packages}
)

# Create the servo scanner executable
add_executable(${PROJECT_NAME}_servo_scanner
  "src/Services/ServoScanner.cpp"
)

# Link the servo scanner against its dependencies
ament_target_dependencies(${PROJECT_NAME}_servo_scanner rclcpp serial_driver io_context)
target_link_libraries(${PROJECT_NAME}_servo_scanner
  ${PROJECT_NAME}_lib
  ${rosidl_get_libraries_for_interface_packages}
)

# Install the executables so they can be found by `ros2 run`
install(TARGETS
  ${PROJECT_NAME}_hw
  ${PROJECT_NAME}_client
  ${PROJECT_NAME}_remote_controller
  ${PROJECT_NAME}_servo_diagnostics
  ${PROJECT_NAME}_servo_scanner
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_dependencies(rclcpp std_msgs sensor_msgs serial_driver io_context yaml-cpp)

# Finalize the package
ament_package()
