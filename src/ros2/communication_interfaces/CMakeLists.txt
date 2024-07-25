cmake_minimum_required(VERSION 3.5)
project(communication_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AuthenticateUser.action"
  "action/CreateUserNfcTag.action"
  "action/ChangeFootprint.action"
  "action/ChangeFootprintPadding.action"
  "action/ComputeInterimGoal.action"
  "action/DrawerInteraction.action"
  "action/DrawerUserAccess.action"
  "action/ImportYamlPoses.action"

  "srv/HandleDrawer.srv"
  "srv/ShelfSetupInfo.srv"
  "srv/ShowDrawerInterface.srv"

  "msg/Box.msg"
  "msg/DrawerAddress.msg"
  "msg/DrawerStatus.msg"
  "msg/DrawerTask.msg"
  "msg/ElectricalDrawerStatus.msg"
  "msg/Led.msg"
  "msg/LedCmd.msg"
  "msg/Module.msg"
  "msg/ModuleConfig.msg"
  "msg/NfcReadStatus.msg"
  "msg/NfcWriteStatus.msg"
  "msg/Task.msg"
  "msg/TaskStatus.msg"
  "msg/Ticket.msg"
  "msg/TrayTask.msg"

  "error_msgs/ErrorBaseMsg.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
