# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(comrob2023_formation CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

set(PLUGINS
  EspMotorModel EspPowerDistribution EspPositionController
  EspAttitudeController EspStaController EspTcController
  EspStsmcController EspNtsmcController)

foreach(plugin ${PLUGINS})
  add_library(${plugin} SHARED plugins/${plugin}.cc)
  target_include_directories(${plugin} PRIVATE plugins)
  target_link_libraries(${plugin} PRIVATE
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-plugin2::register
    gz-math7::gz-math7
    gz-common5::gz-common5)
endforeach()

install(TARGETS ${PLUGINS} LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch models worlds DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS nodes/formation_node.py nodes/control_panel.py DESTINATION lib/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/50.${PROJECT_NAME}.dsv.in")

ament_package()
